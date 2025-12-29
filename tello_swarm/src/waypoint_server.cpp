#include "tello_swarm/waypoint_server.hpp"

#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std::chrono_literals;

namespace tello_swarm
{

WaypointServer::WaypointServer()
    : Node("waypoint_server")
{
    build_waypoint_queues();

    // Create service servers
    query_turn_srv_ = this->create_service<swarm_interfaces::srv::QueryTurn>(
        "/query_turn",
        std::bind(&WaypointServer::handle_query_turn, this, std::placeholders::_1, std::placeholders::_2));

    signal_complete_srv_ = this->create_service<swarm_interfaces::srv::SignalComplete>(
        "/signal_complete",
        std::bind(&WaypointServer::handle_signal_complete, this, std::placeholders::_1, std::placeholders::_2));

    report_failure_srv_ = this->create_service<swarm_interfaces::srv::ReportFailure>(
        "/report_failure",
        std::bind(&WaypointServer::handle_report_failure, this, std::placeholders::_1, std::placeholders::_2));

    // Timer for checking timeouts
    timeout_check_timer_ = this->create_wall_timer(
        1s, std::bind(&WaypointServer::check_timeouts, this));

    RCLCPP_INFO(this->get_logger(), "Waypoint Server started with %zu waypoints configured.",
                waypoint_states_.size());

    // Log initial configuration
    for (const auto &[waypoint_id, state] : waypoint_states_)
    {
        std::string queue_str;
        for (const auto &drone : state.drone_queue)
        {
            queue_str += drone + " ";
        }
        RCLCPP_INFO(this->get_logger(), "  Waypoint %d: queue=[%s]",
                    waypoint_id, queue_str.c_str());
    }
}

void WaypointServer::build_waypoint_queues()
{
    waypoint_states_.clear();

    // Declare parameters
    this->declare_parameter<int>("turn_timeout_sec", 30);
    this->declare_parameter<std::string>("config_file", "");

    // Read parameters
    int timeout_sec = this->get_parameter("turn_timeout_sec").as_int();
    turn_timeout_ = std::chrono::seconds(timeout_sec);

    std::string config_file = this->get_parameter("config_file").as_string();

    if (config_file.empty())
    {
        RCLCPP_ERROR(this->get_logger(),
                        "No config_file parameter provided. Cannot build waypoint queues.");
        return;
    }

    // Parse YAML configuration
    try
    {
        YAML::Node config = YAML::LoadFile(config_file);

        if (!config["drones"])
        {
            RCLCPP_ERROR(this->get_logger(), "No 'drones' section in config file.");
            return;
        }

        // Build map of waypoint_id -> list of drones that need it
        std::unordered_map<int, std::vector<std::string>> waypoint_to_drones;

        // First pass: collect all drones and their waypoints
        for (auto drone_it = config["drones"].begin(); drone_it != config["drones"].end(); ++drone_it)
        {
            std::string drone_id = drone_it->first.as<std::string>();

            if (!drone_it->second["waypoints"])
            {
                RCLCPP_WARN(this->get_logger(), "Drone %s has no waypoints defined.", drone_id.c_str());
                continue;
            }

            const YAML::Node &waypoints = drone_it->second["waypoints"];
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                int waypoint_id = waypoints[i]["id"].as<int>();
                waypoint_to_drones[waypoint_id].push_back(drone_id);
            }
        }

        // Read initial order if provided
        std::vector<std::string> initial_order;
        if (config["waypoint_coordination"] && config["waypoint_coordination"]["initial_order"])
        {
            initial_order = config["waypoint_coordination"]["initial_order"].as<std::vector<std::string>>();
        }

        // Second pass: create WaypointState for each waypoint
        const auto now = std::chrono::steady_clock::now();
        for (const auto &[waypoint_id, drones] : waypoint_to_drones)
        {
            WaypointState state;
            state.waypoint_id = waypoint_id;
            state.drone_queue = drones;

            // Sort queue by initial_order if provided
            if (!initial_order.empty())
            {
                std::sort(state.drone_queue.begin(), state.drone_queue.end(),
                            [&initial_order](const std::string &a, const std::string &b)
                            {
                                auto it_a = std::find(initial_order.begin(), initial_order.end(), a);
                                auto it_b = std::find(initial_order.begin(), initial_order.end(), b);

                                // If both found in initial_order, use that order
                                if (it_a != initial_order.end() && it_b != initial_order.end())
                                {
                                    return it_a < it_b;
                                }
                                // If only one found, prioritize it
                                if (it_a != initial_order.end())
                                    return true;
                                if (it_b != initial_order.end())
                                    return false;
                                // Otherwise, alphabetical
                                return a < b;
                            });
            }
            else
            {
                // Default to alphabetical order
                std::sort(state.drone_queue.begin(), state.drone_queue.end());
            }

            state.current_turn_index = 0;
            state.current_drone = state.drone_queue.empty() ? "" : state.drone_queue[0];
            state.turn_started = now;

            // Initialize completed_by map
            for (const auto &drone : state.drone_queue)
            {
                state.completed_by[drone] = false;
            }

            waypoint_states_[waypoint_id] = state;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Waypoint queues built from config: %s (timeout=%ds)",
                    config_file.c_str(), timeout_sec);
    }
    catch (const YAML::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading config: %s", e.what());
    }
}

void WaypointServer::handle_query_turn(
    const std::shared_ptr<swarm_interfaces::srv::QueryTurn::Request> request,
    std::shared_ptr<swarm_interfaces::srv::QueryTurn::Response> response)
{
    std::lock_guard<std::mutex> lock(mutex_);

    int waypoint_id = request->waypoint_id;
    std::string drone_id = request->drone_id;

    // Check if waypoint exists
    if (waypoint_states_.find(waypoint_id) == waypoint_states_.end())
    {
        response->is_my_turn = false;
        response->already_completed = false;
        response->current_turn_holder = "";
        response->queue_position = -1;
        response->time_remaining = 0.0;
        response->message = "Waypoint not in mission plan";
        return;
    }

    auto &state = waypoint_states_[waypoint_id];

    // Check if already completed
    if (state.completed_by[drone_id])
    {
        response->is_my_turn = false;
        response->already_completed = true;
        response->current_turn_holder = state.current_drone;
        response->queue_position = -1;
        response->time_remaining = 0.0;
        response->message = "Already completed this waypoint";
        return;
    }

    // Find drone's position in queue
    auto it = std::find(state.drone_queue.begin(), state.drone_queue.end(), drone_id);
    if (it == state.drone_queue.end())
    {
        response->is_my_turn = false;
        response->already_completed = false;
        response->current_turn_holder = state.current_drone;
        response->queue_position = -1;
        response->time_remaining = 0.0;
        response->message = "Drone not in queue for this waypoint";
        return;
    }

    int position = std::distance(state.drone_queue.begin(), it);
    response->queue_position = position;
    response->already_completed = false;

    // Check if current turn expired - auto-advance if needed
    if (is_turn_expired(state) && state.current_drone != drone_id)
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::steady_clock::now() - state.turn_started)
                            .count();
        RCLCPP_WARN(this->get_logger(),
                    "Drone %s timeout at waypoint %d (%lds elapsed), advancing turn",
                    state.current_drone.c_str(), waypoint_id, elapsed);
        advance_to_next_turn(waypoint_id);
    }

    // Check if it's this drone's turn
    response->is_my_turn = (state.current_turn_index == static_cast<size_t>(position));
    response->current_turn_holder = state.current_drone;

    if (response->is_my_turn)
    {
        response->message = "Your turn - proceed to waypoint";
        response->time_remaining = 0.0;
    }
    else
    {
        auto elapsed = std::chrono::steady_clock::now() - state.turn_started;
        float remaining = std::chrono::duration<float>(turn_timeout_ - elapsed).count();
        response->time_remaining = std::max(0.0f, remaining);
        response->message = "Wait for your turn";
    }
}

void WaypointServer::handle_signal_complete(
    const std::shared_ptr<swarm_interfaces::srv::SignalComplete::Request> request,
    std::shared_ptr<swarm_interfaces::srv::SignalComplete::Response> response)
{
    std::lock_guard<std::mutex> lock(mutex_);

    int waypoint_id = request->waypoint_id;
    std::string drone_id = request->drone_id;

    if (waypoint_states_.find(waypoint_id) == waypoint_states_.end())
    {
        response->success = false;
        response->next_turn_holder = "";
        response->remaining_drones = 0;
        response->message = "Waypoint not found";
        return;
    }

    auto &state = waypoint_states_[waypoint_id];

    // Verify it's this drone's turn
    if (state.current_drone != drone_id)
    {
        response->success = false;
        response->next_turn_holder = state.current_drone;
        response->remaining_drones = 0;
        response->message = "Not your turn (current holder: " + state.current_drone + ")";
        RCLCPP_WARN(this->get_logger(),
                    "[%s] attempted to complete waypoint %d but it's %s's turn",
                    drone_id.c_str(), waypoint_id, state.current_drone.c_str());
        return;
    }

    // Mark as completed
    state.completed_by[drone_id] = true;

    RCLCPP_INFO(this->get_logger(),
                "[%s] completed waypoint %d",
                drone_id.c_str(), waypoint_id);

    // Advance to next drone
    advance_to_next_turn(waypoint_id);

    response->success = true;
    response->next_turn_holder = state.current_drone;

    // Count remaining drones that haven't completed
    int remaining = 0;
    for (const auto &[drone, completed] : state.completed_by)
    {
        if (!completed)
            remaining++;
    }
    response->remaining_drones = remaining;
    response->message = "Waypoint completed successfully";
}

void WaypointServer::handle_report_failure(
    const std::shared_ptr<swarm_interfaces::srv::ReportFailure::Request> request,
    std::shared_ptr<swarm_interfaces::srv::ReportFailure::Response> response)
{
    std::lock_guard<std::mutex> lock(mutex_);

    int waypoint_id = request->waypoint_id;
    std::string drone_id = request->drone_id;

    if (waypoint_states_.find(waypoint_id) == waypoint_states_.end())
    {
        response->success = false;
        response->next_turn_holder = "";
        response->message = "Waypoint not found";
        return;
    }

    auto &state = waypoint_states_[waypoint_id];

    RCLCPP_ERROR(this->get_logger(),
                    "[%s] failed at waypoint %d: %s",
                    drone_id.c_str(), waypoint_id, request->failure_reason.c_str());

    // Mark as "completed" (skipped) so drone won't get turn again
    state.completed_by[drone_id] = true;

    // Advance to next drone
    advance_to_next_turn(waypoint_id);

    response->success = true;
    response->next_turn_holder = state.current_drone;
    response->message = "Failure recorded, turn advanced";
}

bool WaypointServer::is_turn_expired(const WaypointState &state) const
{
    if (state.current_drone.empty())
        return false;

    auto elapsed = std::chrono::steady_clock::now() - state.turn_started;
    return elapsed > turn_timeout_;
}

void WaypointServer::advance_to_next_turn(int waypoint_id)
{
    auto &state = waypoint_states_[waypoint_id];

    // Find next drone that hasn't completed this waypoint
    size_t original_index = state.current_turn_index;
    do
    {
        state.current_turn_index++;

        if (state.current_turn_index >= state.drone_queue.size())
        {
            // All drones processed for this waypoint
            state.current_drone = "";
            RCLCPP_INFO(this->get_logger(),
                        "All drones completed/skipped waypoint %d", waypoint_id);
            return;
        }

        state.current_drone = state.drone_queue[state.current_turn_index];

        // Prevent infinite loop
        if (state.current_turn_index == original_index)
        {
            RCLCPP_ERROR(this->get_logger(),
                            "All drones in queue have completed waypoint %d, but loop detected",
                            waypoint_id);
            state.current_drone = "";
            return;
        }

    } while (state.completed_by[state.current_drone]); // skip already-completed drones

    state.turn_started = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(),
                "Turn advanced to %s for waypoint %d",
                state.current_drone.c_str(), waypoint_id);
}

void WaypointServer::check_timeouts()
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto &[waypoint_id, state] : waypoint_states_)
    {
        if (is_turn_expired(state) && !state.current_drone.empty())
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                std::chrono::steady_clock::now() - state.turn_started)
                                .count();
            RCLCPP_WARN(this->get_logger(),
                        "Timeout: Drone %s at waypoint %d (%lds), auto-advancing",
                        state.current_drone.c_str(), waypoint_id, elapsed);

            // Mark current drone as completed (failed) and advance
            state.completed_by[state.current_drone] = true;
            advance_to_next_turn(waypoint_id);
        }
    }
}

} // namespace tello_swarm

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tello_swarm::WaypointServer>());
    rclcpp::shutdown();
    return 0;
}
