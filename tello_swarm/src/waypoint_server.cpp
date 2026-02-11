#include "tello_swarm/waypoint_server.hpp"

#include <algorithm>

using namespace std::chrono_literals;

namespace tello_swarm
{

WaypointServer::WaypointServer()
    : Node("waypoint_server")
{
    // Declare parameters
    this->declare_parameter("max_waypoints", 100);
    this->declare_parameter("expiration_duration_s", 10.0);

    // Load parameters
    uint32_t max_waypoints = this->get_parameter("max_waypoints").as_int();
    double expiration_s = this->get_parameter("expiration_duration_s").as_double();
    expiration_duration_ = std::chrono::seconds(static_cast<int64_t>(expiration_s));

    RCLCPP_INFO(this->get_logger(),
                "Initializing Waypoint Server with %u waypoints, %.1fs expiration",
                max_waypoints, expiration_s);

    // Initialize waypoint registry
    populate_waypoint_registry(max_waypoints);

    // Publisher for unavailable waypoints (transient local for late subscribers)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.transient_local();
    unavailable_waypoints_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "/unavailable_waypoints", qos);

    // Subscribe to waypoint heartbeats from drones
    heartbeat_sub_ = this->create_subscription<swarm_interfaces::msg::WaypointHeartbeat>(
        "/waypoint_heartbeat",
        rclcpp::QoS(10).reliable(),
        std::bind(&WaypointServer::handle_waypoint_heartbeat, this, std::placeholders::_1));

    // Create service servers (global namespace for swarm-wide coordination)
    reserve_waypoint_srv_ = this->create_service<swarm_interfaces::srv::ReserveWaypoint>(
        "/reserve_waypoint",
        std::bind(&WaypointServer::handle_reserve_waypoint, this,
                  std::placeholders::_1, std::placeholders::_2));

    unreserve_waypoint_srv_ = this->create_service<swarm_interfaces::srv::ReserveWaypoint>(
        "/unreserve_waypoint",
        std::bind(&WaypointServer::handle_unreserve_waypoint, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Periodically publish unavailable waypoints and cleanup expired reservations
    publish_timer_ = this->create_wall_timer(
        500ms, std::bind(&WaypointServer::timer_callback_, this));

    RCLCPP_INFO(this->get_logger(), "Waypoint Server started and ready.");
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void WaypointServer::populate_waypoint_registry(uint32_t max_waypoints)
{
    std::lock_guard<std::mutex> lock(registry_mutex_);
    
    waypoint_registry_.clear();
    waypoint_registry_.reserve(max_waypoints);

    for (uint32_t idx = 0; idx < max_waypoints; ++idx)
    {
        waypoint_registry_.emplace_back(idx);
    }

    RCLCPP_INFO(this->get_logger(),
                "Waypoint registry initialized with %zu waypoints (indices 0-%u)",
                waypoint_registry_.size(), max_waypoints - 1);
}

// ============================================================================
// SERVICE HANDLERS
// ============================================================================

void WaypointServer::handle_reserve_waypoint(
    const std::shared_ptr<swarm_interfaces::srv::ReserveWaypoint::Request> request,
    std::shared_ptr<swarm_interfaces::srv::ReserveWaypoint::Response> response)
{
    const int waypoint_index = request->waypoint_index;
    const std::string &drone_id = request->drone_id;

    // Thread-safe cleanup before processing
    cleanup_expired_reservations();

    std::lock_guard<std::mutex> lock(registry_mutex_);
    auto it = find_waypoint(waypoint_index);

    // Validate waypoint index
    if (it == waypoint_registry_.end())
    {
        response->success = false;
        response->message = "Invalid waypoint index (out of range)";
        RCLCPP_WARN(this->get_logger(),
                    "[%s] attempted to reserve invalid waypoint %d",
                    drone_id.c_str(), waypoint_index);
        return;
    }

    // Case 1: Waypoint is free - grant reservation
    if (it->state == WaypointReservation::FREE)
    {
        it->state = WaypointReservation::RESERVED;
        it->owner = drone_id;
        it->last_update = std::chrono::steady_clock::now();
        response->success = true;
        response->message = "Waypoint reserved successfully";
        RCLCPP_INFO(this->get_logger(),
                    "[%s] reserved waypoint %d",
                    drone_id.c_str(), waypoint_index);
        publish_unavailable_waypoints_on_update();
        return;
    }

    // Case 2: Renewal by the same drone (idempotent operation)
    if (it->owner == drone_id && it->state == WaypointReservation::RESERVED)
    {
        it->last_update = std::chrono::steady_clock::now();
        response->success = true;
        response->message = "Reservation renewed";
        RCLCPP_DEBUG(this->get_logger(),
                     "[%s] renewed waypoint %d reservation",
                     drone_id.c_str(), waypoint_index);
        return;
    }

    // Case 3: Waypoint reserved by another drone
    response->success = false;
    response->message = "Waypoint reserved by " + it->owner;
    RCLCPP_DEBUG(this->get_logger(),
                 "[%s] failed to reserve waypoint %d (owned by %s)",
                 drone_id.c_str(), waypoint_index, it->owner.c_str());
}

void WaypointServer::handle_unreserve_waypoint(
    const std::shared_ptr<swarm_interfaces::srv::ReserveWaypoint::Request> request,
    std::shared_ptr<swarm_interfaces::srv::ReserveWaypoint::Response> response)
{
    const int waypoint_index = request->waypoint_index;
    const std::string &drone_id = request->drone_id;

    cleanup_expired_reservations();

    std::lock_guard<std::mutex> lock(registry_mutex_);
    auto it = find_waypoint(waypoint_index);

    // Validate waypoint index
    if (it == waypoint_registry_.end())
    {
        response->success = false;
        response->message = "Invalid waypoint index (out of range)";
        RCLCPP_WARN(this->get_logger(),
                    "[%s] attempted to unreserve invalid waypoint %d",
                    drone_id.c_str(), waypoint_index);
        return;
    }

    // Case 1: Waypoint is already free (idempotent operation)
    if (it->state == WaypointReservation::FREE)
    {
        response->success = true;
        response->message = "Waypoint already free";
        RCLCPP_DEBUG(this->get_logger(),
                     "[%s] unreserved already-free waypoint %d",
                     drone_id.c_str(), waypoint_index);
        return;
    }

    // Case 2: Verify ownership before unreserving
    if (it->owner != drone_id)
    {
        response->success = false;
        response->message = "Waypoint owned by " + it->owner + " (permission denied)";
        RCLCPP_WARN(this->get_logger(),
                    "[%s] attempted to unreserve waypoint %d owned by %s",
                    drone_id.c_str(), waypoint_index, it->owner.c_str());
        return;
    }

    // Case 3: Owner unreserves successfully
    it->state = WaypointReservation::FREE;
    it->owner.clear();
    it->last_update = std::chrono::steady_clock::now();
    response->success = true;
    response->message = "Waypoint unreserved successfully";
    RCLCPP_INFO(this->get_logger(),
                "[%s] unreserved waypoint %d",
                drone_id.c_str(), waypoint_index);
    publish_unavailable_waypoints_on_update();
}

void WaypointServer::handle_waypoint_heartbeat(
    const swarm_interfaces::msg::WaypointHeartbeat::SharedPtr msg)
{
    const int waypoint_index = msg->waypoint_index;
    const std::string &drone_id = msg->drone_id;

    std::lock_guard<std::mutex> lock(registry_mutex_);
    auto it = find_waypoint(waypoint_index);

    // Silently ignore invalid indices (high-frequency topic)
    if (it == waypoint_registry_.end())
    {
        return;
    }

    // Only renew if this drone owns the waypoint and it's reserved
    if (it->owner == drone_id && it->state == WaypointReservation::RESERVED)
    {
        it->last_update = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(this->get_logger(),
                     "[%s] heartbeat renewed waypoint %d",
                     drone_id.c_str(), waypoint_index);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "[%s] sent heartbeat for waypoint %d but does not own it (current owner: %s)",
                     drone_id.c_str(), waypoint_index,
                     it->owner.empty() ? "none" : it->owner.c_str());
    }
}

// ============================================================================
// INTERNAL LOGIC
// ============================================================================

void WaypointServer::cleanup_expired_reservations()
{
    std::lock_guard<std::mutex> lock(registry_mutex_);
    const auto now = std::chrono::steady_clock::now();
    bool changed = false;

    for (auto &wp : waypoint_registry_)
    {
        if (wp.state == WaypointReservation::RESERVED &&
            (now - wp.last_update) > expiration_duration_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Waypoint %u (reserved by %s) expired and is now FREE",
                        wp.waypoint_index, wp.owner.c_str());
            wp.state = WaypointReservation::FREE;
            wp.owner.clear();
            wp.last_update = now;
            changed = true;
        }
    }

    // Publish update if any reservations expired
    if (changed)
    {
        publish_unavailable_waypoints_on_update();
    }
}

std::vector<WaypointReservation>::iterator WaypointServer::find_waypoint(int waypoint_index)
{
    // Assumes registry_mutex_ is already locked by caller
    return std::find_if(waypoint_registry_.begin(), waypoint_registry_.end(),
                        [waypoint_index](const WaypointReservation &wp)
                        {
                            return static_cast<int>(wp.waypoint_index) == waypoint_index;
                        });
}

// ============================================================================
// PUBLISHING
// ============================================================================

void WaypointServer::timer_callback_()
{
    cleanup_expired_reservations();
    publish_unavailable_waypoints();
}

void WaypointServer::publish_unavailable_waypoints()
{
    std::lock_guard<std::mutex> lock(registry_mutex_);

    std_msgs::msg::Int32MultiArray msg;
    msg.data.reserve(waypoint_registry_.size());

    // Publish indices of all RESERVED waypoints
    for (const auto &wp : waypoint_registry_)
    {
        if (wp.state == WaypointReservation::RESERVED)
        {
            msg.data.push_back(static_cast<int32_t>(wp.waypoint_index));
        }
    }

    unavailable_waypoints_pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "Published %zu unavailable waypoints",
                 msg.data.size());
}

void WaypointServer::publish_unavailable_waypoints_on_update()
{
    // Immediate publish after state change (mutex already held by caller)
    // Note: This violates DRY slightly but avoids recursive locking
    std_msgs::msg::Int32MultiArray msg;
    msg.data.reserve(waypoint_registry_.size());

    for (const auto &wp : waypoint_registry_)
    {
        if (wp.state == WaypointReservation::RESERVED)
        {
            msg.data.push_back(static_cast<int32_t>(wp.waypoint_index));
        }
    }

    unavailable_waypoints_pub_->publish(msg);
}

} // namespace tello_swarm

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tello_swarm::WaypointServer>());
    rclcpp::shutdown();
    return 0;
}
