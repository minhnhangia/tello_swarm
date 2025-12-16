#ifndef TELLO_SWARM__WAYPOINT_SERVER__HPP_
#define TELLO_SWARM__WAYPOINT_SERVER__HPP_

#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/srv/query_turn.hpp"
#include "swarm_interfaces/srv/signal_complete.hpp"
#include "swarm_interfaces/srv/report_failure.hpp"

#include <string>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <cstdint>

namespace tello_swarm
{

struct WaypointState
{
    int waypoint_id;
    std::vector<std::string> drone_queue;               // ordered list of drones needing this waypoint
    size_t current_turn_index;                          // which drone's turn in the queue
    std::string current_drone;                          // drone currently executing
    std::chrono::steady_clock::time_point turn_started; // when current turn began
    std::unordered_map<std::string, bool> completed_by; // track completions per drone
};

class WaypointServer : public rclcpp::Node
{
public:
    WaypointServer();

private:
    // === Service servers ===
    rclcpp::Service<swarm_interfaces::srv::QueryTurn>::SharedPtr query_turn_srv_;
    rclcpp::Service<swarm_interfaces::srv::SignalComplete>::SharedPtr signal_complete_srv_;
    rclcpp::Service<swarm_interfaces::srv::ReportFailure>::SharedPtr report_failure_srv_;

    // === Waypoint state (per-waypoint tracking) ===
    std::unordered_map<int, WaypointState> waypoint_states_;
    std::chrono::seconds turn_timeout_{30}; // timeout before auto-advancing turn
    std::mutex mutex_;

    // === Timer ===
    rclcpp::TimerBase::SharedPtr timeout_check_timer_;

    // === Internal methods ===
    void build_waypoint_queues();
    void check_timeouts();

    void handle_query_turn(
        const std::shared_ptr<swarm_interfaces::srv::QueryTurn::Request> request,
        std::shared_ptr<swarm_interfaces::srv::QueryTurn::Response> response);

    void handle_signal_complete(
        const std::shared_ptr<swarm_interfaces::srv::SignalComplete::Request> request,
        std::shared_ptr<swarm_interfaces::srv::SignalComplete::Response> response);

    void handle_report_failure(
        const std::shared_ptr<swarm_interfaces::srv::ReportFailure::Request> request,
        std::shared_ptr<swarm_interfaces::srv::ReportFailure::Response> response);

    bool is_turn_expired(const WaypointState &state) const;
    void advance_to_next_turn(int waypoint_id);
};

} // namespace tello_swarm

#endif // TELLO_SWARM__WAYPOINT_SERVER__HPP_
