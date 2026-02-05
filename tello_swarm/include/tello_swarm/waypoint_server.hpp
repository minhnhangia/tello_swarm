#ifndef TELLO_SWARM__WAYPOINT_SERVER__HPP_
#define TELLO_SWARM__WAYPOINT_SERVER__HPP_

#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/srv/reserve_waypoint.hpp"
#include "swarm_interfaces/msg/waypoint_heartbeat.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <string>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <cstdint>

namespace tello_swarm
{

/**
 * @brief Represents a single waypoint reservation in the swarm coordination system.
 * 
 * Each waypoint can be in one of two states:
 * - FREE: Available for reservation
 * - RESERVED: Currently reserved by a drone
 */
struct WaypointReservation
{
    enum State : uint8_t
    {
        FREE = 0,
        RESERVED = 1
    };

    uint32_t waypoint_index;                          ///< Waypoint sequence index (0-based)
    State state;                                       ///< Current reservation state
    std::string owner;                                 ///< Drone ID that owns the reservation
    std::chrono::steady_clock::time_point last_update; ///< Last heartbeat timestamp

    WaypointReservation(uint32_t idx)
        : waypoint_index(idx),
          state(FREE),
          owner(""),
          last_update(std::chrono::steady_clock::now())
    {
    }
};

/**
 * @brief Centralized waypoint reservation server for drone swarm coordination.
 * 
 * Manages mutual exclusion for waypoints to prevent collisions. Key features:
 * - Reserve/unreserve services for exclusive waypoint access
 * - Heartbeat-based reservation renewal (drones must send periodic heartbeats)
 * - Automatic expiration of stale reservations (8 seconds without heartbeat)
 * - Publishes list of unavailable waypoints for monitoring
 * 
 * Similar to MarkerManager but simplified for transient waypoint access.
 */
class WaypointServer : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Waypoint Server.
     * 
     * Initializes service servers, subscribers, publishers, and timers.
     * Default supports 100 waypoints (configurable via parameter).
     */
    WaypointServer();

private:
    // ========================================================================
    // SERVICE HANDLERS
    // ========================================================================

    /**
     * @brief Handle waypoint reservation request.
     * 
     * Attempts to reserve a waypoint for exclusive drone access.
     * 
     * @param request Contains waypoint_index and drone_id
     * @param response Returns success status and message
     */
    void handle_reserve_waypoint(
        const std::shared_ptr<swarm_interfaces::srv::ReserveWaypoint::Request> request,
        std::shared_ptr<swarm_interfaces::srv::ReserveWaypoint::Response> response);

    /**
     * @brief Handle waypoint unreservation request.
     * 
     * Releases a waypoint reservation, making it available for other drones.
     * Only the owning drone can unreserve.
     * 
     * @param request Contains waypoint_index and drone_id
     * @param response Returns success status and message
     */
    void handle_unreserve_waypoint(
        const std::shared_ptr<swarm_interfaces::srv::ReserveWaypoint::Request> request,
        std::shared_ptr<swarm_interfaces::srv::ReserveWaypoint::Response> response);

    /**
     * @brief Handle heartbeat messages from drones.
     * 
     * Renews reservation timestamp to prevent expiration.
     * 
     * @param msg Heartbeat containing drone_id and waypoint_index
     */
    void handle_waypoint_heartbeat(
        const swarm_interfaces::msg::WaypointHeartbeat::SharedPtr msg);

    // ========================================================================
    // INTERNAL LOGIC
    // ========================================================================

    /**
     * @brief Initialize waypoint registry with N waypoints.
     * 
     * Creates WaypointReservation objects for indices 0 to max_waypoints-1.
     * 
     * @param max_waypoints Number of waypoints to support
     */
    void populate_waypoint_registry(uint32_t max_waypoints);

    /**
     * @brief Expire reservations that haven't received heartbeats.
     * 
     * Scans registry and frees reservations older than expiration_duration_.
     * Called periodically and before processing service requests.
     */
    void cleanup_expired_reservations();

    /**
     * @brief Find waypoint reservation by index.
     * 
     * @param waypoint_index Index to search for
     * @return Iterator to waypoint, or end() if not found
     */
    std::vector<WaypointReservation>::iterator find_waypoint(int waypoint_index);

    /**
     * @brief Timer callback for periodic tasks.
     * 
     * Triggers cleanup and publishes unavailable waypoints list.
     */
    void timer_callback_();

    /**
     * @brief Publish list of unavailable waypoints.
     * 
     * Publishes indices of all RESERVED waypoints to /unavailable_waypoints topic.
     * Uses transient_local QoS so late subscribers receive last state.
     */
    void publish_unavailable_waypoints();

    /**
     * @brief Immediate publish on state change.
     * 
     * Called after reserve/unreserve operations to provide real-time updates.
     */
    void publish_unavailable_waypoints_on_update();

    // ========================================================================
    // ROS2 COMMUNICATION
    // ========================================================================

    /// Service for reserving waypoints
    rclcpp::Service<swarm_interfaces::srv::ReserveWaypoint>::SharedPtr reserve_waypoint_srv_;

    /// Service for unreserving waypoints
    rclcpp::Service<swarm_interfaces::srv::ReserveWaypoint>::SharedPtr unreserve_waypoint_srv_;

    /// Subscription to heartbeat messages
    rclcpp::Subscription<swarm_interfaces::msg::WaypointHeartbeat>::SharedPtr heartbeat_sub_;

    /// Publisher for unavailable waypoints (transient_local for late joiners)
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr unavailable_waypoints_pub_;

    /// Timer for periodic cleanup and publishing
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // ========================================================================
    // STATE MANAGEMENT
    // ========================================================================

    /// Registry of all waypoint reservations
    std::vector<WaypointReservation> waypoint_registry_;

    /// Duration after which a reservation expires without heartbeat (default: 8s)
    std::chrono::seconds expiration_duration_{8};

    /// Mutex for thread-safe access to waypoint registry
    std::mutex registry_mutex_;
};

} // namespace tello_swarm

#endif // TELLO_SWARM__WAYPOINT_SERVER__HPP_
