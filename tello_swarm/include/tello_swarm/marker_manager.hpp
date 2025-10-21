#ifndef TELLO_SWARM__MARKER_MANAGER__HPP_
#define TELLO_SWARM__MARKER_MANAGER__HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "swarm_interfaces/srv/reserve_marker.hpp"
#include "swarm_interfaces/srv/mark_landed.hpp"

#include <unordered_map>
#include <string>
#include <chrono>
#include <vector>
#include <cstdint>

namespace tello_swarm
{

struct Marker {
    uint32_t marker_id;
    enum State { FREE, RESERVED, LANDED } state;
    enum Type { VICTIM, FIRE } type;
    std::string owner;      // which drone reserved it
    std::chrono::steady_clock::time_point last_update;
};

class MarkerManager : public rclcpp::Node
{
public:
    MarkerManager();

private:
    // === Publishers ===
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr unavailable_markers_pub_;

    // === Service servers ===
    rclcpp::Service<swarm_interfaces::srv::ReserveMarker>::SharedPtr reserve_marker_srv_;
    rclcpp::Service<swarm_interfaces::srv::ReserveMarker>::SharedPtr unreserve_marker_srv_;
    rclcpp::Service<swarm_interfaces::srv::MarkLanded>::SharedPtr mark_landed_srv_;

    // === Marker state ===
    // Pre-populated list of known markers and their state
    std::vector<Marker> marker_registry_;
    std::chrono::seconds cooldown_duration_{10};    // Duration after which a RESERVED marker becomes FREE if not updated

    // === Timer ===
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // === Internal methods ===
    void populate_marker_registry();

    void handle_reserve_marker(
        const std::shared_ptr<swarm_interfaces::srv::ReserveMarker::Request> request,
        std::shared_ptr<swarm_interfaces::srv::ReserveMarker::Response> response);

    void handle_unreserve_marker(
        const std::shared_ptr<swarm_interfaces::srv::ReserveMarker::Request> request,
        std::shared_ptr<swarm_interfaces::srv::ReserveMarker::Response> response);

    void handle_mark_landed(
        const std::shared_ptr<swarm_interfaces::srv::MarkLanded::Request> request,
        std::shared_ptr<swarm_interfaces::srv::MarkLanded::Response> response);

    void publish_unavailable_markers();
    void publish_unavailable_markers_on_update();
    void cleanup_expired_markers();
    std::vector<Marker>::iterator find_marker(int marker_id);
};

} // namespace tello_swarm

#endif // TELLO_SWARM__MARKER_MANAGER__HPP_
