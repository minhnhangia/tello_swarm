#include "tello_swarm/marker_manager.hpp"

#include <algorithm>

using namespace std::chrono_literals;

namespace tello_swarm
{

MarkerManager::MarkerManager()
    : Node("marker_manager")
{
    populate_marker_registry();

    // Publisher for unavailable markers (transient local so late subscribers get last state)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();
    unavailable_markers_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "unavailable_markers", qos);

    // Create service servers
    reserve_marker_srv_ = this->create_service<swarm_interfaces::srv::ReserveMarker>(
        "reserve_marker",
        std::bind(&MarkerManager::handle_reserve_marker, this, std::placeholders::_1, std::placeholders::_2));

    mark_landed_srv_ = this->create_service<swarm_interfaces::srv::MarkLanded>(
        "mark_landed",
        std::bind(&MarkerManager::handle_mark_landed, this, std::placeholders::_1, std::placeholders::_2));

    // Periodically publish unavailable markers
    publish_timer_ = this->create_wall_timer(
        1s, std::bind(&MarkerManager::publish_unavailable_markers, this));

    RCLCPP_INFO(this->get_logger(), "Marker manager started.");
}

void MarkerManager::populate_marker_registry()
{
    // Initialize known markers [1..20] as FREE with default type (VICTIM)
    marker_registry_.reserve(20);
    for (uint32_t id = 1; id <= 20; ++id)
    {
        marker_registry_.push_back(Marker{
            id,
            Marker::FREE,
            Marker::VICTIM,
            "",
            std::chrono::steady_clock::now()});
    }
}

void MarkerManager::handle_reserve_marker(
    const std::shared_ptr<swarm_interfaces::srv::ReserveMarker::Request> request,
    std::shared_ptr<swarm_interfaces::srv::ReserveMarker::Response> response)
{
    const int marker_id = request->marker_id;
    const std::string &drone_id = request->drone_id;
    cleanup_expired_markers();

    // Find the marker in the registry
    auto it = std::find_if(marker_registry_.begin(), marker_registry_.end(),
                            [marker_id](const Marker &m)
                            { return static_cast<int>(m.marker_id) == marker_id; });
    // Marker not found
    if (it == marker_registry_.end())
    {
        response->success = false;
        response->message = "Unknown marker id.";
        RCLCPP_WARN(this->get_logger(), "[%s] tried to reserve unknown marker %d.", drone_id.c_str(), marker_id);
        return;
    }

    // Handle successful reservation
    if (it->state == Marker::FREE)
    {
        it->state = Marker::RESERVED;
        it->owner = drone_id;
        it->last_update = std::chrono::steady_clock::now();
        response->success = true;
        response->message = "Marker reserved successfully.";
        RCLCPP_INFO(this->get_logger(), "[%s] reserved marker %d.", drone_id.c_str(), marker_id);
        publish_unavailable_markers_on_update();
        return;
    }

    // Renewal by the same drone
    if (it->owner == drone_id)
    {
        it->last_update = std::chrono::steady_clock::now();
        response->success = true;
        response->message = "Reservation renewed.";
        RCLCPP_INFO(this->get_logger(), "[%s] renewed marker %d.", drone_id.c_str(), marker_id);
        return;
    }

    // Reserved or landed by another drone
    response->success = false;
    response->message = "Marker not available (owned by " + it->owner + ")";
    RCLCPP_WARN(this->get_logger(),
                "[%s] failed to reserve marker %d (owned by %s).",
                drone_id.c_str(), marker_id, it->owner.c_str());
}

void MarkerManager::handle_mark_landed(
    const std::shared_ptr<swarm_interfaces::srv::MarkLanded::Request> request,
    std::shared_ptr<swarm_interfaces::srv::MarkLanded::Response> response)
{
    const int marker_id = request->marker_id;
    const std::string &drone_id = request->drone_id;

    // Find the marker in the registry
    auto it = std::find_if(marker_registry_.begin(), marker_registry_.end(),
                            [marker_id](const Marker &m)
                            { return static_cast<int>(m.marker_id) == marker_id; });
    if (it == marker_registry_.end())
    {
        response->success = false;
        response->message = "Unknown marker id.";
        RCLCPP_WARN(this->get_logger(),
                    "[%s] attempted to mark landed on unknown marker %d.",
                    drone_id.c_str(), marker_id);
        return;
    }

    // Marker must be previously RESERVED to mark as LANDED
    if (it->state == Marker::FREE)
    {
        response->success = false;
        response->message = "Marker was not reserved.";
        RCLCPP_WARN(this->get_logger(),
                    "[%s] attempted to mark landed on FREE marker %d.",
                    drone_id.c_str(), marker_id);
        return;
    }

    // Only the owner drone can confirm landing
    if (it->owner != drone_id)
    {
        response->success = false;
        response->message = "Marker owned by another drone: " + it->owner;
        RCLCPP_WARN(this->get_logger(),
                    "[%s] attempted to mark landed on marker %d owned by %s.",
                    drone_id.c_str(), marker_id, it->owner.c_str());
        return;
    }

    // Owner confirms landing → set state LANDED
    it->state = Marker::LANDED;
    it->last_update = std::chrono::steady_clock::now();
    response->success = true;
    response->message = "Marker landed confirmed.";
    RCLCPP_INFO(this->get_logger(),
                "[%s] marked marker %d as landed.",
                drone_id.c_str(), marker_id);

    publish_unavailable_markers_on_update();
}

void MarkerManager::cleanup_expired_markers()
{
    const auto now = std::chrono::steady_clock::now();
    for (auto &m : marker_registry_)
    {
        if (m.state == Marker::RESERVED && (now - m.last_update > cooldown_duration_))
        {
            RCLCPP_INFO(this->get_logger(),
                        "Marker %u (reserved by %s) expired and is now FREE.",
                        m.marker_id, m.owner.c_str());
            m.state = Marker::FREE;
            m.owner.clear();
            m.last_update = now;
        }
    }
}

void MarkerManager::publish_unavailable_markers()
{
    cleanup_expired_markers();

    std_msgs::msg::Int32MultiArray msg;
    // Publish IDs of markers that are not FREE
    for (const auto &m : marker_registry_)
    {
        if (m.state != Marker::FREE)
        {
            msg.data.push_back(static_cast<int32_t>(m.marker_id));
        }
    }

    unavailable_markers_pub_->publish(msg);
}

void MarkerManager::publish_unavailable_markers_on_update()
{
    publish_unavailable_markers();
}

} // namespace tello_swarm

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tello_swarm::MarkerManager>());
    rclcpp::shutdown();
    return 0;
}
