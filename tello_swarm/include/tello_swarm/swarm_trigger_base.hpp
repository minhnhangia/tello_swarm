#ifndef TELLO_SWARM__SWARM_TRIGGER_BASE__HPP_
#define TELLO_SWARM__SWARM_TRIGGER_BASE__HPP_

#include <string>
#include <vector>
#include <map>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace tello_swarm
{

/**
 * @brief Base class for swarm trigger servers (takeoff, landing, etc.)
 * 
 * This class provides common functionality for coordinating Trigger service calls
 * across multiple drones in parallel. Derived classes only need to specify the
 * node name, service names, and parameter names in their constructor.
 */
class SwarmTriggerBase : public rclcpp::Node
{
protected:
    /**
     * @brief Constructor for derived classes
     * 
     * @param node_name Name of the ROS2 node (e.g., "takeoff_server")
     * @param all_service_name Name of the aggregate service (e.g., "takeoff_all")
     * @param individual_service_param Parameter name for individual drone service (e.g., "takeoff_service_name")
     * @param default_service_name Default value for individual service name (e.g., "takeoff")
     */
    SwarmTriggerBase(
        const std::string& node_name,
        const std::string& all_service_name,
        const std::string& individual_service_param,
        const std::string& default_service_name);

    virtual ~SwarmTriggerBase() = default;

private:
    // Parameters
    std::vector<std::string> drone_ids_;
    std::string individual_service_name_;
    std::string operation_name_;  // For logging (e.g., "takeoff", "landing")
    double client_timeout_sec_;

    // Service server to trigger swarm operation
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_all_srv_;

    // Persistent service clients for each drone
    std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;

    // Reentrant callback group for parallel service handling
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;

    void create_service_clients();

    void print_startup_summary();

    void handle_trigger_all(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Async service call helper
    std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>>
    call_single_drone_async(
        const std::string &id,
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client);
};

} // namespace tello_swarm

#endif // TELLO_SWARM__SWARM_TRIGGER_BASE__HPP_
