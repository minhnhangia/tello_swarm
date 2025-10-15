#ifndef TELLO_SWARM__TAKEOFF_SERVER__HPP_
#define TELLO_SWARM__TAKEOFF_SERVER__HPP_

#include <string>
#include <vector>
#include <map>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace tello_swarm
{

class TakeoffServer : public rclcpp::Node
{
public:
    TakeoffServer();

private:
    // Parameters
    std::vector<std::string> drone_ids_;
    std::string takeoff_service_name_;
    double client_timeout_sec_;

    // Service server to trigger swarm takeoff
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_all_srv_;

    // Persistent service clients for each drone
    std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;

    // Reentrant callback group for parallel service handling
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;

    void create_service_clients();

    void print_startup_summary();

    void handle_takeoff_all(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Async service call helper
    std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>>
    call_single_drone_takeoff_async(
        const std::string &id,
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client);
};

} // namespace tello_swarm

#endif // TELLO_SWARM__TAKEOFF_SERVER__HPP_