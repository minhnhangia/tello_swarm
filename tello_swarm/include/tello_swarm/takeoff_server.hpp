#ifndef TELLO_SWARM__TAKEOFF_SERVER__HPP_
#define TELLO_SWARM__TAKEOFF_SERVER__HPP_

#include <string>
#include <vector>

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

    void print_startup_summary();

    // Handlers
    void handle_takeoff_all(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Helper: call a single drone's takeoff service
    bool call_single_drone_takeoff(const std::string &ns);
};

} // namespace tello_swarm

#endif // TELLO_SWARM__TAKEOFF_SERVER__HPP_