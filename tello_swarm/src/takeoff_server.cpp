#include <chrono>
#include <sstream>

#include "tello_swarm/takeoff_server.hpp"

using namespace std::chrono_literals;

namespace tello_swarm
{

TakeoffServer::TakeoffServer() : rclcpp::Node("takeoff_server")
{
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>("drone_ids", std::vector<std::string>{"tello1"});
  // Backward-compat: also accept 'drone_namespaces' if provided in YAML
  this->declare_parameter<std::vector<std::string>>("drone_namespaces", std::vector<std::string>{});
  this->declare_parameter<std::string>("takeoff_service_name", "takeoff");
  this->declare_parameter<double>("client_timeout_sec", 5.0);

  // Get parameters
  this->get_parameter("drone_ids", drone_ids_);
  std::vector<std::string> drone_namespaces_param;
  this->get_parameter("drone_namespaces", drone_namespaces_param);
  if (!drone_namespaces_param.empty())
  {
    // Prefer explicitly set 'drone_namespaces' over default 'drone_ids'
    drone_ids_ = drone_namespaces_param;
  }
  this->get_parameter("takeoff_service_name", takeoff_service_name_);
  this->get_parameter("client_timeout_sec", client_timeout_sec_);

  // Create the service server to trigger takeoff for all drones
  takeoff_all_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "takeoff_all",
      std::bind(&TakeoffServer::handle_takeoff_all, this, std::placeholders::_1, std::placeholders::_2));

  // Startup summary logs
  print_startup_summary();

}

void TakeoffServer::print_startup_summary()
{
  if (drone_ids_.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No drones configured. Set 'drone_ids' parameter.");
  }
  else
  {
    std::ostringstream list;
    for (size_t i = 0; i < drone_ids_.size(); ++i)
    {
      list << drone_ids_[i];
      if (i + 1 < drone_ids_.size())
        list << ", ";
    }
    RCLCPP_INFO(this->get_logger(), "Takeoff Server configured for drones: [%s]", list.str().c_str());
    RCLCPP_INFO(this->get_logger(), "Using takeoff service name: '%s' (timeout: %.1fs)",
                takeoff_service_name_.c_str(), client_timeout_sec_);
    // Log full service paths for clarity
    for (const auto &id : drone_ids_)
    {
      RCLCPP_INFO(this->get_logger(), "Will call service: /%s/%s", id.c_str(), takeoff_service_name_.c_str());
    }
  }
}

void TakeoffServer::handle_takeoff_all(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (drone_ids_.empty())
  {
    response->success = false;
    response->message = "No drone IDs configured";
    return;
  }

  size_t success_count = 0;
  std::ostringstream oss;
  for (const auto &id : drone_ids_)
  {
    bool ok = call_single_drone_takeoff(id);
    if (ok)
    {
      success_count++;
      oss << "[" << id << ":ok] ";
    }
    else
    {
      oss << "[" << id << ":fail] ";
    }
  }

  response->success = success_count == drone_ids_.size();
  oss << success_count << "/" << drone_ids_.size() << " succeeded";
  response->message = oss.str();
}

bool TakeoffServer::call_single_drone_takeoff(const std::string &id)
{
  // Full service name with namespace e.g. /tello1/takeoff
  std::string service = "/" + id + "/" + takeoff_service_name_;
  auto client = this->create_client<std_srvs::srv::Trigger>(service);

  rclcpp::Time start = this->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(client_timeout_sec_);

  // Wait for service with timeout
  while (!client->wait_for_service(500ms))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s", service.c_str());
      return false;
    }
    if ((this->now() - start) > timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for service %s", service.c_str());
      return false;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for %s", service.c_str());
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  // Wait for response with timeout using wait_for instead of spin_until_future_complete
  // to avoid nested spinning issues when called from within a service callback
  rclcpp::Time call_start = this->now();
  while (rclcpp::ok())
  {
    auto wait_status = future.wait_for(100ms);
    if (wait_status == std::future_status::ready)
    {
      break;
    }
    if ((this->now() - call_start) > timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call to %s timed out after %.1fs", 
                   service.c_str(), timeout.seconds());
      return false;
    }
  }

  if (!rclcpp::ok())
  {
    RCLCPP_ERROR(this->get_logger(), "Service call to %s interrupted", service.c_str());
    return false;
  }

  auto resp = future.get();
  if (!resp)
  {
    RCLCPP_ERROR(this->get_logger(), "Null response from %s", service.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Takeoff request to %s -> success=%s msg=\"%s\"",
              service.c_str(), resp->success ? "true" : "false", resp->message.c_str());
  return resp->success;
}

} // namespace tello_swarm

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tello_swarm::TakeoffServer>());
  rclcpp::shutdown();
  return 0;
}