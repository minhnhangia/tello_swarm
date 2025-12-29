#include "tello_swarm/swarm_trigger_base.hpp"

#include <chrono>
#include <sstream>
#include <algorithm>

using namespace std::chrono_literals;

namespace tello_swarm
{

SwarmTriggerBase::SwarmTriggerBase(
    const std::string& node_name,
    const std::string& all_service_name,
    const std::string& individual_service_param,
    const std::string& default_service_name)
    : rclcpp::Node(node_name)
{
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>("drone_ids", std::vector<std::string>{"tello1"});
  this->declare_parameter<std::string>(individual_service_param, default_service_name);
  this->declare_parameter<double>("client_timeout_sec", 5.0);

  // Get parameters
  this->get_parameter("drone_ids", drone_ids_);
  this->get_parameter(individual_service_param, individual_service_name_);
  this->get_parameter("client_timeout_sec", client_timeout_sec_);

  // Extract operation name from service name (e.g., "takeoff" from "takeoff_all")
  operation_name_ = all_service_name;
  // Remove "_all" suffix if present
  size_t suffix_pos = operation_name_.find("_all");
  if (suffix_pos != std::string::npos) {
    operation_name_ = operation_name_.substr(0, suffix_pos);
  }

  // Create a reentrant callback group for parallel service call handling
  reentrant_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  // Service server to trigger operation for all drones
  trigger_all_srv_ = this->create_service<std_srvs::srv::Trigger>(
      all_service_name,
      std::bind(&SwarmTriggerBase::handle_trigger_all, this, 
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::QoS(rclcpp::KeepLast(10)),
      reentrant_callback_group_);

  create_service_clients();
  print_startup_summary();
}

void SwarmTriggerBase::create_service_clients()
{
  // Pre-create service clients for each drone
  for (const auto &id : drone_ids_)
  {
    if (id.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Skipping empty drone ID");
      continue;
    }

    if (clients_.find(id) != clients_.end())
    {
      RCLCPP_WARN(this->get_logger(), "Duplicate drone ID detected: %s (ignoring duplicate)", id.c_str());
      continue;
    }

    std::string service = "/" + id + "/" + individual_service_name_;
    clients_[id] = this->create_client<std_srvs::srv::Trigger>(service);
  }
}

void SwarmTriggerBase::print_startup_summary()
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
    
    // Capitalize first letter for display
    std::string display_name = operation_name_;
    if (!display_name.empty()) {
      display_name[0] = std::toupper(display_name[0]);
    }
    
    RCLCPP_INFO(this->get_logger(), "%s Server configured for drones: [%s]", 
                display_name.c_str(), list.str().c_str());
    for (const auto &id : drone_ids_)
    {
      RCLCPP_INFO(this->get_logger(), "Will call service: /%s/%s", 
                  id.c_str(), individual_service_name_.c_str());
    }
  }
}

void SwarmTriggerBase::handle_trigger_all(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (drone_ids_.empty())
  {
    response->success = false;
    response->message = "No drone IDs configured";
    return;
  }

  // Launch async calls for all drones in parallel
  std::vector<std::pair<std::string, std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>>>> futures;
  for (const auto &id : drone_ids_)
  {
    auto client_it = clients_.find(id);
    if (client_it == clients_.end())
    {
      RCLCPP_ERROR(this->get_logger(), "No service client found for drone [%s] - this should not happen!", id.c_str());
      continue;
    }

    auto client = client_it->second;
    auto future = call_single_drone_async(id, client);
    futures.emplace_back(id, std::move(future));
  }

  // Check if we have any futures to wait for
  if (futures.empty())
  {
    response->success = false;
    response->message = "No valid service clients available";
    RCLCPP_ERROR(this->get_logger(), "No futures created - cannot proceed with %s", operation_name_.c_str());
    return;
  }

  // Wait for all results (non-blocking wait per iteration)
  size_t success_count = 0;
  std::ostringstream oss;
  rclcpp::Time start = this->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(client_timeout_sec_);

  bool all_done = false;
  while (rclcpp::ok() && !all_done)
  {
    all_done = true;
    for (auto &pair : futures)
    {
      auto &fut = pair.second;
      if (fut.wait_for(0ms) != std::future_status::ready)
      {
        all_done = false;
      }
    }
    
    // Check timeout
    if ((this->now() - start) > timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Global timeout reached while waiting for all %s responses", 
                   operation_name_.c_str());
      break;
    }
    rclcpp::sleep_for(50ms);
  }

  // Gather results
  for (auto &pair : futures)
  {
    const std::string &id = pair.first;
    auto &fut = pair.second;
    bool ok = false;
    std::string msg;

    if (fut.wait_for(0ms) == std::future_status::ready)
    {
      try
      {
        auto resp = fut.get();
        if (resp)
        {
          ok = resp->success;
          msg = resp->message;
        }
        else
        {
          msg = "Null response";
        }
      }
      catch (const std::exception &e)
      {
        msg = std::string("Exception: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "[%s] Exception while getting future result: %s", id.c_str(), e.what());
      }
    }
    else
    {
      msg = "Timeout";
    }

    if (ok)
    {
      success_count++;
      RCLCPP_INFO(this->get_logger(), "[%s] success: %s", id.c_str(), msg.c_str());
      oss << "[" << id << ":ok] ";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "[%s] failed: %s", id.c_str(), msg.c_str());
      oss << "[" << id << ":fail] ";
    }
  }

  oss << success_count << "/" << drone_ids_.size() << " succeeded";
  response->success = (success_count == drone_ids_.size());
  response->message = oss.str();
}

std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>>
SwarmTriggerBase::call_single_drone_async(
    const std::string &id,
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client)
{
  std::string service = "/" + id + "/" + individual_service_name_;
  rclcpp::Time start = this->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(client_timeout_sec_);

  // Wait for service to be available (but allow multiple simultaneous waits)
  bool service_available = false;
  while (!client->wait_for_service(200ms))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s", service.c_str());
      break;
    }
    if ((this->now() - start) > timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for service %s", service.c_str());
      break;
    }
  }

  // Check if service is now available
  service_available = client->service_is_ready();

  if (!service_available)
  {
    RCLCPP_WARN(this->get_logger(), "Service %s not available, returning invalid future", service.c_str());
    // Return an immediately-ready future with nullptr to indicate failure
    std::promise<std::shared_ptr<std_srvs::srv::Trigger::Response>> promise;
    promise.set_value(nullptr);
    return promise.get_future().share();
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  return future.future.share();
}

} // namespace tello_swarm
