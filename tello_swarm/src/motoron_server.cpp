#include "tello_swarm/motoron_server.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace tello_swarm
{

MotoronServer::MotoronServer()
    : SwarmTriggerBase(
        "motoron_server",        // node_name
        "motoron_all",           // all_service_name
        "motoron_service_name",  // individual_service_param
        "motoron"                // default_service_name
      )
{
  // All initialization handled by base class
}

} // namespace tello_swarm

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create the motoron server node
  auto node = std::make_shared<tello_swarm::MotoronServer>();

  // Use MultiThreadedExecutor to handle multiple concurrent service calls
  // This ensures we can handle multiple /motoron_all calls simultaneously
  // and process responses from all drones in parallel
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      /* num_threads = */ 0,  // 0 = hardware_concurrency (optimal for system)
      /* yield_before_execute = */ false,
      /* timeout = */ std::chrono::milliseconds(100));

  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Starting MultiThreadedExecutor for MotoronServer");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
