#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lucy_control_panel_package/rest_api_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create the REST API node
  auto node = std::make_shared<lucy_control_panel_package::RestApiNode>();

  // Use MultiThreadedExecutor for handling HTTP requests concurrently
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(node->get_logger(), "Starting Lucy Control Panel API Server...");

  try {
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in executor: %s", e.what());
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Lucy Control Panel API Server shutting down.");
  rclcpp::shutdown();
  return 0;
}