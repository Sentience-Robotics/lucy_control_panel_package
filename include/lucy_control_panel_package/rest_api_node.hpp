#ifndef LUCY_CONTROL_PANEL_PACKAGE__REST_API_NODE_HPP_
#define LUCY_CONTROL_PANEL_PACKAGE__REST_API_NODE_HPP_

#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include "lucy_control_panel_package/joint_controller.hpp"
#include "lucy_control_panel_package/urdf_parser.hpp"
#include "lucy_control_panel_package/handlers/health_handler.hpp"
#include "lucy_control_panel_package/handlers/robot_handler.hpp"
#include "lucy_control_panel_package/handlers/joints_handler.hpp"

// Forward declaration to avoid including heavy httplib.h in header
namespace httplib {
  class Server;
  class Request;
  class Response;
}

namespace lucy_control_panel_package
{

class RestApiNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RestApiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RestApiNode();

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  // HTTP Server
  std::unique_ptr<httplib::Server> server_;
  std::thread server_thread_;
  std::atomic<bool> server_running_;

  // Configuration
  std::string server_host_;
  int server_port_;
  std::string urdf_file_path_;

  // ROS 2 Components
  std::unique_ptr<JointController> joint_controller_;
  std::unique_ptr<UrdfParser> urdf_parser_;

  // Handlers
  std::unique_ptr<handlers::HealthHandler> health_handler_;
  std::unique_ptr<handlers::RobotHandler> robot_handler_;
  std::unique_ptr<handlers::JointsHandler> joints_handler_;

  // ROS 2 Publishers and Subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Current joint states
  sensor_msgs::msg::JointState::SharedPtr current_joint_states_;
  std::shared_ptr<std::mutex> joint_states_mutex_;

  // Methods
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void setup_rest_endpoints();
  void start_server();
  void stop_server();

  // Handler initialization
  void initialize_handlers();
};

}  // namespace lucy_control_panel_package

#endif  // LUCY_CONTROL_PANEL_PACKAGE__REST_API_NODE_HPP_