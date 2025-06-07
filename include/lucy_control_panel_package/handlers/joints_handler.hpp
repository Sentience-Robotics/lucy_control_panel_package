#ifndef LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__JOINTS_HANDLER_HPP_
#define LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__JOINTS_HANDLER_HPP_

#include <memory>
#include <mutex>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lucy_control_panel_package/joint_controller.hpp"

// Forward declaration to avoid including heavy httplib.h in header
namespace httplib {
  class Request;
  class Response;
}

namespace lucy_control_panel_package
{
namespace handlers
{

class JointsHandler
{
public:
  explicit JointsHandler(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    std::shared_ptr<JointController> joint_controller,
    std::shared_ptr<sensor_msgs::msg::JointState> current_joint_states,
    std::shared_ptr<std::mutex> joint_states_mutex
  );
  ~JointsHandler() = default;

  void handle_get_joints(const httplib::Request& req, httplib::Response& res);
  void handle_get_joint_state(const httplib::Request& req, httplib::Response& res);
  void handle_set_joint_position(const httplib::Request& req, httplib::Response& res);

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<JointController> joint_controller_;
  std::shared_ptr<sensor_msgs::msg::JointState> current_joint_states_;
  std::shared_ptr<std::mutex> joint_states_mutex_;
};

}  // namespace handlers
}  // namespace lucy_control_panel_package

#endif  // LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__JOINTS_HANDLER_HPP_
