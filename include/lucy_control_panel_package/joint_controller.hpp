#ifndef LUCY_CONTROL_PANEL_PACKAGE__JOINT_CONTROLLER_HPP_
#define LUCY_CONTROL_PANEL_PACKAGE__JOINT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace lucy_control_panel_package
{

struct JointInfo
{
  std::string name;
  std::string type;  // revolute, prismatic, continuous, etc.
  double min_position;
  double max_position;
  double max_velocity;
  double max_effort;
  std::string parent_link;
  std::string child_link;
};

class JointController
{
public:
  explicit JointController(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
  ~JointController() = default;

  // Joint management
  void add_joint(const JointInfo& joint_info);
  void remove_joint(const std::string& joint_name);
  std::vector<JointInfo> get_all_joints() const;
  JointInfo get_joint_info(const std::string& joint_name) const;
  bool joint_exists(const std::string& joint_name) const;

  // Joint control
  bool set_joint_position(const std::string& joint_name, double position);
  bool set_joint_velocity(const std::string& joint_name, double velocity);
  bool set_joint_effort(const std::string& joint_name, double effort);

  // Batch operations
  bool set_multiple_joint_positions(const std::map<std::string, double>& positions);

  // Joint state management
  void update_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg);
  sensor_msgs::msg::JointState get_current_joint_states() const;
  double get_joint_position(const std::string& joint_name) const;
  double get_joint_velocity(const std::string& joint_name) const;
  double get_joint_effort(const std::string& joint_name) const;

  // Validation
  bool is_position_valid(const std::string& joint_name, double position) const;
  bool is_velocity_valid(const std::string& joint_name, double velocity) const;
  bool is_effort_valid(const std::string& joint_name, double effort) const;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  // Joint information storage
  std::map<std::string, JointInfo> joints_;

  // Current joint states
  sensor_msgs::msg::JointState current_states_;
  mutable std::mutex states_mutex_;

  // Helper methods
  size_t find_joint_index(const std::string& joint_name) const;
  void ensure_joint_in_states(const std::string& joint_name);
};

}  // namespace lucy_control_panel_package

#endif  // LUCY_CONTROL_PANEL_PACKAGE__JOINT_CONTROLLER_HPP_