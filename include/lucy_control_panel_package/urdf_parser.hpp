#ifndef LUCY_CONTROL_PANEL_PACKAGE__URDF_PARSER_HPP_
#define LUCY_CONTROL_PANEL_PACKAGE__URDF_PARSER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "urdf/model.h"
#include "lucy_control_panel_package/joint_controller.hpp"

namespace lucy_control_panel_package
{

class UrdfParser
{
public:
  explicit UrdfParser(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
  ~UrdfParser() = default;

  // URDF loading
  bool load_urdf_from_file(const std::string& file_path);
  bool load_urdf_from_string(const std::string& urdf_string);
  bool load_urdf_from_parameter(const std::string& parameter_name = "robot_description");

  // Joint extraction
  std::vector<JointInfo> extract_joints() const;
  JointInfo extract_joint_info(const std::string& joint_name) const;

  // Robot information
  std::string get_robot_name() const;
  std::vector<std::string> get_joint_names() const;
  std::vector<std::string> get_link_names() const;

  // Validation
  bool is_loaded() const;
  bool has_joint(const std::string& joint_name) const;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<urdf::Model> urdf_model_;

  // Helper methods
  JointInfo convert_urdf_joint_to_joint_info(const urdf::Joint& urdf_joint) const;
  double get_joint_limit_value(const urdf::JointLimits& limits, const std::string& property) const;
  std::string joint_type_to_string(int urdf_type) const;
};

}  // namespace lucy_control_panel_package

#endif  // LUCY_CONTROL_PANEL_PACKAGE__URDF_PARSER_HPP_