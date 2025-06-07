#include "lucy_control_panel_package/urdf_parser.hpp"

#include <fstream>
#include <sstream>

namespace lucy_control_panel_package
{

UrdfParser::UrdfParser(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
: node_(node), urdf_model_(std::make_unique<urdf::Model>())
{
}

bool UrdfParser::load_urdf_from_file(const std::string& file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open URDF file: %s", file_path.c_str());
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string urdf_string = buffer.str();

  return load_urdf_from_string(urdf_string);
}

bool UrdfParser::load_urdf_from_string(const std::string& urdf_string)
{
  if (urdf_string.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "URDF string is empty");
    return false;
  }

  if (!urdf_model_->initString(urdf_string)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse URDF string");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully loaded URDF model: %s", urdf_model_->getName().c_str());
  return true;
}

bool UrdfParser::load_urdf_from_parameter(const std::string& parameter_name)
{
  // Try to get the parameter from the node
  if (!node_->has_parameter(parameter_name)) {
    RCLCPP_WARN(node_->get_logger(), "Parameter %s not found", parameter_name.c_str());
    return false;
  }

  std::string urdf_string;
  try {
    urdf_string = node_->get_parameter(parameter_name).as_string();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get parameter %s: %s", parameter_name.c_str(), e.what());
    return false;
  }

  return load_urdf_from_string(urdf_string);
}

std::vector<JointInfo> UrdfParser::extract_joints() const
{
  std::vector<JointInfo> joints;

  if (!is_loaded()) {
    RCLCPP_WARN(node_->get_logger(), "URDF model not loaded");
    return joints;
  }

  // Iterate through all joints in the URDF model
  for (const auto& [joint_name, joint_ptr] : urdf_model_->joints_) {
    if (joint_ptr) {
      JointInfo joint_info = convert_urdf_joint_to_joint_info(*joint_ptr);
      joints.push_back(joint_info);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Extracted %zu joints from URDF", joints.size());
  return joints;
}

JointInfo UrdfParser::extract_joint_info(const std::string& joint_name) const
{
  if (!is_loaded()) {
    throw std::runtime_error("URDF model not loaded");
  }

  auto joint_ptr = urdf_model_->getJoint(joint_name);
  if (!joint_ptr) {
    throw std::runtime_error("Joint not found in URDF: " + joint_name);
  }

  return convert_urdf_joint_to_joint_info(*joint_ptr);
}

std::string UrdfParser::get_robot_name() const
{
  if (!is_loaded()) {
    return "";
  }

  return urdf_model_->getName();
}

std::vector<std::string> UrdfParser::get_joint_names() const
{
  std::vector<std::string> joint_names;

  if (!is_loaded()) {
    return joint_names;
  }

  for (const auto& [joint_name, joint_ptr] : urdf_model_->joints_) {
    if (joint_ptr) {
      joint_names.push_back(joint_name);
    }
  }

  return joint_names;
}

std::vector<std::string> UrdfParser::get_link_names() const
{
  std::vector<std::string> link_names;

  if (!is_loaded()) {
    return link_names;
  }

  for (const auto& [link_name, link_ptr] : urdf_model_->links_) {
    if (link_ptr) {
      link_names.push_back(link_name);
    }
  }

  return link_names;
}

bool UrdfParser::is_loaded() const
{
  return urdf_model_ && !urdf_model_->getName().empty();
}

bool UrdfParser::has_joint(const std::string& joint_name) const
{
  if (!is_loaded()) {
    return false;
  }

  return urdf_model_->getJoint(joint_name) != nullptr;
}

JointInfo UrdfParser::convert_urdf_joint_to_joint_info(const urdf::Joint& urdf_joint) const
{
  JointInfo joint_info;

  joint_info.name = urdf_joint.name;
  joint_info.type = joint_type_to_string(urdf_joint.type);

  // Set parent and child links
  if (urdf_joint.parent_link_name.empty()) {
    joint_info.parent_link = "world";
  } else {
    joint_info.parent_link = urdf_joint.parent_link_name;
  }
  joint_info.child_link = urdf_joint.child_link_name;

  // Set limits based on joint type and available limit information
  if (urdf_joint.limits) {
    joint_info.min_position = urdf_joint.limits->lower;
    joint_info.max_position = urdf_joint.limits->upper;
    joint_info.max_velocity = urdf_joint.limits->velocity;
    joint_info.max_effort = urdf_joint.limits->effort;
  } else {
    // Default values for joints without explicit limits
    if (joint_info.type == "continuous") {
      joint_info.min_position = -M_PI;
      joint_info.max_position = M_PI;
    } else {
      joint_info.min_position = -1.0;
      joint_info.max_position = 1.0;
    }
    joint_info.max_velocity = 1.0;
    joint_info.max_effort = 1.0;
  }

  return joint_info;
}

double UrdfParser::get_joint_limit_value(const urdf::JointLimits& limits, const std::string& property) const
{
  if (property == "lower") {
    return limits.lower;
  } else if (property == "upper") {
    return limits.upper;
  } else if (property == "velocity") {
    return limits.velocity;
  } else if (property == "effort") {
    return limits.effort;
  }

  return 0.0;
}

std::string UrdfParser::joint_type_to_string(int urdf_type) const
{
  switch (urdf_type) {
    case urdf::Joint::REVOLUTE:
      return "revolute";
    case urdf::Joint::CONTINUOUS:
      return "continuous";
    case urdf::Joint::PRISMATIC:
      return "prismatic";
    case urdf::Joint::FIXED:
      return "fixed";
    case urdf::Joint::FLOATING:
      return "floating";
    case urdf::Joint::PLANAR:
      return "planar";
    default:
      return "unknown";
  }
}

}  // namespace lucy_control_panel_package