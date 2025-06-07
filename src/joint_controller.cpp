#include "lucy_control_panel_package/joint_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace lucy_control_panel_package
{

JointController::JointController(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
: node_(node)
{
  // Initialize empty joint states
  current_states_.name.clear();
  current_states_.position.clear();
  current_states_.velocity.clear();
  current_states_.effort.clear();
}

void JointController::add_joint(const JointInfo& joint_info)
{
  std::lock_guard<std::mutex> lock(states_mutex_);
  joints_[joint_info.name] = joint_info;
  ensure_joint_in_states(joint_info.name);

  RCLCPP_INFO(node_->get_logger(), "Added joint: %s (type: %s)",
              joint_info.name.c_str(), joint_info.type.c_str());
}

void JointController::remove_joint(const std::string& joint_name)
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  // Remove from joints map
  joints_.erase(joint_name);

  // Remove from current states
  auto it = std::find(current_states_.name.begin(), current_states_.name.end(), joint_name);
  if (it != current_states_.name.end()) {
    size_t index = std::distance(current_states_.name.begin(), it);
    current_states_.name.erase(current_states_.name.begin() + index);

    if (index < current_states_.position.size()) {
      current_states_.position.erase(current_states_.position.begin() + index);
    }
    if (index < current_states_.velocity.size()) {
      current_states_.velocity.erase(current_states_.velocity.begin() + index);
    }
    if (index < current_states_.effort.size()) {
      current_states_.effort.erase(current_states_.effort.begin() + index);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Removed joint: %s", joint_name.c_str());
}

std::vector<JointInfo> JointController::get_all_joints() const
{
  std::lock_guard<std::mutex> lock(states_mutex_);
  std::vector<JointInfo> result;

  for (const auto& [name, joint_info] : joints_) {
    result.push_back(joint_info);
  }

  return result;
}

JointInfo JointController::get_joint_info(const std::string& joint_name) const
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  auto it = joints_.find(joint_name);
  if (it == joints_.end()) {
    throw std::runtime_error("Joint not found: " + joint_name);
  }

  return it->second;
}

bool JointController::joint_exists(const std::string& joint_name) const
{
  std::lock_guard<std::mutex> lock(states_mutex_);
  return joints_.find(joint_name) != joints_.end();
}

bool JointController::set_joint_position(const std::string& joint_name, double position)
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  if (!joint_exists(joint_name)) {
    RCLCPP_WARN(node_->get_logger(), "Attempted to set position for unknown joint: %s", joint_name.c_str());
    return false;
  }

  if (!is_position_valid(joint_name, position)) {
    RCLCPP_WARN(node_->get_logger(), "Invalid position %f for joint %s", position, joint_name.c_str());
    return false;
  }

  size_t index = find_joint_index(joint_name);
  if (index < current_states_.position.size()) {
    current_states_.position[index] = position;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Set joint %s position to %f", joint_name.c_str(), position);
  return true;
}

bool JointController::set_joint_velocity(const std::string& joint_name, double velocity)
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  if (!joint_exists(joint_name)) {
    return false;
  }

  if (!is_velocity_valid(joint_name, velocity)) {
    return false;
  }

  size_t index = find_joint_index(joint_name);
  if (index < current_states_.velocity.size()) {
    current_states_.velocity[index] = velocity;
  }

  return true;
}

bool JointController::set_joint_effort(const std::string& joint_name, double effort)
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  if (!joint_exists(joint_name)) {
    return false;
  }

  if (!is_effort_valid(joint_name, effort)) {
    return false;
  }

  size_t index = find_joint_index(joint_name);
  if (index < current_states_.effort.size()) {
    current_states_.effort[index] = effort;
  }

  return true;
}

bool JointController::set_multiple_joint_positions(const std::map<std::string, double>& positions)
{
  bool all_success = true;

  for (const auto& [joint_name, position] : positions) {
    if (!set_joint_position(joint_name, position)) {
      all_success = false;
    }
  }

  return all_success;
}

void JointController::update_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  // Update header
  current_states_.header = msg->header;

  // Update joint states for known joints
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& joint_name = msg->name[i];

    if (joint_exists(joint_name)) {
      size_t index = find_joint_index(joint_name);

      if (i < msg->position.size() && index < current_states_.position.size()) {
        current_states_.position[index] = msg->position[i];
      }
      if (i < msg->velocity.size() && index < current_states_.velocity.size()) {
        current_states_.velocity[index] = msg->velocity[i];
      }
      if (i < msg->effort.size() && index < current_states_.effort.size()) {
        current_states_.effort[index] = msg->effort[i];
      }
    }
  }
}

sensor_msgs::msg::JointState JointController::get_current_joint_states() const
{
  std::lock_guard<std::mutex> lock(states_mutex_);
  return current_states_;
}

double JointController::get_joint_position(const std::string& joint_name) const
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  size_t index = find_joint_index(joint_name);
  if (index < current_states_.position.size()) {
    return current_states_.position[index];
  }

  return 0.0;
}

double JointController::get_joint_velocity(const std::string& joint_name) const
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  size_t index = find_joint_index(joint_name);
  if (index < current_states_.velocity.size()) {
    return current_states_.velocity[index];
  }

  return 0.0;
}

double JointController::get_joint_effort(const std::string& joint_name) const
{
  std::lock_guard<std::mutex> lock(states_mutex_);

  size_t index = find_joint_index(joint_name);
  if (index < current_states_.effort.size()) {
    return current_states_.effort[index];
  }

  return 0.0;
}

bool JointController::is_position_valid(const std::string& joint_name, double position) const
{
  auto it = joints_.find(joint_name);
  if (it == joints_.end()) {
    return false;
  }

  const JointInfo& joint = it->second;

  // For continuous joints, any position is valid
  if (joint.type == "continuous") {
    return true;
  }

  // Check limits for other joint types
  return position >= joint.min_position && position <= joint.max_position;
}

bool JointController::is_velocity_valid(const std::string& joint_name, double velocity) const
{
  auto it = joints_.find(joint_name);
  if (it == joints_.end()) {
    return false;
  }

  const JointInfo& joint = it->second;
  return std::abs(velocity) <= joint.max_velocity;
}

bool JointController::is_effort_valid(const std::string& joint_name, double effort) const
{
  auto it = joints_.find(joint_name);
  if (it == joints_.end()) {
    return false;
  }

  const JointInfo& joint = it->second;
  return std::abs(effort) <= joint.max_effort;
}

size_t JointController::find_joint_index(const std::string& joint_name) const
{
  auto it = std::find(current_states_.name.begin(), current_states_.name.end(), joint_name);
  if (it != current_states_.name.end()) {
    return std::distance(current_states_.name.begin(), it);
  }

  return current_states_.name.size(); // Return invalid index
}

void JointController::ensure_joint_in_states(const std::string& joint_name)
{
  // Check if joint is already in states
  if (std::find(current_states_.name.begin(), current_states_.name.end(), joint_name) != current_states_.name.end()) {
    return;
  }

  // Add joint to states
  current_states_.name.push_back(joint_name);
  current_states_.position.push_back(0.0);
  current_states_.velocity.push_back(0.0);
  current_states_.effort.push_back(0.0);
}

}  // namespace lucy_control_panel_package