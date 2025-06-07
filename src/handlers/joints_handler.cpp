#include "lucy_control_panel_package/handlers/joints_handler.hpp"

#include <map>
#include <string>
#include "nlohmann/json.hpp"
#include "httplib.h"

using json = nlohmann::json;

namespace lucy_control_panel_package
{
namespace handlers
{

JointsHandler::JointsHandler(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  std::shared_ptr<JointController> joint_controller,
  std::shared_ptr<sensor_msgs::msg::JointState> current_joint_states,
  std::shared_ptr<std::mutex> joint_states_mutex
)
: node_(node),
  joint_controller_(joint_controller),
  current_joint_states_(current_joint_states),
  joint_states_mutex_(joint_states_mutex)
{
}

void JointsHandler::handle_get_joints(const httplib::Request& /*req*/, httplib::Response& res)
{
  auto joints = joint_controller_->get_all_joints();
  json response = json::array();

  for (const auto& joint : joints) {
    json joint_json = {
      {"name", joint.name},
      {"type", joint.type},
      {"min_position", joint.min_position},
      {"max_position", joint.max_position},
      {"max_velocity", joint.max_velocity},
      {"max_effort", joint.max_effort},
      {"parent_link", joint.parent_link},
      {"child_link", joint.child_link}
    };
    response.push_back(joint_json);
  }

  res.set_content(response.dump(), "application/json");
}

void JointsHandler::handle_get_joint_state(const httplib::Request& /*req*/, httplib::Response& res)
{
  std::lock_guard<std::mutex> lock(*joint_states_mutex_);

  if (!current_joint_states_) {
    json error = {{"error", "No joint states available"}};
    res.status = 404;
    res.set_content(error.dump(), "application/json");
    return;
  }

  json response = {
    {"header", {
      {"stamp", {
        {"sec", current_joint_states_->header.stamp.sec},
        {"nanosec", current_joint_states_->header.stamp.nanosec}
      }},
      {"frame_id", current_joint_states_->header.frame_id}
    }},
    {"name", current_joint_states_->name},
    {"position", current_joint_states_->position},
    {"velocity", current_joint_states_->velocity},
    {"effort", current_joint_states_->effort}
  };

  res.set_content(response.dump(), "application/json");
}

void JointsHandler::handle_set_joint_position(const httplib::Request& req, httplib::Response& res)
{
  try {
    auto request_json = json::parse(req.body);

    if (request_json.contains("joint_name") && request_json.contains("position")) {
      // Single joint position
      std::string joint_name = request_json["joint_name"];
      double position = request_json["position"];

      if (joint_controller_->set_joint_position(joint_name, position)) {
        json response = {{"success", true}, {"message", "Joint position set successfully"}};
        res.set_content(response.dump(), "application/json");
      } else {
        json error = {{"error", "Failed to set joint position"}};
        res.status = 400;
        res.set_content(error.dump(), "application/json");
      }
    } else if (request_json.contains("positions")) {
      // Multiple joint positions
      std::map<std::string, double> positions;
      for (const auto& [joint_name, position] : request_json["positions"].items()) {
        positions[joint_name] = position;
      }

      if (joint_controller_->set_multiple_joint_positions(positions)) {
        json response = {{"success", true}, {"message", "Joint positions set successfully"}};
        res.set_content(response.dump(), "application/json");
      } else {
        json error = {{"error", "Failed to set joint positions"}};
        res.status = 400;
        res.set_content(error.dump(), "application/json");
      }
    } else {
      json error = {{"error", "Invalid request format"}};
      res.status = 400;
      res.set_content(error.dump(), "application/json");
    }
  } catch (const json::exception& e) {
    json error = {{"error", std::string("JSON parse error: ") + e.what()}};
    res.status = 400;
    res.set_content(error.dump(), "application/json");
  }
}

}  // namespace handlers
}  // namespace lucy_control_panel_package