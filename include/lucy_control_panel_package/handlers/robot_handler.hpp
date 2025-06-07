#ifndef LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__ROBOT_HANDLER_HPP_
#define LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__ROBOT_HANDLER_HPP_

#include <memory>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lucy_control_panel_package/urdf_parser.hpp"

// Forward declaration to avoid including heavy httplib.h in header
namespace httplib {
  class Request;
  class Response;
}

namespace lucy_control_panel_package
{
namespace handlers
{

class RobotHandler
{
public:
  explicit RobotHandler(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    std::shared_ptr<UrdfParser> urdf_parser
  );
  ~RobotHandler() = default;

  void handle_get_robot_info(const httplib::Request& req, httplib::Response& res);

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<UrdfParser> urdf_parser_;
};

}  // namespace handlers
}  // namespace lucy_control_panel_package

#endif  // LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__ROBOT_HANDLER_HPP_
