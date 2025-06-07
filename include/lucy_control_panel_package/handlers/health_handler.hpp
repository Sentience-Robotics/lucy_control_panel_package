#ifndef LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__HEALTH_HANDLER_HPP_
#define LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__HEALTH_HANDLER_HPP_

#include <memory>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Forward declaration to avoid including heavy httplib.h in header
namespace httplib {
  class Request;
  class Response;
}

namespace lucy_control_panel_package
{
namespace handlers
{

class HealthHandler
{
public:
  explicit HealthHandler(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
  ~HealthHandler() = default;

  void handle_health_check(const httplib::Request& req, httplib::Response& res);

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

}  // namespace handlers
}  // namespace lucy_control_panel_package

#endif  // LUCY_CONTROL_PANEL_PACKAGE__HANDLERS__HEALTH_HANDLER_HPP_
