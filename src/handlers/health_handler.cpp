#include "lucy_control_panel_package/handlers/health_handler.hpp"

#include <chrono>
#include "nlohmann/json.hpp"
#include "httplib.h"

using json = nlohmann::json;

namespace lucy_control_panel_package
{
namespace handlers
{

HealthHandler::HealthHandler(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
: node_(node)
{
}

void HealthHandler::handle_health_check(const httplib::Request& /*req*/, httplib::Response& res)
{
  json response = {
    {"status", "healthy"},
    {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()},
    {"node_name", node_->get_name()},
    {"server_version", "1.0.0"}
  };

  res.set_content(response.dump(), "application/json");
}

}  // namespace handlers
}  // namespace lucy_control_panel_package