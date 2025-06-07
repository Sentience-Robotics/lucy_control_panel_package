#include "lucy_control_panel_package/handlers/robot_handler.hpp"

#include "nlohmann/json.hpp"
#include "httplib.h"

using json = nlohmann::json;

namespace lucy_control_panel_package
{
    namespace handlers
    {

        RobotHandler::RobotHandler(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
            std::shared_ptr<UrdfParser> urdf_parser
        ): node_(node), urdf_parser_(urdf_parser)
        {
        }

        void RobotHandler::handle_get_robot_info(const httplib::Request& /*req*/, httplib::Response& res)
        {
            json response = {
                {"robot_name", urdf_parser_->is_loaded() ? urdf_parser_->get_robot_name() : "unknown"},
                {"joint_count", urdf_parser_->is_loaded() ? urdf_parser_->get_joint_names().size() : 0},
                {"urdf_loaded", urdf_parser_->is_loaded()}
            };

            if (urdf_parser_->is_loaded()) {
                response["joint_names"] = urdf_parser_->get_joint_names();
                response["link_names"] = urdf_parser_->get_link_names();
            }

            res.set_content(response.dump(), "application/json");
        }

    }  // namespace handlers
}  // namespace lucy_control_panel_package