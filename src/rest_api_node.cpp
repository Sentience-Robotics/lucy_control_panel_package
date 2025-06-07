#include "lucy_control_panel_package/rest_api_node.hpp"

#include <functional>
#include <thread>
#include <chrono>

#include "httplib.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace lucy_control_panel_package
{

RestApiNode::RestApiNode(const rclcpp::NodeOptions & options)
: LifecycleNode("lucy_api_server", options),
  server_running_(false)
{
  // Declare parameters
  this->declare_parameter("server_host", "0.0.0.0");
  this->declare_parameter("server_port", 8080);
  this->declare_parameter("urdf_file_path", "");

  RCLCPP_INFO(this->get_logger(), "Lucy Control Panel API Server node created");
}

RestApiNode::~RestApiNode()
{
  stop_server();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RestApiNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Lucy API Server...");

  // Get parameters
  server_host_ = this->get_parameter("server_host").as_string();
  server_port_ = this->get_parameter("server_port").as_int();
  urdf_file_path_ = this->get_parameter("urdf_file_path").as_string();

  // Initialize mutex
  joint_states_mutex_ = std::make_shared<std::mutex>();

  // Initialize components
  joint_controller_ = std::make_unique<JointController>(this->shared_from_this());
  urdf_parser_ = std::make_unique<UrdfParser>(this->shared_from_this());

  // Initialize handlers
  initialize_handlers();

  // Load URDF if path provided
  if (!urdf_file_path_.empty()) {
    if (urdf_parser_->load_urdf_from_file(urdf_file_path_)) {
      auto joints = urdf_parser_->extract_joints();
      for (const auto& joint : joints) {
        joint_controller_->add_joint(joint);
      }
      RCLCPP_INFO(this->get_logger(), "Loaded %zu joints from URDF: %s",
                  joints.size(), urdf_file_path_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to load URDF from: %s", urdf_file_path_.c_str());
    }
  } else {
    // Try to load from robot_description parameter
    if (urdf_parser_->load_urdf_from_parameter()) {
      auto joints = urdf_parser_->extract_joints();
      for (const auto& joint : joints) {
        joint_controller_->add_joint(joint);
      }
      RCLCPP_INFO(this->get_logger(), "Loaded %zu joints from robot_description parameter",
                  joints.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "No URDF loaded - server will start with empty joint list");
    }
  }

  // Setup ROS 2 publishers and subscribers
  joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_commands", 10);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&RestApiNode::joint_state_callback, this, std::placeholders::_1));

  // Initialize HTTP server
  server_ = std::make_unique<httplib::Server>();
  setup_rest_endpoints();

  RCLCPP_INFO(this->get_logger(), "Configuration complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RestApiNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating Lucy API Server...");

  // Start HTTP server
  start_server();

  RCLCPP_INFO(this->get_logger(), "Lucy API Server activated and listening on %s:%d",
              server_host_.c_str(), server_port_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RestApiNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Lucy API Server...");

  stop_server();

  RCLCPP_INFO(this->get_logger(), "Lucy API Server deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RestApiNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Lucy API Server...");

  // Reset handlers
  health_handler_.reset();
  robot_handler_.reset();
  joints_handler_.reset();

  // Reset components
  joint_controller_.reset();
  urdf_parser_.reset();
  server_.reset();

  // Reset ROS 2 interfaces
  joint_command_pub_.reset();
  joint_state_sub_.reset();

  RCLCPP_INFO(this->get_logger(), "Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void RestApiNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(*joint_states_mutex_);
  current_joint_states_ = msg;
  joint_controller_->update_joint_states(msg);
}

void RestApiNode::setup_rest_endpoints()
{
  // Enable CORS
  server_->set_pre_routing_handler([](const httplib::Request& /*req*/, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization");
    return httplib::Server::HandlerResponse::Unhandled;
  });

  // Handle OPTIONS requests for CORS
  server_->Options(".*", [](const httplib::Request& /*req*/, httplib::Response& res) {
    return;
  });

  // Health check endpoint
  server_->Get("/health", [this](const httplib::Request& req, httplib::Response& res) {
    health_handler_->handle_health_check(req, res);
  });

  // Robot information endpoints
  server_->Get("/api/robot/info", [this](const httplib::Request& req, httplib::Response& res) {
    robot_handler_->handle_get_robot_info(req, res);
  });

  // Joint endpoints
  server_->Get("/api/joints", [this](const httplib::Request& req, httplib::Response& res) {
    joints_handler_->handle_get_joints(req, res);
  });

  server_->Get("/api/joints/state", [this](const httplib::Request& req, httplib::Response& res) {
    joints_handler_->handle_get_joint_state(req, res);
  });

  server_->Post("/api/joints/position", [this](const httplib::Request& req, httplib::Response& res) {
    joints_handler_->handle_set_joint_position(req, res);
  });
}

void RestApiNode::start_server()
{
  if (server_running_.load()) {
    return;
  }

  server_running_.store(true);
  server_thread_ = std::thread([this]() {
    RCLCPP_INFO(this->get_logger(), "HTTP server starting on %s:%d",
                server_host_.c_str(), server_port_);

    if (!server_->listen(server_host_, server_port_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start HTTP server on %s:%d",
                   server_host_.c_str(), server_port_);
      server_running_.store(false);
    }
  });
}

void RestApiNode::stop_server()
{
  if (!server_running_.load()) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Stopping HTTP server...");
  server_running_.store(false);

  if (server_) {
    server_->stop();
  }

  if (server_thread_.joinable()) {
    server_thread_.join();
  }

  RCLCPP_INFO(this->get_logger(), "HTTP server stopped");
}

void RestApiNode::initialize_handlers()
{
  health_handler_ = std::make_unique<handlers::HealthHandler>(this->shared_from_this());
  robot_handler_ = std::make_unique<handlers::RobotHandler>(
    this->shared_from_this(),
    std::shared_ptr<UrdfParser>(urdf_parser_.get(), [](UrdfParser*){})
  );
  joints_handler_ = std::make_unique<handlers::JointsHandler>(
    this->shared_from_this(),
    std::shared_ptr<JointController>(joint_controller_.get(), [](JointController*){}),
    current_joint_states_,
    joint_states_mutex_
  );
}



}  // namespace lucy_control_panel_package