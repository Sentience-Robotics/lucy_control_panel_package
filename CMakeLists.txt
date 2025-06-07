cmake_minimum_required(VERSION 3.8)
project(lucy_control_panel_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Include directories
include_directories(include)
include_directories(third_party)

# Create executable for the REST API node
add_executable(lucy_api_server
  src/lucy_api_server.cpp
  src/rest_api_node.cpp
  src/joint_controller.cpp
  src/urdf_parser.cpp
  src/handlers/health_handler.cpp
  src/handlers/robot_handler.cpp
  src/handlers/joints_handler.cpp
)

# Specify dependencies
ament_target_dependencies(lucy_api_server
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  urdf
  robot_state_publisher
)

# Add pthread for cpp-httplib
target_link_libraries(lucy_api_server pthread)

# Install executable
install(TARGETS
  lucy_api_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files (examples)
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linting
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()