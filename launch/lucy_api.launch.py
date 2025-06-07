#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    server_host_arg = DeclareLaunchArgument(
        'server_host',
        default_value='0.0.0.0',
        description='Host address for the REST API server'
    )

    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='8080',
        description='Port for the REST API server'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',
        description='Path to URDF file (optional, will use robot_description parameter if not provided)'
    )

    # API Server Node
    api_server_node = Node(
        package='lucy_control_panel_package',
        executable='lucy_api_server',
        name='lucy_api_server',
        output='screen',
        parameters=[{
            'server_host': LaunchConfiguration('server_host'),
            'server_port': LaunchConfiguration('server_port'),
            'urdf_file_path': LaunchConfiguration('urdf_file'),
        }],
        # Enable lifecycle management
        ros_arguments=['--log-level', 'info']
    )

    return LaunchDescription([
        server_host_arg,
        server_port_arg,
        urdf_file_arg,
        api_server_node,
    ])