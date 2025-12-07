import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_name = 'rakuda_description'

    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'rakuda_waist_test.xacro'
    )

    # Process xacro
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # YAML controllers
    controller_config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'waist_controllers.yaml'
    )

    robot_description = {'robot_description': robot_description_raw}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # controller_manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # Spawner per joint_state_broadcaster
    jspawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawner per waist_position_controller
    waist_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['waist_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # joint_state_publisher_gui per muovere a mano il joint (facoltativo)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        controller_manager_node,
        jspawner,
        waist_spawner,
        joint_state_publisher_gui,
    ])
