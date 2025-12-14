from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('rakuda_description')

    # Xacro principale (che include rakuda.ros2_control.xacro)
    xacro_file = os.path.join(share_dir, 'urdf', 'rakuda.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Controllers yaml
    controllers_file = os.path.join(share_dir, 'config', 'rakuda_controllers.yaml')

    # RViz
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='True'
    )
    use_rviz = LaunchConfiguration('rviz')

    # Publishes TF using robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    # ros2_control controller manager + hardware
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_urdf},
            controllers_file
        ],
        output='screen'
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    torso_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'torso_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'head_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        torso_controller_spawner,
        head_controller_spawner,
        rviz_node
    ])
