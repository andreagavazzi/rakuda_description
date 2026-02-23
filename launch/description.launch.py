from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_share = get_package_share_directory('rakuda_description')

    # Args
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
        description='If true, starts rviz with a pre-configured display.')
    demo_arg = DeclareLaunchArgument('demo', default_value='false',
        description='If true, starts joint_state_publisher_gui for manual joint control (demo mode).')

    use_rviz = LaunchConfiguration('rviz')
    demo = LaunchConfiguration('demo')

    # Robot description (xacro -> urdf xml)
    xacro_file = os.path.join(description_share, 'urdf', 'rakuda.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # RViz config
    rviz_config_file = os.path.join(description_share, 'rviz', 'display.rviz')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    # Only in demo mode: publish /joint_states via GUI sliders
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(demo),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
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
        demo_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

