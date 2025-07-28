import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'rakuda_description'
    file_subpath = 'urdf/test.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )


    rviz_config_file = (
        get_package_share_directory('rakuda_description') + '/launch/display.rviz'
    )
    
    rviz = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )
        
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        rviz,
        joint_state_publisher
        
    ])
    
