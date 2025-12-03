# rakuda_description

`rakuda_description` provides the full robot model and visualization assets for the Rakuda humanoid research platform. It contains the URDF/Xacro definitions, kinematic tree, inertial parameters and meshes needed to load the robot in ROS 2 tools such as `robot_state_publisher`, RViz2 and Gazebo.

The package is designed as the single source of truth for the mechanical structure of Rakuda, supporting dual 7-DOF arms, torso, head and sensor mounts, and is meant to be reused by control, perception and simulation packages across the project.

Requires package OrbbecSDK_ROS2

# rakuda_description

Clone and build

colcon build --packages-select rakuda_description
ros2 launch rakuda_description display.launch.py

