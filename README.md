# rakuda_description

This package provides a URDF model of Rakuda | Echo in xacro format

<img align="center" src="https://github.com/andreagavazzi/rakuda_description/blob/main/assets/rakuda_rviz.png"/>


Here's a graphical representation of the description (click to see the PDF version):

### Coordinate frames
**base_link** - Rigidly attached to the robot base. Provides an obvious point of reference for other links in the model. Located on the upper plane of the robot, in the central mounting hole, with the X axis facing forward like in the picture below:

**camera_frame** - Attached to the camera sensor. Represents the viewport of the camera and allows to express relative pose in the standard convention.

**camera_optical_frame** - The "optical" frame of the camera. Located at the same position as the camera_frame, but uses a slightly different convention for the axis orientation (see REP 103).

### Usage
The package does not provide any functionality on its own. Rather, it is used by a variety of other components.
For example, if you want to visualize the model in RViz.
