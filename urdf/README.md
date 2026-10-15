# URDF folder

This folder contains the URDF files for the robot description. 

## Xacro files

Xacro (XML Macros) is a tool that extends the standard URDF format by allowing the use of XML macros to generate URDF files. Xacro files, with the `.xacro` extension, enable you to write more flexible and reusable robot descriptions by defining reusable elements and parameters. This can help to manage complex robot models by allowing for the inclusion of parameters and the use of macros for repetitive structures.

For more information on the URDF file structure, please refer to the [ROS wiki](https://wiki.ros.org/urdf).

## Files 

- `tag.urdf`: Simple blue cylinder to represent a UWB tag
- `unicycle.urdf.xacro`: Unicycle with all the necessary plugin declarations for the Gazebo simulations.
