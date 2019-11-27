General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The base_info_handler package

This ROS 2 package provides the base_info_handler node (implemented in Python) to translate the drive_base_msgs/BaseInfo data received via micro-ROS from the Olimex board to the standard message ROS expects a base driver to produce (nav_msgs/Odometry and geometry_msgs/TFMessage). This allows us to conserve bandwidth to the base, while still integrating with regular packages.

In addition, it contains the circle_odom_publisher node to simulate a circular driving Kobuki.