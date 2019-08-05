General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The micro-ros_kobuki_demo_remote package

This ROS 2 package starts the ROS 2 side of the micro-ROS demo based on a [Kobuki (Turtlebot 2)](http://kobuki.yujinrobot.com/about2/) and an [Olimex STM32-E407 board](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware).

In detail, it starts a robot_state_publisher and rviz2 (with the UDRF from [micro-ros_kobuki_demo_robot-description](../micro-ros_kobuki_demo_robot-description)) and the [odom_to_tf](../odom_to_tf) node.

This ROS 2 package also provides the odom_to_tf node (implemented in Python) to to translate the odometry data received via micro-ROS from the Olimex board to TF.

In addition, it contains the circular_odom_publisher node to simulate a circular driving Kobuki.