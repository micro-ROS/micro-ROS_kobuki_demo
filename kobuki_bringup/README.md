General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The kobuki_bringup package

This ROS 2 package starts the ROS 2 side of the microROS demo based on a [Kobuki (Turtlebot 2)](http://kobuki.yujinrobot.com/about2/) and an [Olimex STM32-E407 board](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware).

In detail, it starts a robot_state_publisher and rviz2 (with the UDRF from [kobuki_description](../kobuki_description)) and the [odom_to_tf](../odom_to_tf) node.