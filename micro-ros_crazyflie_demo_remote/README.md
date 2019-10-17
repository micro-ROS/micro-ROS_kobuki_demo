General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The micro-ros_kobuki_demo_remote package

This ROS 2 package provides nodes and launch files for the ROS 2 side of the micro-ROS demo based on a [Kobuki (Turtlebot 2)](http://kobuki.yujinrobot.com/about2/) and an [Olimex STM32-E407 board](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware). Most important, it contains a tiny node named [odom_to_tf](odom_to_tf/odom_to_tf.py) (implemented in Python) to translate the odometry data received via micro-ROS from the Olimex board to TF.

For starting the ROS 2 side of the demo, this package provides three launch files:

* `remote_without_control.launch.py` starts a robot_state_publisher, rviz2 (with the UDRF from [micro-ros_kobuki_demo_robot-description](../micro-ros_kobuki_demo_robot-description)) and the odom_to_tf node.

  For keyboard control of the robot, you may start the teleop_twist_keyboard node by `ros2 run teleop_twist_keyboard teleop_twist_keyboard` in addition.

* `remote_teleop_joy.launch.py` starts robot_state_publisher, rviz2, odom_to_tf, and a teleop_twist_joy node for remote control by a joystick.

* `remote_simulate_circular_odom.launch.py` provides a test setup for the ROS 2 side of the demo by starting robot_state_publisher, rviz2, and odom_to_tf, together a tiny node named [circular_odom_publisher.py](odom_to_tf/circular_odom_publisher.py), which simulates odometry data from a circular driving.
