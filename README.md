# The microROS_kobuki_demo repository

This repository provides a demo of micro-ROS (and in particular its client library features) based on a [Kobuki (Turtlebot 2)](http://kobuki.yujinrobot.com/about2/) and an [Olimex STM32-E407 board](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware).

In detail, this repository contains the following ROS 2 and micro-ROS packages:

* [kobuki_bringup](kobuki_bringup/) provides a launch file to start rviz2, robot_state_publisher and the odom_to_tf node.
* [kobuki_description](kobuki_description/) provides the URDF and meshes and textures for the Kobuki.
* [odom_to_tf](odom_to_tf/) implements a node to translate the odometry data received via microROS from the Olimex board to TF.
* [simple_keyboard_teleop](simple_keyboard_teleop/) provides a keyboard control in the style of [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) from ROS 1.

Technical information on these packages is given in the README.md files in the corresponding subfolders.

## Purpose of the project

The software is not ready for production use. It has neither been developed nor tested for a specific use case. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards (e.g. ISO 26262).

## Requirements, how to build, test, install, use, etc

Clone the repository into a ROS workspace and build it using [colcon](https://colcon.readthedocs.io/).

## License

microROS_kobuki_demo is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open source components included in microROS_kobuki_demo, see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Quality assurance

The colcon_test tool is used for quality assurances, which includes cpplint, uncrustify, flake8, xmllint and various other tools.

## Known issues/limitations

Please notice the following issues/limitations:

* Currently, the demo does not use the standard ROS 2 odometry message types but shorter types due to issues in the middleware.
* When exiting the simple_keyboard_teleop using `e`, an assertion error is thrown.
