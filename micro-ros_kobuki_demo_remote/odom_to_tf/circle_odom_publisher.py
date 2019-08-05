# Copyright (c) 2019 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/micro-ROS/micro-ROS_kobuki_demo.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import rclpy
from rclpy.qos import QoSProfile
from rclpy.clock import Clock
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class CircleOdomPublisher(Node):

    def __init__(self):
        super().__init__('circle_odom_publisher')

        self.pub_robot_pose = self.create_publisher(Vector3, "/robot_pose", QoSProfile(depth=10))

        self.angle = 0.0

        self.circle_timer = self.create_timer(0.05, self.timer_callback)


    def timer_callback(self):
        self.angle = self.angle + 0.03
        while self.angle > 2 * math.pi:
            self.angle = self.angle - 2 * math.pi
        msg = Vector3()
        msg.x = 1.0 * math.cos(self.angle)
        msg.y = 1.0 * math.sin(self.angle)
        msg.z = self.angle + math.pi / 2.0
        self.pub_robot_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CircleOdomPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()