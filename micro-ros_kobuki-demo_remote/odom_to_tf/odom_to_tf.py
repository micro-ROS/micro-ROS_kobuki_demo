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
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

class OdomToTf(Node):

    def __init__(self):
        super().__init__('odom_to_tf')

        self.last_position = Vector3()
        self.last_orientation = Quaternion()
        self.last_orientation.x = 0.0
        self.last_orientation.y = 0.0
        self.last_orientation.z = 0.0
        self.last_orientation.w = 1.0

        self.sub_robot_pose = self.create_subscription(Vector3, "/robot_pose", self.robot_pose_callback, QoSProfile(depth=10))
        self.pub_tf = self.create_publisher(TFMessage, "/tf", QoSProfile(depth=10))

        self.tf_timer = self.create_timer(0.05, self.tf_timer_callback)

    
    def robot_pose_callback(self, msg):
        self.last_position.x = msg.x;
        self.last_position.y = msg.y;
        self.last_orientation.z = math.sin(msg.z / 2.0);
        self.last_orientation.w = math.cos(msg.z / 2.0);


    def tf_timer_callback(self):
        msg = TransformStamped()
        msg.header.frame_id = "/map"
        msg.header.stamp = Clock().now().to_msg()
        msg.child_frame_id = "/base_footprint"
        msg.transform.translation = self.last_position
        msg.transform.rotation = self.last_orientation
        tfmsg = TFMessage()
        tfmsg.transforms = [msg]
        self.pub_tf.publish(tfmsg)


def main(args=None):
    rclpy.init(args=args)

    node = OdomToTf()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()