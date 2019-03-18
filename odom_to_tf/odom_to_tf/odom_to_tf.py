import math

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
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

        self.sub_position = self.create_subscription(Vector3, "/odom_position", self.odom_position_callback)
        self.sub_orientation = self.create_subscription(Quaternion, "/odom_orientation", self.odom_orientation_callback)
        self.pub_tf = self.create_publisher(TFMessage, "/tf")

        self.tf_timer = self.create_timer(0.05, self.tf_timer_callback)

    
    def odom_position_callback(self, msg):
        self.last_position = msg;


    def odom_orientation_callback(self, msg):
        self.last_orientation = msg;


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