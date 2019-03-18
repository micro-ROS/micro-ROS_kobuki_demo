import math

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

class CircleOdomPublisher(Node):

    def __init__(self):
        super().__init__('circle_odom_publisher')

        self.pub_position = self.create_publisher(Vector3, "/odom_position")
        self.pub_orientation = self.create_publisher(Quaternion, "/odom_orientation")

        self.angle = 0.0

        self.circle_timer = self.create_timer(0.05, self.timer_callback)


    def timer_callback(self):
        self.angle = self.angle + 0.03
        while self.angle > 2 * math.pi:
            self.angle = self.angle - 2 * math.pi
        position_msg = Vector3()
        orientation_msg = Quaternion()
        position_msg.x = 1.0 * math.cos(self.angle)
        position_msg.y = 1.0 * math.sin(self.angle)
        orientation_msg.z = math.sin((self.angle + 0.5 * math.pi) / 2.0)
        orientation_msg.w = math.cos((self.angle + 0.5 * math.pi) / 2.0)
        self.pub_position.publish(position_msg)
        self.pub_orientation.publish(orientation_msg)


def main(args=None):
    rclpy.init(args=args)

    node = CircleOdomPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()