import math

import sys, termios, tty, os, time, threading

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleKeyboardTeleop(Node):


    def read_char_from_stdin(self):
        fd = sys.stdin.fileno()
        original_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.last_input_char != 'e':
                self.last_input_char = sys.stdin.read(1)
                time.sleep(0.01)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original_settings)
            self.destroy_node()
            rclpy.shutdown()

    
    def print_help(self):
        print("  Linear speed    Angular speed    Control    E=Exit")
        print("       Q                W           U I O           ")
        print("       A                S           J   L           ")
        print("                                    M , .           ")


    def __init__(self):
        super().__init__('simple_keyboard_teleop')

        self.print_help()

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel")

        self.linear_speed_percent = 50
        self.angular_speed_percent = 50

        self.last_input_char = ' '

        self.MAX_LINEAR_SPEED = 0.5
        self.MAX_ANGULAR_SPEED = math.pi / 10.0

        self.circle_timer = self.create_timer(0.1, self.timer_callback)

        keyboard_thread = threading.Thread(target=SimpleKeyboardTeleop.read_char_from_stdin, args=(self,))
        keyboard_thread.start()


    def timer_callback(self):
        if self.last_input_char == 'q':
            self.linear_speed_percent = min([self.linear_speed_percent + 10, 100])
        elif self.last_input_char == 'a':
            self.linear_speed_percent = max([self.linear_speed_percent - 10, 0])

        linear_speed = 0.0
        if self.last_input_char in ['u', 'i', 'o']:
            linear_speed = self.linear_speed_percent * self.MAX_LINEAR_SPEED / 100.0
        elif self.last_input_char in ['m', ',', '.']:
            linear_speed = - self.linear_speed_percent * self.MAX_LINEAR_SPEED / 100.0

        if self.last_input_char == 'w':
            self.angular_speed_percent = min([self.angular_speed_percent + 10, 100])
        elif self.last_input_char == 's':
            self.angular_speed_percent = max([self.angular_speed_percent - 10, 0])

        angular_speed = 0.0
        if self.last_input_char in ['u', 'j', 'm']:
            angular_speed = self.angular_speed_percent * self.MAX_ANGULAR_SPEED / 100.0
        elif self.last_input_char in ['o', 'l', '.']:
            angular_speed = - self.angular_speed_percent * self.MAX_ANGULAR_SPEED / 100.0

        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.pub_cmd_vel.publish(msg)

        self.last_input_char = ' '


def main(args=None):
    rclpy.init(args=args)
    node = SimpleKeyboardTeleop()
    rclpy.spin(node)


if __name__ == '__main__':
    main()