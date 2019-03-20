import math

import sys, select, termios, tty, os, time

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from geometry_msgs.msg import Twist


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
}


class SimpleKeyboardTeleop(Node):


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        sr, sw, se = select.select([sys.stdin], [], [], 0.5)
        key = 'k'
        if sr != [] or sw != [] or se != []:
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def vels(self,speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)


    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

        super().__init__('simple_keyboard_teleop')

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel")

        print(msg)


    def run(self):
        speed = 0.5
        turn = 1.0
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0
        status = 0

        try:
            while(1):
                key = self.getKey()
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]
                    th = moveBindings[key][3]
                elif key in speedBindings.keys():
                    speed = speed * speedBindings[key][0]
                    turn = turn * speedBindings[key][1]

                    print(self.vels(speed,turn))
                    if (status == 14):
                        print(msg)
                    status = (status + 1) % 15
                else:
                    x = 0.0
                    y = 0.0
                    z = 0.0
                    th = 0.0
                    if (key == '\x03'):
                        break

                twist = Twist()
                twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
                self.pub_cmd_vel.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleKeyboardTeleop()
    node.run()


if __name__ == '__main__':
    main()