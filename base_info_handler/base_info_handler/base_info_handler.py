import math

import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSDurabilityPolicy
from rclpy.node import Node
from rclpy.logging import get_logger
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import TransformStamped, TwistWithCovariance, Twist
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from sensor_msgs.msg import Joy
from drive_base_msgs.msg import BaseInfo, CommandHeader, TRVCommand

class BaseInfoHandler(Node):
    URDF_PARAM = "urdf"

    def __init__(self):
        super().__init__('base_info_handler')
        self._world_link = "/map"
        self._base_link = "/base_footprint"
        self._last_cmd = TRVCommand()
        try:
            self.declare_parameter(self.URDF_PARAM)
        except AttributeError:
            pass  # ROS 2 prior to dashing does not have this method, so ignore AttributeErrors
        
        urdf_filename = self.get_parameter(self.URDF_PARAM).value        
        get_logger(self.get_name()).info("Robot URDF: %s" % urdf_filename)

        self._odom = Odometry(header=Header(frame_id=self._world_link),
            child_frame_id=self._base_link,
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
        )
        self._transform = TransformStamped(
            header=Header(frame_id=self._world_link),
            child_frame_id=self._base_link)

        self.sub_robot_pose = self.create_subscription(BaseInfo, "/base_info", self.base_info_cb,
            qos_profile=qos_profile_sensor_data)

        # these bridge from standard geometry_msgs/Twist to drive_base_msgs/TRVCommand
        self.pub_cmd = self.create_publisher(TRVCommand, "/drive_cmd", qos_profile=QoSProfile(depth=1))
        self.sub_joy = self.create_subscription(Joy, "/joy", self.joy_cb, qos_profile=QoSProfile(depth=1))

        # standard odom topic. every message replaces the last, so we only need one
        self.pub_odom = self.create_publisher(Odometry, "/odom", qos_profile=QoSProfile(depth=1))
        # tf topic. messages build a history, so give subscribers a chance to get them all
        # CHECKME: use transient local?
        self.pub_tf = self.create_publisher(TFMessage, "/tf", qos_profile=QoSProfile(depth=10))

        # publish URDF "latched" -- doesn't seem to work, though
        if urdf_filename is not None:
            self.pub_urdf = self.create_publisher(String, "/robot_description", qos_profile=QoSProfile(depth=1, 
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL))
            self._urdf_msg = String(data=urdf_filename)
            self.pub_urdf.publish(self._urdf_msg)
            self.timer_desc = self.create_timer(1, self.description_timer_cb)

        
    def description_timer_cb(self):
        self.pub_urdf.publish(self._urdf_msg)

    def joy_cb(self, msg):
        in_motion = False
        cmd_stop = False
        period_elapsed = False

        cmd_tv = msg.axes[4]
        cmd_rv = msg.axes[3]
        cmd_enable = msg.buttons[5]

        if self._last_cmd.translational_velocity != 0.0 or self._last_cmd.rotational_velocity != 0.0:
            in_motion = True
        if not cmd_enable or (cmd_tv == 0.0 and cmd_rv == 0):
            cmd_stop = True

        last_time = rclpy.time.Time().from_msg(self._last_cmd.header.stamp)
        cur_time = rclpy.time.Time().from_msg(msg.header.stamp)        
        period_elapsed = ((cur_time - last_time).nanoseconds < 90000000)

        send_cmd = False
        # always send stop commands when we're moving. otherwise only send when period is elapsed
        if in_motion:
            if cmd_stop:
                send_cmd = True
            elif period_elapsed:
                send_cmd = True
        elif not cmd_stop or period_elapsed:
            send_cmd = True

        if send_cmd:
            hdr = CommandHeader(stamp=msg.header.stamp, command_id=self._last_cmd.header.command_id + 1)
            trv_cmd = TRVCommand(header=hdr)
            
            # FIXME make magic numbers here configurable
            trv_cmd.translational_velocity=cmd_tv * 0.8
            trv_cmd.rotational_velocity=cmd_rv * 2.0
            self._last_cmd = trv_cmd        
        
            self.pub_cmd.publish(trv_cmd)

    def base_info_cb(self, msg):
        self._odom.header.stamp = msg.stamp
        self._odom.pose.pose.position.x = msg.x
        self._odom.pose.pose.position.y = msg.y
        self._odom.pose.pose.orientation.z = math.sin(msg.orientation / 2.0)
        self._odom.pose.pose.orientation.w = math.cos(msg.orientation / 2.0)
        self._odom.twist.twist.linear.x = msg.forward_velocity
        self._odom.twist.twist.angular.z = msg.rotational_velocity
        self.pub_odom.publish(self._odom)        

        self._transform.header.stamp = msg.stamp
        pos = self._odom.pose.pose.position
        self._transform.transform.translation = Vector3(x=pos.x, y=pos.y, z=pos.z)
        self._transform.transform.rotation = self._odom.pose.pose.orientation
        tfmsg = TFMessage()
        tfmsg.transforms = [self._transform]
        self.pub_tf.publish(tfmsg)



def main(args=None):
    rclpy.init(args=args)

    node = BaseInfoHandler()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()