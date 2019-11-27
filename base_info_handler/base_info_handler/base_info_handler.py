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
from drive_base_msgs.msg import BaseInfo

class BaseInfoHandler(Node):
    URDF_PARAM = "urdf"

    def __init__(self):
        super().__init__('base_info_handler')
        self._world_link = "/map"
        self._base_link = "/base_footprint"
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
        # standard odom topic. every message replaces the last, so we only need one
        self.pub_odom = self.create_publisher(Odometry, "/odom", qos_profile=QoSProfile(depth=1))
        # tf topic. messages build a history, so give subscribers a chance to get them all
        # CHECKME: use transient local?
        self.pub_tf = self.create_publisher(TFMessage, "/tf", qos_profile=QoSProfile(depth=10))

        # publish URDF "latched"
        if urdf_filename is not None:
            self.pub_urdf = self.create_publisher(String, "/robot_description", qos_profile=QoSProfile(depth=1, 
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL))
            self._urdf_msg = String(data=urdf_filename)
            self.pub_urdf.publish(self._urdf_msg)
            self.timer_desc = self.create_timer(0.1, self.description_timer_cb)
        
    def description_timer_cb(self):
        self.pub_urdf.publish(self._urdf_msg)

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