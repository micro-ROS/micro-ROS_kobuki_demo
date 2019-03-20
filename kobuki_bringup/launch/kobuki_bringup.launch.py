
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('kobuki_description'),
                        'urdf', 'kobuki.urdf')

    config = os.path.join(get_package_share_directory('kobuki_bringup'),
                        'config', 'kobuki.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen', arguments=[urdf]),
        Node(
            package='odom_to_tf',
            node_executable='odom_to_tf',
            output='screen'),
        Node(
            package='rviz2',
            node_executable='rviz2',
            arguments=['-d', config])
])