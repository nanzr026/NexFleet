"""One transport bridge for the fleet; no operational coordination here."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mappings = []
    for robot_id in ('amr_01', 'amr_02', 'amr_03'):
        prefix = f'/model/{robot_id}'
        mappings.extend([
            f'{prefix}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'{prefix}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'{prefix}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ])
    return LaunchDescription([Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='edgefleet_bridge', arguments=mappings, output='screen',
    )])
