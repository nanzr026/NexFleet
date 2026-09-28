from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nodes = [
        DeclareLaunchArgument('start_bridge', default_value='false'),
        DeclareLaunchArgument('start_controllers', default_value='true'),
    ]

    robot_configs = [
        ('amr_01', 90.0),
        ('amr_02', 65.0),
        ('amr_03', 40.0),
    ]

    for robot_id, battery in robot_configs:

        # -----------------------------------------
        # ROS 2 <-> GAZEBO BRIDGE
        # -----------------------------------------

        gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{robot_id}_gz_bridge',
            condition=IfCondition(LaunchConfiguration('start_bridge')),
            output='screen',
            arguments=[
                f'/model/{robot_id}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/model/{robot_id}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/model/{robot_id}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/model/{robot_id}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            ]
        )

        # -----------------------------------------
        # LOCAL ROBOT CONTROLLER
        # -----------------------------------------

        controller = Node(
            package='bel_amr_ros',
            executable='robot_controller',
            name=f'{robot_id}_controller',
            condition=IfCondition(LaunchConfiguration('start_controllers')),
            output='screen',
            parameters=[
                {
                    'robot_id': robot_id,
                    'initial_battery': battery
                }
            ]
        )

        # -----------------------------------------
        # LOCAL P2P COMMUNICATION AGENT
        # -----------------------------------------

        p2p_agent = Node(
            package='bel_amr_ros',
            executable='p2p_agent',
            name=f'{robot_id}_p2p',
            condition=IfCondition(LaunchConfiguration('start_controllers')),
            output='screen',
            parameters=[
                {
                    'robot_id': robot_id
                }
            ]
        )

        nodes.append(gz_bridge)
        nodes.append(controller)
        nodes.append(p2p_agent)

    nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='pallet_bridge', condition=IfCondition(LaunchConfiguration('start_bridge')), output='screen', arguments=[
            '/model/pallet_box_mobile/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/amr_02/product_01/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/amr_02/product_01/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/amr_02/product_01/attached@std_msgs/msg/String[gz.msgs.StringMsg']))
    return LaunchDescription(nodes)
