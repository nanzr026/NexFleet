"""Three independent local trainers; explicitly synthetic bootstrap for the demo."""
import uuid

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    run_id = str(uuid.uuid4())
    return LaunchDescription([
        Node(package='bel_amr_ros', executable='federated_agent',
             name=f'{robot_id}_federated', output='screen',
             parameters=[{'robot_id': robot_id, 'run_id': run_id,
                          'bootstrap': True, 'rounds': 3}])
        for robot_id in ['amr_01', 'amr_02', 'amr_03']
    ])
