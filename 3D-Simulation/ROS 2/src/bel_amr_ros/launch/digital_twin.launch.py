"""Start one digital twin, initialize its cargo, then start the fleet."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def start(context):
    world = Path(LaunchConfiguration('world').perform(context)).resolve()
    if not world.is_file():
        raise RuntimeError(f'World not found: {world}; pass world:=/absolute/path')
    models = world.parent.parent / 'models'
    simulator = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', str(world)],
        additional_env={'GZ_SIM_RESOURCE_PATH':
                        str(models) + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')},
        output='screen')
    initializer = Node(package='bel_amr_ros', executable='simulation_startup',
                       output='screen')
    fleet = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('bel_amr_ros')) /
        'launch' / 'three_amrs.launch.py')))

    def initialized(event, _context):
        if event.returncode == 0:
            return [fleet]
        return [EmitEvent(event=Shutdown(reason='Digital twin initialization failed'))]

    return [
        RegisterEventHandler(OnProcessExit(target_action=initializer,
                                           on_exit=initialized)),
        RegisterEventHandler(OnProcessExit(target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='Simulator exited'))])),
        simulator, initializer,
    ]


def generate_launch_description():
    default = ''
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'Gazebo Harmonic/worlds/openrobotics_warehouse.sdf'
        if candidate.is_file():
            default = str(candidate)
            break
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default),
        OpaqueFunction(function=start),
    ])
