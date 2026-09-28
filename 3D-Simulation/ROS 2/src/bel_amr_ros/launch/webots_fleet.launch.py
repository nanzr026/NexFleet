"""Three local planners and three peer agents; no centralized allocator."""
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    nodes=[]
    for rid,battery in [('amr_01',90.0),('amr_02',65.0),('amr_03',40.0)]:
        parameters={'robot_id':rid,'initial_battery':battery}
        nodes.append(Node(package='bel_amr_ros',executable='webots_navigation',
            name=rid+'_navigation',parameters=[parameters],output='screen'))
        nodes.append(Node(package='bel_amr_ros',executable='webots_p2p',
            name=rid+'_peer',parameters=[dict(parameters,enable_ai=False,pickup_capable=False)],output='screen'))
    return LaunchDescription(nodes)
