"""Opt-in graph advice; navigation executables and all safety logic are unchanged."""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    root=os.environ["EDGE_ROOT"]
    processes=[]
    for rid,battery in [("amr_01",90.),("amr_02",65.),("amr_03",40.)]:
        processes.append(Node(package="bel_amr_ros",executable="webots_navigation",
            name=rid+"_navigation",parameters=[dict(robot_id=rid,initial_battery=battery)],output="screen"))
        processes.append(ExecuteProcess(cmd=["python3","-m","bel_amr_ros.graph_peer",
            "--ros-args","-r","__node:="+rid+"_graph_peer","-p","robot_id:="+rid,
            "-p","initial_battery:="+str(battery),"-p","enable_ai:=false",
            "-p","pickup_capable:=false"],output="screen"))
        if os.environ.get("EDGE_DISABLE_INFERENCE")=="1":
            continue
        processes.append(ExecuteProcess(cmd=[root+"/.venv-edge/bin/python","-m",
            "bel_amr_ros.graph_agent","--ros-args","-r","__node:="+rid+"_graph_agent",
            "-p","robot_id:="+rid],output="screen"))
    return LaunchDescription(processes)
