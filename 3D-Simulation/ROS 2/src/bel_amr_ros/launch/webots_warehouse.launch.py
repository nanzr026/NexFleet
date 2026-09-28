from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    nodes=[]
    for rid,battery in [("amr_01",90.),("amr_02",65.),("amr_03",40.)]:
        for mode in ("navigation","peer"):
            nodes.append(ExecuteProcess(cmd=["python3","-m","bel_amr_ros.warehouse_nodes",mode,
                "--ros-args","-r","__node:="+rid+"_warehouse_"+mode,"-p","robot_id:="+rid,
                "-p","initial_battery:="+str(battery),"-p","pickup_capable:=true"]+
                (["-p","enable_ai:=false"] if mode=="peer" else []),output="screen"))
    return LaunchDescription(nodes)
