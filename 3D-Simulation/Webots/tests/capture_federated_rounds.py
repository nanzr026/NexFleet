"""Independent DDS capture of a seed-only replay; compares with physical-run models."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
ROOT=Path(__file__).resolve().parents[2]
run="federated_capture_"+str(time.time_ns())
os.environ["ROS_DOMAIN_ID"]="29"
out=ROOT/"Webots/edge_ai_runs"/run
out.mkdir(parents=True)
rclpy.init();node=rclpy.create_node("federated_capture")
updates=[];statuses=[]
qos=QoSProfile(depth=30,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def capture(msg):
    value=json.loads(msg.data)
    if value.get("run")==run:
        updates.append(dict(message=value,utf8_bytes=len(msg.data.encode())))
node.create_subscription(String,"/edge/model_update",capture,qos)
node.create_subscription(String,"/edge/status",lambda m:statuses.append(json.loads(m.data)),qos)
processes=[];logs=[]
env=dict(os.environ,EDGE_ROOT=str(ROOT),EDGE_RUN=run,OMP_NUM_THREADS="1")
report=dict(run=run,status="FAIL")
try:
    for rid in ("amr_01","amr_02","amr_03"):
        log=(out/(rid+".log")).open("w");logs.append(log)
        processes.append(subprocess.Popen([str(ROOT/".venv-edge/bin/python"),"-m",
            "bel_amr_ros.graph_agent","--ros-args","-r","__node:="+rid+"_capture_agent",
            "-p","robot_id:="+rid],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    end=time.monotonic()+420
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
        done={s["robot_id"] for s in statuses if s.get("run")==run and s["event"]=="AGGREGATED" and s["version"]==3}
        if len(done)==3 and len(updates)>=9:
            report["status"]="PASS";break
    assert report["status"]=="PASS","DDS capture timeout"
finally:
    report.update(updates=updates,statuses=statuses)
    (out/"capture.json").write_text(json.dumps(report,indent=2))
    for process in processes:
        if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
    for process in processes:
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)
    for log in logs:log.close()
    node.destroy_node();rclpy.shutdown()
    print("CAPTURE="+run,flush=True)
