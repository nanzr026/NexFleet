"""Controlled unavailable-inference test: existing graph bidders, no learner processes."""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
IDS=("amr_01","amr_02","amr_03")

def probe():
    import rclpy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    rclpy.init();n=rclpy.create_node("graph_failsafe_probe")
    physics=[];states={};poses={};audits={};bids={};results={};decisions={}
    n.create_subscription(String,"/validation/physics",lambda m:physics.append(json.loads(m.data)),10)
    n.create_subscription(String,"/task/result",lambda m:results.update({json.loads(m.data)["task_id"]:json.loads(m.data)}),10)
    n.create_subscription(String,"/edge/bid_audit",lambda m:audits.update({json.loads(m.data)["robot_id"]:json.loads(m.data)}),10)
    n.create_subscription(String,"/edge/decision",lambda m:decisions.update({json.loads(m.data)["robot_id"]:json.loads(m.data)}),10)
    for rid in IDS:
        n.create_subscription(String,f"/{rid}/telemetry",lambda m,r=rid:states.update({r:json.loads(m.data)}),10)
        n.create_subscription(String,f"/{rid}/task_bid",lambda m,r=rid:bids.update({r:json.loads(m.data)}),10)
        n.create_subscription(PoseStamped,f"/model/{rid}/pose",lambda m,r=rid:poses.update({r:time.monotonic()}),1)
    pub=n.create_publisher(String,"/task/announcement",10)
    def wait(condition,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)
            if physics and physics[-1]["collision_episodes"]:
                raise AssertionError("Contact")
            if condition():return
        raise RuntimeError("Failsafe test timeout")
    report=dict(status="FAIL",controlled_failure="No graph learner/inference processes launched",
                run=os.environ["EDGE_RUN"])
    try:
        wait(lambda:physics and physics[-1]["time"]>3 and all(
            states.get(r,{}).get("bootstrap_ready") and
            time.monotonic()-poses.get(r,0)<1 for r in IDS),900)
        initial=physics[-1]
        task=dict(task_id=os.environ["EDGE_RUN"],kind="navigate",pickup_x=12.,pickup_y=-10.)
        pub.publish(String(data=json.dumps(task)))
        wait(lambda:task["task_id"] in results,1200)
        result=results[task["task_id"]]
        assert result["status"]=="COMPLETED"
        assert len(audits)==3 and all(a["event"]=="AI_FAILURE" and
            a["advice"]=={} and a["final"]==a["deterministic"] for a in audits.values()),audits
        winner=result["robot_id"]
        assert len(decisions)==3 and all(d["winner"]==winner for d in decisions.values())
        assert math.dist(physics[-1]["poses"][winner][:2],(12.,-10.))<.3
        assert math.dist(initial["poses"][winner][:2],physics[-1]["poses"][winner][:2])>2.5
        wait(lambda:states[winner]["state"]=="IDLE",20)
        report.update(status="PASS",task=task,result=result,winner=winner,
            initial=initial,final=physics[-1],audits=audits,bids=bids,decisions=decisions,
            contacts=physics[-1]["collision_episodes"],final_robot_state=states[winner]["state"],
            completion_time_sim_seconds=physics[-1]["time"]-initial["time"])
    except Exception as error:
        report["error"]=str(error);raise
    finally:
        report.update(physics=physics,telemetry=states)
        out=ROOT/"Webots/edge_ai_runs"/os.environ["EDGE_RUN"]
        (out/"failsafe_result.json").write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ("physics","telemetry")},indent=2),flush=True)
        n.destroy_node();rclpy.shutdown()

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--probe",action="store_true");args=parser.parse_args()
    if args.probe:return probe()
    run="failsafe_"+str(time.time_ns());out=ROOT/"Webots/edge_ai_runs"/run;out.mkdir(parents=True)
    env=dict(os.environ,EDGE_ROOT=str(ROOT),EDGE_RUN=run,EDGE_DISABLE_INFERENCE="1",
             ROS_DOMAIN_ID="30",LP_NUM_THREADS="2",OMP_NUM_THREADS="1")
    world=(ROOT/"Webots/worlds/bel_warehouse_ros.wbt").read_text()
    for i,y in enumerate((-10,-6,-2),1):
        block=f'DEF AMR{i:02} AMR {{ translation 15.8 {y} 0 rotation 0 0 1 {math.pi} name "AMR{i:02}" controller "amr_ros" }}'
        world=re.sub(r'DEF AMR'+f'{i:02}'+r' AMR \{[^}]*\}',lambda m:block,world)
    world=world.replace('controller "ros_observer"','controller "coordination_observer"')
    path=ROOT/"Webots/worlds/graph_failsafe.wbt";path.write_text(world)
    processes=[];logs=[]
    print("FAILSAFE="+run,flush=True)
    try:
        for label,command in [
            ("world",["/usr/local/bin/webots","--batch","--minimize","--no-rendering",
                       "--mode=fast","--stdout","--stderr","--port=1239",str(path)]),
            ("fleet",["ros2","launch","bel_amr_ros","webots_graph_fleet.launch.py"])]:
            log=(out/(label+".log")).open("w");logs.append(log)
            processes.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        child=subprocess.Popen(["python3",__file__,"--probe"],env=env,start_new_session=True)
        processes.append(child);raise SystemExit(child.wait())
    finally:
        for process in reversed(processes):
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for process in processes:
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)
        for log in logs:log.close()
if __name__=="__main__":main()
