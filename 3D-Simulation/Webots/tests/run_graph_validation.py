"""Fresh physical holdout, three DDS FedAvg peers, then AI-advised task execution."""
import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
from run_coordination import make_world, scenario, ROOT
IDS=("amr_01","amr_02","amr_03")

def probe(limit):
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    n=rclpy.create_node("graph_validation_probe")
    physics=[]; events=[]; statuses=[]; results={}; telemetry={}; reservations={}
    bids={}; claims={}; audits={}; decisions={}
    qos=QoSProfile(depth=30,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String,"/edge/status",lambda m:statuses.append(json.loads(m.data)),qos)
    n.create_subscription(String,"/validation/physics",lambda m:physics.append(json.loads(m.data)),10)
    n.create_subscription(String,"/task/result",lambda m:results.update({json.loads(m.data)["task_id"]:json.loads(m.data)}),10)
    n.create_subscription(String,"/edge/bid_audit",lambda m:audits.update({json.loads(m.data)["robot_id"]:json.loads(m.data)}),10)
    n.create_subscription(String,"/edge/decision",lambda m:decisions.update({json.loads(m.data)["robot_id"]:json.loads(m.data)}),10)
    pose_received={}
    pubs={}
    for rid in IDS:
        n.create_subscription(PoseStamped,f"/model/{rid}/pose",lambda m,r=rid:pose_received.update({r:time.monotonic()}),1)
        for topic, destination in (("telemetry",telemetry),("reservation",reservations),
                                   ("task_bid",bids),("task_claim",claims)):
            n.create_subscription(String,f"/{rid}/{topic}",lambda m,r=rid,d=destination:d.update({r:json.loads(m.data)}),10)
        n.create_subscription(String,f"/{rid}/coordination_event",lambda m:events.append(json.loads(m.data)),100)
        pubs[rid]=n.create_publisher(String,f"/{rid}/assigned_task",10)
    announce=n.create_publisher(String,"/task/announcement",10)
    fresh=n.create_publisher(String,"/edge/train_fresh",10)
    run=os.environ["EDGE_RUN"]
    out=ROOT/"Webots/edge_ai_runs"/run
    report=dict(status="FAIL",run=run)
    def wait(condition,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)
            if physics and physics[-1]["collision_episodes"]:
                raise AssertionError("Physical contact")
            if condition():return
        raise RuntimeError("Timeout; statuses="+str(statuses[-8:])+" results="+str(results))
    def round_done(version):
        peers={s["robot_id"]:s for s in statuses if s["event"]=="AGGREGATED" and s["version"]==version}
        return len(peers)==3 and len({s["digest"] for s in peers.values()})==1
    try:
        nodes,edges,starts,goals,yaws=scenario("C")
        wait(lambda:physics and physics[-1]["time"]>3 and round_done(3) and all(
            time.monotonic()-pose_received.get(r,0)<1.0 and
            telemetry.get(r,{}).get("bootstrap_ready") and r in reservations and
            math.dist((telemetry[r]["x"],telemetry[r]["y"]),nodes[starts[i]])<.25
            for i,r in enumerate(IDS)),900)
        initial=physics[-1]
        spec=dict(run=run,members=list(IDS),nodes=nodes,edges=edges)
        pending={}
        for i,rid in enumerate(IDS):
            x,y=nodes[goals[i]]
            pending[rid]=String(data=json.dumps(dict(task_id=run+"_"+rid,
                kind="navigate",pickup_x=x,pickup_y=y,coordination=spec)))
        last_send={}
        def accepted():
            for rid,msg in pending.items():
                if (reservations.get(rid,{}).get("run")!=run and
                    time.monotonic()-pose_received.get(rid,0)<1. and
                    time.monotonic()-last_send.get(rid,0)>2):
                    pubs[rid].publish(msg)
                    last_send[rid]=time.monotonic()
            return all(reservations.get(r,{}).get("run")==run for r in IDS)
        wait(accepted,120)
        wait(lambda:all(run+"_"+r in results for r in IDS),limit)
        assert all(results[run+"_"+r]["status"]=="COMPLETED" for r in IDS),results
        coordinated_final=physics[-1]
        for i,r in enumerate(IDS):
            assert math.dist(coordinated_final["poses"][r][:2],nodes[goals[i]])<.25
        wait(lambda:len({s["robot_id"] for s in statuses if s["event"]=="EVALUATED"})==3,40)
        fresh.publish(String(data=json.dumps(dict(run=run))))
        wait(lambda:round_done(4),120)
        # Frozen holdout evaluations above are saved before this fresh-data learning round.
        wait(lambda:all(telemetry.get(r,{}).get("state")=="IDLE" or
                        telemetry.get(r,{}).get("robot_state")=="IDLE" for r in IDS),30)
        task=dict(task_id=run+"_ADVISED",kind="navigate",pickup_x=7.,pickup_y=-10.)
        before_bid=physics[-1]
        announce.publish(String(data=json.dumps(task)))
        wait(lambda:task["task_id"] in results,limit)
        result=results[task["task_id"]]
        assert result["status"]=="COMPLETED",result
        assert all(bids.get(r,{}).get("task_id")==task["task_id"] for r in IDS),bids
        assert all(r in audits and audits[r]["advice"].get("version")==4 for r in IDS),audits
        winner=sorted((b for b in bids.values() if b["available"]),key=lambda b:(-b["bid"],b["robot_id"]))[0]["robot_id"]
        assert winner==result["robot_id"]
        assert len(decisions)==3 and all(d["winner"]==winner and d["task_id"]==task["task_id"] for d in decisions.values()),decisions
        assert claims[winner]["task_id"]==task["task_id"]
        assert math.dist(physics[-1]["poses"][winner][:2],(7.,-10.))<.3
        assert math.dist(before_bid["poses"][winner][:2],physics[-1]["poses"][winner][:2])>2.5
        assert any(e["event"]=="PIBT RESOLUTION" for e in events)
        report.update(status="PASS",initial=initial,coordinated_final=coordinated_final,
            before_bid=before_bid,final=physics[-1],bids=bids,claims=claims,audits=audits,
            winner=winner,decisions=decisions,minimum_separation=physics[-1]["minimum_separation"],
            collisions=physics[-1]["collision_episodes"])
    except Exception as error:
        report["error"]=str(error)
        raise
    finally:
        report.update(statuses=statuses,results=results,events=events,physics=physics,
                      telemetry=telemetry)
        (out/"physical_result.json").write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in
                          ("physics","events","statuses","telemetry")},indent=2),flush=True)
        n.destroy_node()
        rclpy.shutdown()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--probe",action="store_true")
    parser.add_argument("--limit",type=float,default=1800)
    args=parser.parse_args()
    if args.probe:return probe(args.limit)
    run="graph_"+str(time.time_ns())
    out=ROOT/"Webots/edge_ai_runs"/run
    out.mkdir(parents=True)
    env=dict(os.environ,EDGE_ROOT=str(ROOT),EDGE_RUN=run,ROS_DOMAIN_ID="28",
             LP_NUM_THREADS="2",OMP_NUM_THREADS="1")
    world=make_world("C")
    processes=[];logs=[]
    print("RUN="+run,flush=True)
    try:
        for label,command in [
            ("world",["/usr/local/bin/webots","--batch","--minimize","--no-rendering",
                       "--mode=fast","--stdout","--stderr","--port=1238",str(world)]),
            ("fleet",["ros2","launch","bel_amr_ros","webots_graph_fleet.launch.py"])]:
            log=(out/(label+".log")).open("w");logs.append(log)
            processes.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        child=subprocess.Popen(["python3",__file__,"--probe","--limit",str(args.limit)],
                               env=env,start_new_session=True)
        processes.append(child)
        raise SystemExit(child.wait())
    finally:
        for process in reversed(processes):
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for process in processes:
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)
        for log in logs:log.close()

if __name__=="__main__":main()
