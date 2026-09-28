"""Physical warehouse tests with separate immutable run directories."""
import argparse,json,math,os,re,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
IDS=["amr_01","amr_02","amr_03"]
def world(name):
    text=(ROOT/"Webots/worlds/bel_warehouse_ros.wbt").read_text()
    for i,(x,y,yaw) in enumerate([(0,-10,0),(0,-6,0),(0,-14,0)],1):
        block=f'DEF AMR{i:02} AMR {{ translation {x} {y} 0 rotation 0 0 1 {yaw} name "AMR{i:02}" controller "amr_ros" }}'
        text=re.sub(r'DEF AMR'+f'{i:02}'+r' AMR \{[^}]*\}',lambda m:block,text)
    text=text.replace('controller "ros_observer"','controller "warehouse_supervisor"')
    text=re.sub(r'DEF PRODUCT_01 Pallet \{[^}]*\}', 'DEF PRODUCT_01 Pallet { translation 4 -10 0 name "PRODUCT_01" }',text)
    if name in ("D1","D2"):text=text.replace('translation 4 -10 0 name "PRODUCT_01"','translation -12 -11 0 name "PRODUCT_01"')
    path=ROOT/f"Webots/worlds/workflow_{name}.wbt";path.write_text(text);return path
def probe(name,limit):
    import rclpy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import LaserScan
    rclpy.init();n=rclpy.create_node("warehouse_probe")
    physics=[];events=[];states={};bids={};decisions={};results={};paths={};fresh={};path_history=[];environments=[];scan_hits=[]
    for topic,dest in [("/validation/physics",physics),("/warehouse/events",events),("/warehouse/task_events",events)]:
        n.create_subscription(String,topic,lambda m,d=dest:d.append(json.loads(m.data)),100)
    n.create_subscription(String,"/warehouse/decisions",lambda m:decisions.update({json.loads(m.data)["robot_id"]:json.loads(m.data)}),10)
    n.create_subscription(String,"/task/result",lambda m:results.update({json.loads(m.data)["task_id"]:json.loads(m.data)}),10)
    for rid in IDS:
        n.create_subscription(PoseStamped,f"/model/{rid}/pose",lambda m,r=rid:fresh.update({r:time.monotonic()}),1)
        for suffix,dest in [("telemetry",states),("task_bid",bids),("path_intention",paths)]:
            n.create_subscription(String,f"/{rid}/{suffix}",lambda m,r=rid,d=dest:d.update({r:json.loads(m.data)}),10)
    n.create_subscription(String,"/warehouse/environment",lambda m:environments.append(json.loads(m.data)),10)
    n.create_subscription(String,"/amr_01/path_intention",lambda m:path_history.append(json.loads(m.data)),100)
    def scan(msg):
        own=states.get("amr_01")
        if not own or not environments or not environments[-1]["blockages"]:return
        for b in environments[-1]["blockages"]:
            if not b["active"]:continue
            for i,d in enumerate(msg.ranges):
                if not math.isfinite(d):continue
                a=own["yaw"]+msg.angle_min+i*msg.angle_increment
                x=own["x"]+.12*math.cos(own["yaw"])+d*math.cos(a)
                y=own["y"]+.12*math.sin(own["yaw"])+d*math.sin(a)
                if abs(x-b["x"])<=b["sx"]/2+.12 and abs(y-b["y"])<=b["sy"]/2+.12:
                    scan_hits.append(dict(time=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,blockage=b["id"],range=d))
                    return
    n.create_subscription(LaserScan,"/model/amr_01/scan",scan,10)
    pub=n.create_publisher(String,"/task/announcement",10)
    environment_command=n.create_publisher(String,"/warehouse/environment_command",10)
    report=dict(test=name,status="FAIL",run=os.environ["WORKFLOW_RUN"])
    def wait(condition,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)
            if physics and physics[-1]["collision_episodes"]:raise AssertionError("Physical contact")
            if physics and physics[-1].get("product_environment_overlaps"):raise AssertionError("Product clips environment")
            if any(r["status"]=="FAILED" for r in results.values()):raise AssertionError(results)
            if condition():return
        raise RuntimeError("Timeout: "+str(states))
    try:
        wait(lambda:physics and physics[-1]["time"]>3 and all(
            states.get(r,{}).get("bootstrap_ready") and time.monotonic()-fresh.get(r,0)<1 for r in IDS),900)
        initial=physics[-1]
        task=dict(task_id=os.environ["WORKFLOW_RUN"],kind="pickup",product_id="PRODUCT_01",
            pickup_x=4.,pickup_y=-10.,delivery_x=10.,delivery_y=-10.)
        if name in ("D1","D2"):task.update(kind="navigate",pickup_x=10.,pickup_y=-10.)
        def send_block(action):
            environment_command.publish(String(data=json.dumps(dict(action=action,id="AISLE_TEST",x=6.,y=-10.,sx=.8,sy=1.2))))
        def synced(version):
            return len({e["robot_id"] for e in events if e["event"]=="MAP_SYNCHRONIZED" and e["version"]==version})==3
        if name=="D2":
            send_block("BLOCK_AISLE")
            wait(lambda:synced(1),60)
            assert physics[-1]["blockages"][0]["active"]
            send_block("UNBLOCK_AISLE")
            wait(lambda:synced(2),60)
            assert not physics[-1]["blockages"][0]["active"]
        pub.publish(String(data=json.dumps(task)))
        if name in ("D1","P2"):
            wait(lambda:"amr_01" in paths and math.dist(initial["poses"]["amr_01"][:2],physics[-1]["poses"]["amr_01"][:2])>.6 and
                 (name=="D1" or (physics[-1]["product"]["state"]=="ATTACHED" and physics[-1]["poses"]["amr_01"][0]>3.05)),limit)
            report["before_blockage"]=physics[-1]
            report["original_path"]=paths["amr_01"]["path"]
            send_block("BLOCK_AISLE")
            wait(lambda:any(e["event"]=="REPLAN_COMPLETED" for e in events),60)
            report["replan"]=next(e for e in events if e["event"]=="REPLAN_COMPLETED")
            assert report["replan"]["path"]!=report["original_path"]
            wait(lambda:bool(scan_hits),30)
        wait(lambda:task["task_id"] in results,limit)
        winner=results[task["task_id"]]["robot_id"]
        wait(lambda:states.get(winner,{}).get("state")=="IDLE",30)
        final=physics[-1]
        assert len(bids)==3 and len(decisions)==3
        assert all(d["winner"]==winner for d in decisions.values())
        if name in ("P1","P2","P3"):
            assert final["product"]["state"]=="DELIVERED" and final["product"]["carrier"] is None
            assert math.dist(final["product"]["pose"][:2],(10,-10))<.23
            carried=[p for p in physics if p["product"]["state"]=="ATTACHED"]
            assert carried and math.dist(carried[0]["product"]["pose"][:2],carried[-1]["product"]["pose"][:2])>3
            assert all(math.dist(p["poses"][winner][:2],p["product"]["pose"][:2])<.16 for p in carried)
            assert any(e["event"]=="DROP_VERIFIED" for e in events)
        else:
            assert math.dist(final["poses"][winner][:2],(10,-10))<.25
        if name=="D2":
            assert any(5.5<p["poses"][winner][0]<6.5 and abs(p["poses"][winner][1]+10)<.3 for p in physics)
        report.update(status="PASS",winner=winner,task=task,initial=initial,final=final,
            completion_time_sim_s=final["time"]-initial["time"],contacts=final["collision_episodes"],
            minimum_separation=final["minimum_separation"],bids=bids,decisions=decisions)
    except Exception as error:
        report["error"]=str(error);raise
    finally:
        report.update(physics=physics,events=events,states=states,results=results,paths=paths,path_history=path_history,environments=environments,lidar_blockage_hits=scan_hits)
        path=ROOT/"Webots/workflow_runs"/os.environ["WORKFLOW_RUN"]/"result.json"
        path.write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ("physics","events","states","paths")},indent=2),flush=True)
        n.destroy_node();rclpy.shutdown()
def main():
    parser=argparse.ArgumentParser();parser.add_argument("test");parser.add_argument("--probe",action="store_true");parser.add_argument("--limit",type=float,default=1500)
    args=parser.parse_args()
    if args.probe:return probe(args.test,args.limit)
    run=args.test+"_"+str(time.time_ns());out=ROOT/"Webots/workflow_runs"/run;out.mkdir(parents=True)
    env=dict(os.environ,ROS_DOMAIN_ID="31",WORKFLOW_RUN=run,LP_NUM_THREADS="2",OMP_NUM_THREADS="1")
    processes=[];logs=[];print("RUN="+run,flush=True)
    try:
        for label,command in [("world",["/usr/local/bin/webots","--batch","--minimize","--no-rendering","--mode=fast","--stdout","--stderr","--port=1240",str(world(args.test))]),
                              ("fleet",["ros2","launch","bel_amr_ros","webots_warehouse.launch.py"])]:
            log=(out/(label+".log")).open("w");logs.append(log)
            processes.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        child=subprocess.Popen(["python3",__file__,args.test,"--probe","--limit",str(args.limit)],env=env,start_new_session=True)
        processes.append(child);raise SystemExit(child.wait())
    finally:
        for p in reversed(processes):
            if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
        for p in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
        for log in logs:log.close()
if __name__=="__main__":main()
