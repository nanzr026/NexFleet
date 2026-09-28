"""Product/environment adapter. Only product poses are written, never AMR poses."""
import json,math,os,sys
from pathlib import Path
from controller import Supervisor
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"ROS 2/src/bel_amr_ros"))
from bel_amr_ros.warehouse_workflow import pickup_rejection,drop_rejection,OFFSET
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy
from bel_amr_ros.warehouse_workflow import rectangles_overlap
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
s=Supervisor();step=int(s.getBasicTimeStep())
rclpy.init();n=rclpy.create_node("warehouse_environment")
robots={f"amr_{i:02}":s.getFromDef(f"AMR{i:02}") for i in range(1,4)}
product=s.getFromDef("PRODUCT_01")
status_pub=n.create_publisher(String,"/warehouse/product_state",10)
event_pub=n.create_publisher(String,"/warehouse/events",100)
physics_pub=n.create_publisher(String,"/validation/physics",10)
clock=n.create_publisher(Clock,"/clock",10)
state="AVAILABLE";carrier=None;task_id=None;operation=None;delivery=None
out=ROOT/"Webots/workflow_runs"/os.environ["WORKFLOW_RUN"];out.mkdir(parents=True,exist_ok=True)
def robot_pose(rid):
    r=robots[rid];p=r.getPosition();o=r.getOrientation()
    return p[0],p[1],math.atan2(o[3],o[0])
def event(label,**data):
    value=dict(event=label,time=s.getTime(),product_id="PRODUCT_01",carrier=carrier,task_id=task_id,**data)
    event_pub.publish(String(data=json.dumps(value)))
    with (out/"environment_events.jsonl").open("a") as f:f.write(json.dumps(value)+chr(10))
def command(msg):
    global state,carrier,task_id,operation,delivery
    try:
        value=json.loads(msg.data);rid=value["robot_id"];action=value["action"]
        if rid not in robots or value.get("product_id")!="PRODUCT_01":return
        pose=robot_pose(rid);speed=math.hypot(*robots[rid].getVelocity()[:2])
        if operation:return
        if action=="ATTACH":
            reason=pickup_rejection(pose,product.getPosition(),speed,carrier,state)
            if reason:event("PICKUP_REJECTED",reason=reason,requester=rid);return
            delivery=list(map(float,value["delivery"]))
            if len(delivery)!=2 or not all(math.isfinite(v) for v in delivery):return
            carrier=rid;task_id=value["task_id"]
            operation=dict(action=action,start=s.getTime(),position=product.getPosition(),robot=pose)
            state="PICKING";event("PICKUP_STARTED")
        elif action=="DETACH":
            reason=drop_rejection(pose,speed,carrier,rid,delivery or (1e9,1e9))
            if reason:event("DROP_REJECTED",reason=reason,requester=rid);return
            if value["task_id"]!=task_id:return
            operation=dict(action=action,start=s.getTime(),position=product.getPosition(),robot=pose)
            state="DROPPING";event("DROP_STARTED")
    except (ValueError,KeyError,TypeError):event("INVALID_PRODUCT_COMMAND")
n.create_subscription(String,"/warehouse/product_command",command,10)
blocks={};version=0;pending_environment=None
environment_pub=n.create_publisher(String,"/warehouse/environment",
    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
layout=json.loads((ROOT/"Webots/config/warehouse_layout.json").read_text())
def environment_command(msg):
    global version,pending_environment
    try:
        value=json.loads(msg.data);ident=value["id"];action=value["action"]
        if pending_environment or not ident.replace("_","").isalnum():return
        if action=="BLOCK_AISLE":
            if ident in blocks and blocks[ident]["active"]:return
            x,y,sx,sy=[float(value[k]) for k in ("x","y","sx","sy")]
            if not all(math.isfinite(v) for v in (x,y,sx,sy)) or not .2<=sx<=3 or not .2<=sy<=3:return
            if not -18<x<18 or not -14<y<14:return
            if any(abs(r.getPosition()[0]-x)<sx/2+1.4 and abs(r.getPosition()[1]-y)<sy/2+1.4 for r in robots.values()):
                event("BLOCKAGE_REJECTED",reason="Robot clearance");return
            source=f'DEF BLOCK_{ident} Solid {{ translation {x} {y} .6 name "blockage_{ident}" children [ Shape {{ appearance PBRAppearance {{ baseColor 1 .35 .03 }} geometry Box {{ size {sx} {sy} 1.2 }} }} ] boundingObject Box {{ size {sx} {sy} 1.2 }} }}'
            s.getRoot().getField("children").importMFNodeFromString(-1,source)
            if s.getFromDef("BLOCK_"+ident) is None:return
            blocks[ident]=dict(id=ident,x=x,y=y,sx=sx,sy=sy,active=True,inserted= s.getTime(),removed=None)
        elif action=="UNBLOCK_AISLE":
            if ident not in blocks or not blocks[ident]["active"]:return
            obj=s.getFromDef("BLOCK_"+ident)
            if obj:obj.remove()
            blocks[ident]["active"]=False;blocks[ident]["removed"]=s.getTime()
        else:return
        pending_environment=dict(id=ident,action=action,time=s.getTime())
    except (ValueError,KeyError,TypeError):event("INVALID_BLOCKAGE_COMMAND")
n.create_subscription(String,"/warehouse/environment_command",environment_command,10)
last_environment=-1
last=0;minimum=100.;episodes=0;touching=False
with (out/"physics.jsonl").open("w") as log:
    while s.step(step)!=-1:
        now=s.getTime();clock.publish(Clock(clock=rclpy.time.Time(seconds=now).to_msg()))
        rclpy.spin_once(n,timeout_sec=0)
        if operation:
            t=min(1.,(now-operation["start"])/2.)
            x,y,yaw=operation["robot"]
            if operation["action"]=="ATTACH":
                px,py,pz=operation["position"]
                q=min(1.,t*2)
                position=[px,py,pz+(.80-pz)*q] if t<=.5 else [px+(x-px)*(2*t-1),py+(y-py)*(2*t-1),.80]
            else:
                q=min(1.,t*2)
                position=[x+OFFSET*math.cos(yaw)*q,y+OFFSET*math.sin(yaw)*q,.8] if t<=.5 else [x+OFFSET*math.cos(yaw),y+OFFSET*math.sin(yaw),.8*(2-2*t)]
            product.getField("translation").setSFVec3f(position)
            product.getField("rotation").setSFRotation([0,0,1,yaw])
            if t>=1:
                if operation["action"]=="ATTACH":state="ATTACHED";event("ATTACHMENT_VERIFIED")
                else:
                    carrier=None;state="DELIVERED";event("DETACHMENT_VERIFIED")
                operation=None
        elif carrier and state=="ATTACHED":
            x,y,yaw=robot_pose(carrier)
            product.getField("translation").setSFVec3f([x,y,.80])
            product.getField("rotation").setSFRotation([0,0,1,yaw])
        if pending_environment and now>pending_environment["time"]:
            ident=pending_environment["id"]
            present=s.getFromDef("BLOCK_"+ident) is not None
            if present==blocks[ident]["active"]:
                version+=1
                event("BLOCKAGE_CONFIRMED",id=ident,action=pending_environment["action"],command_time=pending_environment["time"],active=present,version=version)
                pending_environment=None
        if now-last_environment>=.5:
            last_environment=now
            environment_pub.publish(String(data=json.dumps(dict(confirmed=True,version=version,time=now,blockages=list(blocks.values())))))
        poses={rid:r.getPosition() for rid,r in robots.items()}
        minimum=min(minimum,*[math.dist(a[:2],b[:2]) for i,a in enumerate(poses.values()) for b in list(poses.values())[:i]])
        contacts={rid:[list(c.point) for c in r.getContactPoints(True) if c.point[2]>.04] for rid,r in robots.items()}
        contacts["PRODUCT_01"]=[list(c.point) for c in product.getContactPoints(True) if c.point[2]>.04]
        # Static/kinematic pairs may not create force contacts. Independently audit footprint overlap.
        overlaps=[]
        pp=product.getPosition();po=product.getOrientation();pyaw=math.atan2(po[3],po[0])
        envelope=(pp[0],pp[1],.6,.4,pyaw)
        for obstacle in layout["racks"]+layout["docks"]+layout["obstacles"]:
            if rectangles_overlap(envelope,(obstacle["x"],obstacle["y"],obstacle["size"][0]/2,obstacle["size"][1]/2,0)):
                overlaps.append(obstacle["name"])
        for b in blocks.values():
            if b["active"] and rectangles_overlap(envelope,(b["x"],b["y"],b["sx"]/2,b["sy"]/2,0)):overlaps.append(b["id"])
        active=any(contacts.values())
        if active and not touching:episodes+=1
        touching=active
        if now-last>=.096:
            last=now
            value=dict(product_id="PRODUCT_01",pose=product.getPosition(),state=state,carrier=carrier,task_id=task_id,delivery=delivery,time=now,yaw=pyaw)
            status_pub.publish(String(data=json.dumps(value)))
            record=dict(time=now,poses=poses,product=value,minimum_separation=minimum,collision_episodes=episodes,nonfloor_contacts=contacts,product_environment_overlaps=overlaps,blockages=list(blocks.values()),environment_version=version)
            log.write(json.dumps(record)+chr(10));log.flush();physics_pub.publish(String(data=json.dumps(record)))
n.destroy_node()
if rclpy.ok():rclpy.shutdown()
