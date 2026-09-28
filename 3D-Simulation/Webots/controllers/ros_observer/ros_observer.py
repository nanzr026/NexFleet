"""Validation observer and simulation clock only; no fleet command publishers."""
import json,math
from pathlib import Path
from controller import Supervisor
import rclpy
from rosgraph_msgs.msg import Clock
s=Supervisor();step=int(s.getBasicTimeStep())
rclpy.init();node=rclpy.create_node('webots_clock_observer')
clock=node.create_publisher(Clock,'/clock',10)
robots={name:s.getFromDef(name) for name in ('AMR01','AMR02','AMR03')}
out=Path(__file__).resolve().parents[2]/'logs'
last=-1.0;saved=False
view=None
children=s.getRoot().getField('children')
for i in range(children.getCount()):
    candidate=children.getMFNode(i)
    if candidate.getTypeName()=='Viewpoint':view=candidate
original_view=None
closeup_saved=False
with (out/'ros_physical_history.jsonl').open('w') as f:
    while s.step(step)!=-1:
        now=s.getTime()
        clock.publish(Clock(clock=rclpy.time.Time(seconds=now).to_msg()))
        if now-last>.1:
            last=now
            states={name:{'position':n.getPosition(),'yaw':math.atan2(n.getOrientation()[3],n.getOrientation()[0])} for name,n in robots.items()}
            f.write(json.dumps({'time':now,'robots':states})+chr(10));f.flush()
        if now>1 and not saved:
            s.exportImage(str(out/'warehouse_ros.png'),95);saved=True
            if view:
                original_view=(view.getField('position').getSFVec3f(),view.getField('orientation').getSFRotation())
                target=robots['AMR01'].getPosition()
                camera=[target[0]+2.5,target[1]-3.0,target[2]+2.2]
                dx,dy,dz=[a-b for a,b in zip([target[0],target[1],target[2]+.3],camera)]
                yaw=math.atan2(dy,dx);pitch=math.atan2(-dz,math.hypot(dx,dy))
                q=[-math.sin(yaw/2)*math.sin(pitch/2),math.cos(yaw/2)*math.sin(pitch/2),math.sin(yaw/2)*math.cos(pitch/2),math.cos(yaw/2)*math.cos(pitch/2)]
                angle=2*math.acos(q[3]);axis=[a/math.sin(angle/2) for a in q[:3]]
                view.getField('position').setSFVec3f(camera)
                view.getField('orientation').setSFRotation(axis+[angle])
        if now>1.3 and original_view and not closeup_saved:
            s.exportImage(str(out/'amr_detail.png'),95);closeup_saved=True
            view.getField('position').setSFVec3f(original_view[0])
            view.getField('orientation').setSFRotation(original_view[1])
node.destroy_node()
if rclpy.ok():rclpy.shutdown()
