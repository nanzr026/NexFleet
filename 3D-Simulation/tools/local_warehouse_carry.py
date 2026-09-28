"""Short measured carry test at PRODUCT_01; fleet controllers must be stopped."""
import json, math, time, subprocess
from pathlib import Path
import rclpy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from verify_demo import Probe
rclpy.init()
n=Probe()
vel=n.create_publisher(Twist,'/model/amr_02/cmd_vel',10)
attach=n.create_publisher(Empty,'/amr_02/product_01/attach',10)
def drive(seconds,speed):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        vel.publish(Twist(linear=__import__('geometry_msgs.msg',fromlist=['Vector3']).Vector3(x=speed)))
        rclpy.spin_once(n,timeout_sec=0.05)
    vel.publish(Twist())
def pose(name,x,y,z,yaw):
    req=f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
    p=subprocess.run(['gz','service','-s','/world/world_demo/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True,timeout=15)
    assert 'data: true' in p.stdout,p.stdout+p.stderr
try:
    n.wait(lambda:'amr_02' in n.poses and 'pallet_box_mobile' in n.poses,30,'poses')
    n.detach.publish(Empty()); drive(2,0.0)
    pose('pallet_box_mobile',4.4161,14.6952,0.17,0.0)
    pose('amr_02',5.60,14.6952,0.02,math.pi)
    drive(3,0.0)
    before=dict(n.poses)
    drive(5,-0.08)
    free=dict(n.poses)
    print('FREE',before,free,flush=True)
    pose('amr_02',5.60,14.6952,0.02,math.pi)
    drive(3,0.0)
    n.attachment=None
    n.wait(lambda:n.attachment=='attached',15,'attach',lambda:attach.publish(Empty()))
    drive(1,0.0)
    baseline=dict(n.poses)
    drive(12,-0.08)
    final=dict(n.poses)
    delta={k:math.dist(baseline[k],final[k]) for k in ('amr_02','pallet_box_mobile')}
    print('CARRY',delta,'BASELINE',baseline,'FINAL',final,flush=True)
    assert min(delta.values())>0.35,delta
    assert abs(delta['amr_02']-delta['pallet_box_mobile'])<0.1,delta
    drive(5,0.0)
    n.attachment=None
    n.wait(lambda:n.attachment=='detached',15,'detach',lambda:n.detach.publish(Empty()))
    drive(2,0.0)
    baseline=dict(n.poses)
    drive(8,-0.08)
    delta={k:math.dist(baseline[k],n.poses[k]) for k in ('amr_02','pallet_box_mobile')}
    print('RELEASE',delta,flush=True)
    assert delta['amr_02']>0.25 and delta['pallet_box_mobile']<0.15,delta
finally:
    vel.publish(Twist())
    (Path(__file__).resolve().parents[1]/'demo_logs/local_warehouse_carry.json').write_text(json.dumps(n.history,indent=2))
    n.destroy_node()
    rclpy.shutdown()
