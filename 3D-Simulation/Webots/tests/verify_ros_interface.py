"""Send real DDS commands and verify GPS motion, encoder odometry, LiDAR, watchdog."""
import json,math,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('webots_interface_probe')
ids=['amr_01','amr_02','amr_03']
poses={};odom={};scans={};safety={};pubs={}
def pose_cb(m,k):
    q=m.pose.orientation
    poses[k]=(m.pose.position.x,m.pose.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z),m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
for rid in ids:
    n.create_subscription(PoseStamped,f'/model/{rid}/pose',lambda m,k=rid:pose_cb(m,k),10)
    n.create_subscription(Odometry,f'/model/{rid}/odometry',lambda m,k=rid:odom.update({k:m}),10)
    n.create_subscription(LaserScan,f'/model/{rid}/scan',lambda m,k=rid:scans.update({k:m}),10)
    n.create_subscription(String,f'/{rid}/safety',lambda m,k=rid:safety.update({k:m.data}),10)
    pubs[rid]=n.create_publisher(Twist,f'/model/{rid}/cmd_vel',10)
def spin_until(condition,timeout=360,command=None):
    deadline=time.monotonic()+timeout
    last=0
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.02)
        if command and time.monotonic()-last>.04:
            last=time.monotonic()
            v,w=command
            for p in pubs.values():
                msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);p.publish(msg)
        if condition():return
    raise RuntimeError('Timed out waiting for fresh interface data or physical motion')
def phase(seconds,v,w):
    start=min(poses[r][3] for r in ids)
    spin_until(lambda:all(poses[r][3]-start>=seconds for r in ids),240,(v,w))
    return dict(poses)
report={'status':'FAIL'}
try:
    spin_until(lambda:all(r in poses and r in odom and r in scans for r in ids))
    initial=dict(poses)
    forward=phase(4,.25,0)
    left=phase(2,0,.5)
    right=phase(2,0,-.5)
    # No more commands: the per-robot watchdog must stop all wheels.
    time.sleep(1)
    start=dict(poses)
    spin_until(lambda:all(safety.get(r)=='COMMAND_TIMEOUT' for r in ids),30)
    stopped=phase(1,0,0)
    end=phase(1,0,0)
    results={}
    for rid in ids:
        a,b=initial[rid],forward[rid]
        distance=(b[0]-a[0])*math.cos(a[2])+(b[1]-a[1])*math.sin(a[2])
        delta=lambda x:math.atan2(math.sin(x),math.cos(x))
        turn_left=delta(left[rid][2]-b[2]);turn_right=delta(right[rid][2]-left[rid][2])
        drift=math.dist(stopped[rid][:2],end[rid][:2])
        finite=sum(math.isfinite(x) for x in scans[rid].ranges)
        assert distance>.6 and turn_left>.5 and turn_right<-.5 and drift<.05,(rid,distance,turn_left,turn_right,drift)
        assert len(scans[rid].ranges)==360 and finite>10,(rid,finite)
        assert math.hypot(odom[rid].pose.pose.position.x,odom[rid].pose.pose.position.y)>.6
        results[rid]={'forward_m':distance,'left_rad':turn_left,'right_rad':turn_right,'stop_drift_m':drift,'scan_samples':len(scans[rid].ranges),'finite_returns':finite}
    report={'status':'PASS','robots':results,'command_watchdog':'PASS'}
finally:
    for p in pubs.values():p.publish(Twist())
    (Path(__file__).resolve().parents[1]/'logs/ros_interface_result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    n.destroy_node();rclpy.shutdown()
