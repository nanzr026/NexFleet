"""AMR03 approaches a real rack; Webots LiDAR must stop translation."""
import json,time,math
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('webots_safety_probe')
data={}
n.create_subscription(PoseStamped,'/model/amr_03/pose',lambda m:data.update(pose=(m.pose.position.x,m.pose.position.y)),10)
n.create_subscription(String,'/amr_03/safety',lambda m:data.update(safety=m.data),10)
pub=n.create_publisher(Twist,'/model/amr_03/cmd_vel',1)
report={'status':'FAIL'}
try:
    start=time.monotonic();initial=None;blocked=None
    while time.monotonic()-start<180:
        rclpy.spin_once(n,timeout_sec=.03)
        if initial is None and 'pose' in data:initial=data['pose']
        msg=Twist();msg.linear.x=.25;pub.publish(msg)
        if data.get('safety')=='LIDAR_STOP':
            blocked=data['pose'];break
    assert blocked is not None,data
    start=time.monotonic()
    while time.monotonic()-start<3:
        pub.publish(Twist())
        rclpy.spin_once(n,timeout_sec=.03)
    drift=math.dist(blocked,data['pose'])
    assert drift<.08,drift
    assert data['pose'][0]>13.5,data
    report={'status':'PASS','initial':initial,'blocked':blocked,'stopped':data['pose'],'stop_drift_m':drift}
finally:
    pub.publish(Twist())
    (Path(__file__).resolve().parents[1]/'logs/lidar_stop_result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
    n.destroy_node();rclpy.shutdown()
