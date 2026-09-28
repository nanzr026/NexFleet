import math, time
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
rclpy.init()
n=rclpy.create_node('carry_diagnostic')
data={}
def scan(m):
    rear=[r for i,r in enumerate(m.ranges) if math.isfinite(r) and abs(m.angle_min+i*m.angle_increment)>math.radians(140)]
    data['rear']=min(rear,default=float('inf'))
n.create_subscription(LaserScan,'/model/amr_02/scan',scan,10)
for rid in ['amr_02','pallet_box_mobile']:
    n.create_subscription(PoseStamped,f'/model/{rid}/pose',
        lambda m,k=rid: data.update({k:(m.pose.position.x,m.pose.position.y)}) if m.header.frame_id=='world_demo' else None,10)
n.create_subscription(Odometry,'/model/amr_02/odometry',lambda m:data.update(odom_velocity=m.twist.twist.linear.x),10)
pub=n.create_publisher(Twist,'/model/amr_02/cmd_vel',10)
start=time.monotonic()
try:
    while time.monotonic()-start<15:
        rclpy.spin_once(n,timeout_sec=0.1)
        cmd=Twist()
        if time.monotonic()-start>3 and data.get('rear',0)>0.8:
            cmd.linear.x=-0.08
        pub.publish(cmd)
        if int((time.monotonic()-start)*10)%10==0:
            print(dict(data,command=cmd.linear.x),flush=True)
finally:
    pub.publish(Twist())
    n.destroy_node()
    rclpy.shutdown()
