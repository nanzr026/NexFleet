"""Per-robot Webots/RCLPy adapter. No fleet-level decisions live here."""
import math,time
from controller import Robot
import rclpy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile,ReliabilityPolicy
robot=Robot()
step=int(robot.getBasicTimeStep())
rid='amr_'+robot.getName()[-2:]
rclpy.init()
node=rclpy.create_node(rid+'_webots_interface')
left,right=[robot.getDevice(k+'_wheel_motor') for k in ('left','right')]
encoders=[robot.getDevice(k+'_wheel_sensor') for k in ('left','right')]
for motor in (left,right):
    motor.setPosition(float('inf'));motor.setVelocity(0)
for sensor in encoders:sensor.enable(step)
gps=robot.getDevice('gps');gps.enable(step)
imu=robot.getDevice('imu');imu.enable(step)
lidar=robot.getDevice('lidar');lidar.enable(step*8)
pose_pub=node.create_publisher(PoseStamped,f'/model/{rid}/pose',10)
odom_pub=node.create_publisher(Odometry,f'/model/{rid}/odometry',10)
scan_pub=node.create_publisher(LaserScan,f'/model/{rid}/scan',10)
safety_pub=node.create_publisher(String,f'/{rid}/safety',10)
command=[0.0,0.0,-float('inf')]
def receive(msg):
    if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
        command[:]=[max(-.45,min(.45,msg.linear.x)),max(-.7,min(.7,msg.angular.z)),time.monotonic()]
node.create_subscription(Twist,f'/model/{rid}/cmd_vel',receive,1)
last_enc=None
ox=oy=heading=0.0
last_scan=-1
last_pub=-1
try:
    while robot.step(step)!=-1 and rclpy.ok():
        rclpy.spin_once(node,timeout_sec=0)
        now=robot.getTime()
        stamp=rclpy.time.Time(seconds=now).to_msg()
        raw=list(lidar.getRangeImage())
        # Webots scans left-to-right; ROS LaserScan increases counter-clockwise.
        ranges=list(reversed(raw))
        increment=2*math.pi/(len(ranges)-1)
        front=min((r for i,r in enumerate(ranges) if math.isfinite(r)
                   and abs(-math.pi+i*increment)<math.radians(40)),default=float('inf'))
        rear=min((r for i,r in enumerate(ranges) if math.isfinite(r)
                  and abs(-math.pi+i*increment)>math.radians(140)),default=float('inf'))
        v,w=command[:2]
        reason='CLEAR'
        if time.monotonic()-command[2]>.75:
            v=w=0.0;reason='COMMAND_TIMEOUT'
        if (v>0 and front<.70) or (v<0 and rear<.70):
            v=0.0;reason='LIDAR_STOP'
        if abs(w)>0 and min((r for r in ranges if math.isfinite(r)),default=99)<.57:
            w=0.0;reason='LIDAR_TURN_STOP'
        left.setVelocity((v-.76*w/2)/.14)
        right.setVelocity((v+.76*w/2)/.14)
        encoder=[sensor.getValue() for sensor in encoders]
        velocity=angular=0.0
        if last_enc is not None:
            dl,dr=[.14*(a-b) for a,b in zip(encoder,last_enc)]
            distance=(dl+dr)/2;delta=(dr-dl)/.76
            ox+=distance*math.cos(heading+delta/2)
            oy+=distance*math.sin(heading+delta/2)
            heading=math.atan2(math.sin(heading+delta),math.cos(heading+delta))
            velocity=distance/(step/1000);angular=delta/(step/1000)
        last_enc=encoder
        if now-last_pub>=.048:
            last_pub=now
            pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='webots_world'
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=gps.getValues()
            roll,pitch,yaw=imu.getRollPitchYaw()
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
            pose_pub.publish(pose)
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id=rid+'/odom';odom.child_frame_id=rid+'/base_link'
            odom.pose.pose.position.x=ox;odom.pose.pose.position.y=oy
            odom.pose.pose.orientation.z=math.sin(heading/2);odom.pose.pose.orientation.w=math.cos(heading/2)
            odom.twist.twist.linear.x=velocity;odom.twist.twist.angular.z=angular
            odom_pub.publish(odom)
            safety_pub.publish(String(data=reason))
        if now-last_scan>=step*8/1000:
            last_scan=now
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id=rid+'/lidar'
            scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=increment
            scan.range_min=lidar.getMinRange();scan.range_max=lidar.getMaxRange()
            scan.scan_time=step*8/1000;scan.ranges=ranges
            scan_pub.publish(scan)
finally:
    left.setVelocity(0);right.setVelocity(0)
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
