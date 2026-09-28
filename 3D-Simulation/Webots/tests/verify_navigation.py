"""Observe decentralized bidding and A* world motion on the Webots map."""
import json,math,time,uuid
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('webots_navigation_probe')
ids=['amr_01','amr_02','amr_03']
poses={};states={};bids={};claims={};results={};paths={};history=[];scans=set();odom=set()
for rid in ids:
    n.create_subscription(PoseStamped,f'/model/{rid}/pose',lambda m,k=rid:poses.update({k:(m.pose.position.x,m.pose.position.y)}),10)
    for suffix,dest in [('telemetry',states),('task_bid',bids),('task_claim',claims),('path_intention',paths)]:
        n.create_subscription(String,f'/{rid}/{suffix}',lambda m,k=rid,d=dest:d.update({k:json.loads(m.data)}),10)
    n.create_subscription(LaserScan,f'/model/{rid}/scan',lambda m,k=rid:scans.add(k),10)
    n.create_subscription(Odometry,f'/model/{rid}/odometry',lambda m,k=rid:odom.add(k),10)
n.create_subscription(String,'/task/result',lambda m:results.update({json.loads(m.data)['task_id']:json.loads(m.data)}),10)
pub=n.create_publisher(String,'/task/announcement',10)
def wait(condition,limit):
    end=time.monotonic()+limit;last=0
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if time.monotonic()-last>.5:
            last=time.monotonic();history.append({'time':time.time(),'poses':dict(poses),'states':dict(states)})
        if condition():return
    raise RuntimeError('Navigation timeout: '+str(states))
report={'status':'FAIL'}
try:
    wait(lambda:all(r in poses and r in scans and r in odom and states.get(r,{}).get('bootstrap_ready') for r in ids),360)
    initial=dict(poses);tid='WEBOTS_NAV_'+uuid.uuid4().hex[:8]
    task={'task_id':tid,'kind':'navigate','pickup_x':12.0,'pickup_y':-10.0}
    pub.publish(String(data=json.dumps(task)))
    wait(lambda:tid in results,900)
    result=results[tid]
    assert result['status']=='COMPLETED',result
    assert all(bids.get(r,{}).get('task_id')==tid for r in ids),bids
    eligible=[b for b in bids.values() if b['available']]
    winner=sorted(eligible,key=lambda b:(-b['bid'],b['robot_id']))[0]['robot_id']
    assert result['robot_id']==winner and claims[winner]['task_id']==tid
    assert math.dist(initial[winner],poses[winner])>2.5
    assert math.dist(poses[winner],(12,-10))<.3
    assert paths[winner]['task_id']==tid
    report={'status':'PASS','task':task,'result':result,'winner':winner,'initial_poses':initial,'final_poses':dict(poses),'bids':bids,'claims':claims,'path':paths[winner]}
finally:
    report['history']=history
    (Path(__file__).resolve().parents[1]/'logs/navigation_result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2),flush=True)
    n.destroy_node();rclpy.shutdown()
