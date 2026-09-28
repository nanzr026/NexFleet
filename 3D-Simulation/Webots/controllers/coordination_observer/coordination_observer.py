"""Read-only physics evidence. No pose setters, task or motor publishers."""
import json,math
from pathlib import Path
from controller import Supervisor
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
s=Supervisor();step=int(s.getBasicTimeStep())
rclpy.init();n=rclpy.create_node('coordination_physics_observer')
pub=n.create_publisher(String,'/validation/physics',10)
clock=n.create_publisher(Clock,'/clock',10)
robots={f'amr_{i:02}':s.getFromDef(f'AMR{i:02}') for i in range(1,4)}
out=Path(__file__).resolve().parents[2]/'logs'
last=0;minimum=100.;episodes=0;touching=False
with (out/'coordination_physics.jsonl').open('w') as log:
    while s.step(step)!=-1:
        now=s.getTime()
        clock.publish(Clock(clock=rclpy.time.Time(seconds=now).to_msg()))
        poses={rid:robot.getPosition() for rid,robot in robots.items()}
        pairs=[math.dist(a[:2],b[:2]) for i,a in enumerate(poses.values()) for b in list(poses.values())[:i]]
        minimum=min(minimum,*pairs)
        contacts={rid:[list(c.point) for c in robot.getContactPoints(True) if c.point[2]>.04]
                  for rid,robot in robots.items()}
        active=any(contacts.values())
        if active and not touching:episodes+=1
        touching=active
        if now-last>=.096:
            last=now
            record=dict(time=now,poses=poses,minimum_separation=minimum,
                        collision_episodes=episodes,nonfloor_contacts=contacts)
            log.write(json.dumps(record)+chr(10));log.flush()
            pub.publish(String(data=json.dumps(record)))
        rclpy.spin_once(n,timeout_sec=0)
n.destroy_node()
if rclpy.ok():rclpy.shutdown()
