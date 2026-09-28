"""Small Webots adapters reusing the existing A* controller and decentralized bidder."""
import json,math,time
import rclpy
from std_msgs.msg import String
from .robot_controller import AMRController
from .p2p_agent import P2PAgent
from .webots_coordination import CoordinationSession
from .webots_map import WebotsWarehouseMap,load_layout
SPAWNS={r['ros_id']:(r['x'],r['y']) for r in load_layout()['robots']}
class WebotsNavigation(AMRController):
    WORLD_FRAME='webots_world'
    LOCALIZATION_SOURCE='webots_gps_imu'
    MAP_CLASS=WebotsWarehouseMap
    ROBOT_SPAWNS=SPAWNS
    def __init__(self):
        self.coordination=None
        super().__init__()
        self.coordination_pub=self.create_publisher(String,f'/{self.robot_id}/reservation',10)
        self.coordination_events=self.create_publisher(String,f'/{self.robot_id}/coordination_event',100)
        for peer in SPAWNS:
            if peer!=self.robot_id:
                self.create_subscription(String,f'/{peer}/reservation',
                    lambda msg,rid=peer:self.reservation_callback(rid,msg),10)
        self.create_timer(.1,self.publish_reservation)
        self.bootstrap_ready=True
        self.charging_stations=dict(SPAWNS)
        self.get_logger().info('Webots navigation ready; physical pallet transport is not enabled')
    def assigned_task_callback(self,msg):
        try:
            task=json.loads(msg.data)
        except (ValueError,TypeError):
            return
        if task.get('kind')!='navigate':
            self.result_pub.publish(String(data=json.dumps({
                'task_id':task.get('task_id'),'robot_id':self.robot_id,
                'status':'FAILED','reason':'Webots load transport is not enabled in this milestone'})))
            return
        spec=task.get('coordination')
        if spec and self.robot_state=='IDLE':
            peers=self.warehouse_map.peer_poses
            self.warehouse_map.peer_poses={}
            try:
                route=self.astar.plan((self.x,self.y),(task['pickup_x'],task['pickup_y']))
                if not route:raise ValueError('No independent A* route')
                session=CoordinationSession(self,task,route)
                super().assigned_task_callback(msg)
                if self.current_task_id==task['task_id']:
                    self.coordination=session
            except (ValueError,KeyError,TypeError) as error:
                self.get_logger().error('Coordination rejected: '+str(error))
            finally:
                self.warehouse_map.peer_poses=peers
        else:
            super().assigned_task_callback(msg)

    def reservation_callback(self,rid,msg):
        if self.coordination:
            try:self.coordination.receive(rid,json.loads(msg.data))
            except (ValueError,KeyError,TypeError):pass

    def publish_reservation(self):
        if self.coordination:
            self.coordination_pub.publish(String(data=json.dumps(self.coordination.state())))
        else:
            self.coordination_pub.publish(String(data=json.dumps(
                dict(robot_id=self.robot_id,capability='dds_reservation_v1'))))

    def trigger_reroute(self,hit):
        if self.coordination and not self.coordination.done:
            self.stop_robot()
            return
        super().trigger_reroute(hit)

    def move_to_goal(self):
        c=self.coordination
        if c and not c.done:
            c.update()
            if c.done:
                self.finish_task()
                return
            if not c.allow:
                self.stop_robot()
                return
            self.goal_x,self.goal_y=c.nodes[c.target[c.index]]
            self.waypoints=[(self.goal_x,self.goal_y)]
            self.current_waypoint_index=0
            tolerance=self.goal_tolerance
            self.goal_tolerance=.10
            try:super().move_to_goal()
            finally:self.goal_tolerance=tolerance
        else:
            super().move_to_goal()

class WebotsPeer(P2PAgent):
    WORLD_FRAME='webots_world'
    def __init__(self):
        super().__init__()
        self.spawn_positions=dict(SPAWNS)
        self.spawn_x,self.spawn_y=SPAWNS[self.robot_id]
        self.x,self.y=self.spawn_x,self.spawn_y
        self.pickup_capable=False
def run(cls,args=None):
    rclpy.init(args=args)
    node=cls()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if isinstance(node,AMRController) and rclpy.ok():node.stop_robot()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
def navigation_main(args=None):run(WebotsNavigation,args)
def peer_main(args=None):run(WebotsPeer,args)
