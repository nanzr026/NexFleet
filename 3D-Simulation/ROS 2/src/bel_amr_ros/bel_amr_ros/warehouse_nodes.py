"""Optional physical warehouse adapter; accepted navigation and coordination remain intact."""
import json,math,time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .webots_nodes import WebotsNavigation,WebotsPeer,run
from .robot_controller import AMRController
from .warehouse_workflow import OFFSET,wrap,released_at
from .webots_map import WebotsWarehouseMap
from .astar_planner import AStarPlanner
from .warehouse_environment import EnvironmentMap
from rclpy.qos import QoSProfile,DurabilityPolicy

class WarehouseNavigation(WebotsNavigation):
    MAP_CLASS=EnvironmentMap
    def __init__(self):
        self.workflow=None;self.product=None;self.product_received=0.;self.command_sent=0.
        self.phase_started=0.;self.settled=None;self.loaded_start=None;self.environment=None
        super().__init__()
        self.pickup_capable=True
        self.workflow_pub=self.create_publisher(String,"/warehouse/task_events",100)
        self.product_command=self.create_publisher(String,"/warehouse/product_command",10)
        self.create_subscription(String,"/warehouse/product_state",self.product_state,10)
        self.create_subscription(String,"/warehouse/environment",self.environment_state,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def emit(self,label,**data):
        self.workflow_pub.publish(String(data=json.dumps(dict(event=label,robot_id=self.robot_id,
            task_id=self.current_task_id,time=getattr(self,"pose_sim_time",0.),**data))))
    def transition(self,state,reason):
        previous=getattr(self,"robot_state",None)
        super().transition(state,reason)
        if previous!=state and hasattr(self,"workflow_pub"):
            self.phase_started=self.pose_sim_time
            self.emit(state,reason=reason)
    def product_state(self,msg):
        self.product=json.loads(msg.data);self.product_received=time.monotonic()
        if self.product["carrier"]==self.robot_id and self.product["state"]=="ATTACHED":
            self.attachment_state="attached"
        elif self.product["carrier"] is None:self.attachment_state="detached"
    def assigned_task_callback(self,msg):
        try:task=json.loads(msg.data)
        except (ValueError,TypeError):return
        if task.get("kind")=="navigate":return super().assigned_task_callback(msg)
        if self.robot_state!="IDLE" or self.current_task_id:return
        try:
            pickup=[float(task[k]) for k in ("pickup_x","pickup_y")]
            delivery=[float(task[k]) for k in ("delivery_x","delivery_y")]
            if not all(math.isfinite(x) for x in pickup+delivery):return
            if not self.product or self.product["state"]!="AVAILABLE" or math.dist(pickup,self.product["pose"][:2])>.2:return
            if time.monotonic()-self.pose_time>2 or time.monotonic()-self.product_received>2:return
            if self.battery_level<=20:return
        except (KeyError,TypeError,ValueError):return
        self.workflow=task
        self.current_task_id=task["task_id"];self.task_kind="pickup"
        self.task_started=time.monotonic();self.task_start_battery=self.battery_level
        self.distance_traveled=0.;self.reroute_count=0;self.obstacle_count=0
        self.carry_verified=False;self.coordination=None
        self.emit("ASSIGNED",product_id="PRODUCT_01",pickup=pickup,delivery=delivery)
        # Defined west-side dock, facing east; product and approach validated in the map.
        goal=(pickup[0]-OFFSET,pickup[1])
        if not self.set_route(goal,"MOVING_TO_PICKUP"):self.fail_task("No pickup approach route")
    def control_loop(self):
        if not self.workflow:return super().control_loop()
        now=time.monotonic()
        if now-self.pose_time>2 or now-self.scan_time>2 or now-self.product_received>2:
            self.stop_robot();return
        if self.robot_state=="DOCKING":
            self.align_and_act("PICKING");return
        if self.robot_state=="DELIVERY_DOCKING":
            self.align_and_act("DROPPING");return
        if self.robot_state in ("PICKING","DROPPING","VERIFYING_RELEASE"):
            self.product_step();return
        if self.robot_state=="MOVING_TO_DELIVERY":
            if self.product["state"]!="ATTACHED" or self.product["carrier"]!=self.robot_id:
                self.fail_task("Product attachment lost",holding=True);return
            if math.dist(self.product["pose"][:2],(self.x,self.y))>.15:
                self.fail_task("Product follower geometry failed",holding=True);return
            if self.loaded_start and math.dist(self.loaded_start,(self.x,self.y))>.5:self.carry_verified=True
            self.move_to_goal();return
        super().control_loop()
    def align_and_act(self,next_state):
        error=wrap(-self.yaw)
        if abs(error)>.06:
            cmd=Twist();cmd.angular.z=max(-.25,min(.25,error));self.cmd_vel_pub.publish(cmd)
            self.settled=None;return
        self.stop_robot()
        if self.settled is None:self.settled=self.pose_sim_time
        if self.pose_sim_time-self.settled>.5:
            self.transition(next_state,"Stopped and aligned at station")
            self.command_sent=0.;self.settled=None
    def product_step(self):
        self.stop_robot()
        state=self.product["state"]
        if self.robot_state=="PICKING" and state=="ATTACHED" and self.product["carrier"]==self.robot_id:
            self.loaded_start=(self.x,self.y);self.emit("CARRYING",product_pose=self.product["pose"])
            self.warehouse_map=EnvironmentMap(robot_inflation=.85)
            if self.environment:self.warehouse_map.apply(self.environment)
            self.astar=AStarPlanner(self.warehouse_map)
            goal=(self.workflow["delivery_x"]-OFFSET,self.workflow["delivery_y"])
            if not self.set_route(goal,"MOVING_TO_DELIVERY"):self.fail_task("No loaded delivery route",holding=True)
            return
        if self.robot_state=="DROPPING" and released_at(self.product["pose"],
            (self.workflow["delivery_x"],self.workflow["delivery_y"]),self.product["carrier"],state):
            self.release_position=self.product["pose"][:2]
            self.transition("VERIFYING_RELEASE","Detached; verify stationary product");return
        if self.robot_state=="VERIFYING_RELEASE":
            if math.dist(self.release_position,self.product["pose"][:2])>.03:
                self.fail_task("Product drift after drop");return
            if self.pose_sim_time-self.phase_started>2:
                self.emit("DROP_VERIFIED",product_pose=self.product["pose"])
                self.workflow=None;self.finish_task()
            return
        if self.pose_sim_time-self.phase_started>12:
            self.fail_task("Pickup/drop acknowledgement timeout",holding=True);return
        if self.pose_sim_time-self.command_sent>1:
            action="ATTACH" if self.robot_state=="PICKING" else "DETACH"
            self.product_command.publish(String(data=json.dumps(dict(action=action,
                robot_id=self.robot_id,product_id="PRODUCT_01",task_id=self.current_task_id,
                delivery=[self.workflow["delivery_x"],self.workflow["delivery_y"]]))))
            self.command_sent=self.pose_sim_time
    def move_to_goal(self):
        if self.workflow and self.robot_state=="MOVING_TO_DELIVERY" and not (self.coordination and not self.coordination.done):
            if self.goal_x is not None and math.hypot(self.goal_x-self.x,self.goal_y-self.y)<self.goal_tolerance and self.current_waypoint_index+1>=len(self.waypoints):
                self.stop_robot();self.goal_x=self.goal_y=None
                if not self.carry_verified:self.fail_task("Insufficient physical carry",holding=True);return
                self.transition("DELIVERY_DOCKING","Delivery approach reached");return
            return AMRController.move_to_goal(self)
        return super().move_to_goal()
    def environment_state(self,msg):
        value=json.loads(msg.data)
        if not value.get("confirmed"):return
        self.environment=value
        if not self.warehouse_map.apply(value):return
        self.emit("MAP_SYNCHRONIZED",version=value["version"],affected_cells=self.warehouse_map.affected_cells())
        if self.goal_x is None or not self.robot_state.startswith("MOVING"):return
        route=[(self.x,self.y)]+self.waypoints[self.current_waypoint_index:]
        invalid=any(not self.astar.has_line_of_sight(a,b) for a,b in zip(route,route[1:]))
        if not invalid:return
        self.stop_robot()
        self.emit("ROUTE_INVALIDATED",version=value["version"],original_path=route)
        if self.coordination and not self.coordination.done:
            self.coordination.failed=True
            self.fail_task("Environment changed during agreed coordination session",holding=bool(self.workflow));return
        started=time.monotonic()
        state=self.robot_state;goal=(self.target_world_x,self.target_world_y)
        self.emit("REPLANNING",version=value["version"])
        if self.set_route(goal,state):
            self.reroute_count+=1
            self.emit("REPLAN_COMPLETED",version=value["version"],latency_seconds=time.monotonic()-started,
                      path=[(self.x,self.y)]+self.waypoints)
        else:
            self.goal_x=self.goal_y=None
            self.emit("BLOCKED",version=value["version"])
    def publish_reservation(self):
        if self.coordination:return super().publish_reservation()
        self.coordination_pub.publish(String(data=json.dumps(dict(robot_id=self.robot_id,
            capability="dds_reservation_v1",task_id=self.current_task_id,
            route=[(self.x,self.y)]+self.waypoints[self.current_waypoint_index:],
            environment_version=getattr(self.warehouse_map,"version",0),
            footprint_radius=self.warehouse_map.robot_inflation))))
    def finish_task(self):
        super().finish_task()
        if self.environment:self.warehouse_map.apply(self.environment)
    def fail_task(self,reason,holding=False):
        super().fail_task(reason,holding);self.workflow=None

class WarehousePeer(WebotsPeer):
    def __init__(self):
        super().__init__();self.pickup_capable=True
        self.audit=self.create_publisher(String,"/warehouse/decisions",10)
    def evaluate_tasks(self):
        previous=set(self.finalized_tasks);super().evaluate_tasks()
        for task in self.finalized_tasks-previous:
            self.audit.publish(String(data=json.dumps(dict(robot_id=self.robot_id,task_id=task,winner=self.task_claims.get(task)))))
def main_navigation():run(WarehouseNavigation)
def main_peer():run(WarehousePeer)
if __name__=="__main__":
    import sys
    mode=sys.argv.pop(1)
    (main_navigation if mode=="navigation" else main_peer)()
