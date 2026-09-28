"""Local graph learner and decentralized FedAvg peer; publishes no motion commands."""
import json
import math
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from .fleet_graph import IDS, SCHEMA, snapshot, digest
from .graph_learning import ARCH, initial_model, weights, load, train, predict, fedavg
from .graph_experience import label_graphs

class GraphAgent(Node):
    def __init__(self):
        super().__init__("graph_agent")
        self.declare_parameter("robot_id", "amr_01")
        self.rid = self.get_parameter("robot_id").value
        self.run = os.environ["EDGE_RUN"]
        self.root = Path(os.environ["EDGE_ROOT"])
        self.out = self.root/"Webots/edge_ai_runs"/self.run/self.rid
        self.out.mkdir(parents=True, exist_ok=True)
        self.samples = json.loads((self.root/f"Webots/edge_ai_runs/seed/{self.rid}.json").read_text())
        self.mean = [sum(s["target"][i] for s in self.samples)/len(self.samples) for i in range(2)]
        self.model = initial_model()
        self.version = 0
        self.target_round = 3
        self.pending = {}
        self.sent = None
        self.states = {}
        self.goals = {}
        self.previous = {}
        self.last_graph = -1e9
        self.track = []
        self.task_id = None
        self.completed = None
        self.evaluated = False
        self.last_update = None
        qos = QoSProfile(depth=30, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.update_pub = self.create_publisher(String, "/edge/model_update", qos)
        self.status_pub = self.create_publisher(String, "/edge/status", qos)
        self.advice_pub = self.create_publisher(String, f"/{self.rid}/graph_advice", 10)
        self.create_subscription(String, "/edge/model_update", self.update, qos)
        self.create_subscription(String, "/edge/train_fresh", self.fresh, 10)
        self.create_subscription(String, "/task/result", self.result, 10)
        self.create_subscription(String, f"/{self.rid}/graph_request", self.request, 10)
        for rid in IDS:
            self.create_subscription(PoseStamped, f"/model/{rid}/pose",
                                     lambda m,r=rid:self.pose(r,m), 10)
            self.create_subscription(String, f"/{rid}/assigned_task",
                                     lambda m,r=rid:self.task(r,m), 10)
        self.create_timer(1., self.tick)

    def record(self, name, value):
        with (self.out/(name+".jsonl")).open("a") as f:
            f.write(json.dumps(value, allow_nan=False)+chr(10))

    def pose(self, rid, msg):
        if msg.header.frame_id != "webots_world":
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        x,y=msg.pose.position.x,msg.pose.position.y
        previous=self.previous.get(rid)
        self.previous[rid]=(stamp,x,y)
        if previous and stamp > previous[0]:
            dt=stamp-previous[0]
            self.states[rid]=dict(x=x,y=y,vx=(x-previous[1])/dt,vy=(y-previous[2])/dt,
                                  stamp=stamp, received=time.monotonic())

    def graph(self, goals=None):
        if len(self.states) != 3:
            return None
        now=time.monotonic()
        stamps=[s["stamp"] for s in self.states.values()]
        if max(stamps)-min(stamps)>.5 or any(now-s["received"]>2 for s in self.states.values()):
            return None
        return snapshot(self.states,max(stamps),self.goals if goals is None else goals,
                        "webots_pose_dds")

    def task(self, rid, msg):
        try:
            task=json.loads(msg.data)
            if not task.get("task_id") or not all(math.isfinite(float(task[k])) for k in ("pickup_x", "pickup_y")):
                return
        except (ValueError,TypeError,KeyError,AttributeError):
            return
        if task.get("kind") != "navigate":
            return
        self.goals[rid]=[task["pickup_x"],task["pickup_y"]]
        if rid==self.rid and task["task_id"] != self.task_id:
            self.task_id=task["task_id"]
            self.track=[]
            self.completed=None
            self.evaluated=False

    def result(self, msg):
        result=json.loads(msg.data)
        if result.get("status") in ("COMPLETED", "FAILED"):
            self.goals.pop(result.get("robot_id"), None)
        if result.get("robot_id")==self.rid and result.get("task_id")==self.task_id:
            if result.get("status")=="COMPLETED":
                graph=self.graph()
                if graph:
                    self.completed=graph["timestamp"]
                    self.track.append(graph)
                    self.evaluate()
            self.goals.pop(self.rid,None)

    def evaluate(self):
        if self.evaluated:
            return
        rows=label_graphs(self.track,self.rid,self.completed)
        if not rows:
            return
        predictions=[]
        for sample in rows:
            g=sample["graph"]
            # Remaining time assuming unobstructed 0.45m/s, zero proximity exposure.
            baseline=[min(1.,g["nodes"][self.rid]["distance_to_goal"]/.45/180),0.]
            predictions.append(dict(timestamp=g["timestamp"],target=sample["target"],
                gnn=predict(self.model,g,self.rid),
                no_neighbors=predict(self.model,g,self.rid,True),
                training_mean=self.mean,kinematic=baseline))
        scores={key:[sum(abs(p[key][i]-p["target"][i]) for p in predictions)/len(predictions)
                     for i in range(2)]
                for key in ("gnn","no_neighbors","training_mean","kinematic")}
        report=dict(robot_id=self.rid,task_id=self.task_id,version=self.version,
                    digest=digest(weights(self.model)),samples=len(rows),
                    mae=scores,remaining_time_mae_seconds={k:v[0]*180 for k,v in scores.items()},
                    predictions=predictions)
        self.record("evaluation",report)
        self.record("local_experience",dict(task_id=self.task_id,rows=rows))
        self.samples.extend(rows)
        self.evaluated=True
        self.status("EVALUATED",samples=len(rows),mae=scores)

    def fresh(self,msg):
        value=json.loads(msg.data)
        if value.get("run")==self.run and self.evaluated and self.version==3:
            self.target_round=4

    def status(self,event,**fields):
        value=dict(run=self.run,robot_id=self.rid,event=event,version=self.version,
                   digest=digest(weights(self.model)),**fields)
        self.status_pub.publish(String(data=json.dumps(value)))
        self.record("status",value)

    def request(self,msg):
        try:
            task=json.loads(msg.data)
            if not task.get("task_id") or not all(math.isfinite(float(task[k])) for k in ("pickup_x", "pickup_y")):
                return
        except (ValueError,TypeError,KeyError,AttributeError):
            return
        if self.version<3:
            return
        goals=dict(self.goals)
        goals[self.rid]=[task["pickup_x"],task["pickup_y"]]
        graph=self.graph(goals)
        if graph is None:
            return
        cost,risk=predict(self.model,graph,self.rid)
        advice=dict(run=self.run,robot_id=self.rid,task_id=task["task_id"],
                    time=time.monotonic(),schema=SCHEMA,version=self.version,
                    model_digest=digest(weights(self.model)),graph_digest=digest(graph),
                    suitability=1-cost,risk=risk)
        self.record("candidate",dict(graph=graph,advice=advice))
        self.advice_pub.publish(String(data=json.dumps(advice)))

    def update(self,msg):
        try:
            u=json.loads(msg.data)
            if u["run"]!=self.run or u["architecture"]!=ARCH or u["robot_id"] not in IDS:
                return
            version=u["version"]
            if not isinstance(version,int) or not self.version < version <= 4:
                return
            cohort=self.pending.setdefault(version,{})
            old=cohort.get(u["robot_id"])
            if old is not None and digest(old)!=digest(u):
                self.status("REJECTED",reason="conflicting duplicate update")
                return
            cohort[u["robot_id"]]=u
        except (ValueError,TypeError,KeyError):
            self.status("REJECTED",reason="malformed update")

    def tick(self):
        if self.version < self.target_round:
            next_version=self.version+1
            parent=digest(weights(self.model))
            if self.sent!=next_version:
                local=initial_model()
                load(local,weights(self.model))
                metrics=train(local,self.samples)
                self.last_update=dict(run=self.run,robot_id=self.rid,version=next_version,
                    architecture=ARCH,parent=parent,samples=len(self.samples),weights=weights(local))
                self.record("outgoing_update",self.last_update)
                self.update_pub.publish(String(data=json.dumps(self.last_update)))
                self.pending.setdefault(next_version,{})[self.rid]=self.last_update
                self.record("training",dict(version=next_version,parent=parent,**metrics))
                self.sent=next_version
            cohort=self.pending.get(next_version,{})
            if len(cohort)==3:
                try:
                    aggregation_started=time.perf_counter()
                    merged=fedavg(list(cohort.values()),parent,next_version,self.run)
                    load(self.model,merged)
                    self.version=next_version
                    (self.out/f"model_{self.version}.json").write_text(json.dumps(
                        dict(architecture=ARCH,version=self.version,weights=weights(self.model))))
                    self.status("AGGREGATED",counts={r:u["samples"] for r,u in cohort.items()},
                                aggregation_seconds=time.perf_counter()-aggregation_started)
                except (ValueError,KeyError,TypeError) as error:
                    self.status("REJECTED",reason=str(error))
        graph=self.graph()
        if graph and graph["timestamp"]-self.last_graph>=1:
            self.last_graph=graph["timestamp"]
            self.record("graphs",graph)
            if self.task_id and self.completed is None:
                self.track.append(graph)

def main():
    rclpy.init()
    node=GraphAgent()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=="__main__":main()
