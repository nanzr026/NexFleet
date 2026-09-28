"""Optional bounded graph advice around the unchanged decentralized bidder."""
import json
import time
from std_msgs.msg import String
from .fleet_graph import advisory_bid, valid_advice
from .webots_nodes import WebotsPeer, run

class GraphPeer(WebotsPeer):
    def __init__(self):
        super().__init__()
        self.requests={}
        self.advice={}
        self.active_candidate=None
        self.request_pub=self.create_publisher(String,f"/{self.robot_id}/graph_request",10)
        self.audit_pub=self.create_publisher(String,"/edge/bid_audit",10)
        self.decision_pub=self.create_publisher(String,"/edge/decision",10)
        self.create_subscription(String,f"/{self.robot_id}/graph_advice",self.receive_advice,10)
        self.create_timer(.05,self.flush_requests)

    def task_callback(self,msg):
        try:
            task=json.loads(msg.data)
            task_id=task["task_id"]
            if task_id in self.requests or task_id in self.tasks:
                return
            self.requests[task_id]=(time.monotonic(),msg)
            self.request_pub.publish(msg)
        except (ValueError,KeyError,TypeError):
            return

    def receive_advice(self,msg):
        try:
            value=json.loads(msg.data)
            if value.get("robot_id")==self.robot_id and value.get("task_id") in self.requests:
                self.advice[value["task_id"]]=value
        except (ValueError,TypeError):
            pass

    def flush_requests(self):
        for task_id,(started,msg) in list(self.requests.items()):
            if task_id in self.advice or time.monotonic()-started>1.5:
                self.active_candidate=task_id
                # Parent still checks battery, availability, pose freshness and load capability.
                super().task_callback(msg)
                self.requests.pop(task_id,None)
                self.active_candidate=None

    def evaluate_tasks(self):
        previous=set(self.finalized_tasks)
        super().evaluate_tasks()
        for task_id in self.finalized_tasks-previous:
            self.decision_pub.publish(String(data=json.dumps(dict(robot_id=self.robot_id,
                task_id=task_id,winner=self.task_claims.get(task_id)))))

    def calculate_bid(self,x,y):
        base=super().calculate_bid(x,y)
        advice=self.advice.get(self.active_candidate,{})
        now=time.monotonic()
        final=advisory_bid(base,advice,self.active_candidate,now)
        event="AI_ADVICE" if valid_advice(advice,self.active_candidate,now) else "AI_FAILURE"
        if event=="AI_FAILURE":
            self.get_logger().warning("AI_FAILURE: deterministic bidding fallback")
        self.audit_pub.publish(String(data=json.dumps(dict(event=event,robot_id=self.robot_id,
            task_id=self.active_candidate,deterministic=base,final=final,advice=advice))))
        return final

def main():run(GraphPeer)
if __name__=="__main__":main()
