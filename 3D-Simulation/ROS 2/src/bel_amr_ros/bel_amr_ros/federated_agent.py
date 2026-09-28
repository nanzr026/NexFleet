"""Peer FedAvg over DDS; raw observations stay in the owning robot's buffer."""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from .edge_learning import CostModel, bootstrap, fedavg


class FederatedAgent(Node):
    def __init__(self):
        super().__init__('federated_agent')
        self.declare_parameter('robot_id', 'amr_01')
        self.declare_parameter('run_id', 'manual_demo')
        self.declare_parameter('bootstrap', False)
        self.declare_parameter('rounds', 3)
        self.robot_id = self.get_parameter('robot_id').value
        self.run_id = self.get_parameter('run_id').value
        self.members = ['amr_01', 'amr_02', 'amr_03']
        if self.robot_id not in self.members:
            raise ValueError('Robot not a member of this training cohort')
        self.version = 0
        self.model = CostModel()
        self.updates = {}
        self.outgoing = None
        self.assignments = {}
        self.seen_results = set()
        self.data_path = Path.home()/'.local/share/bel_amr'/self.robot_id/'experience.jsonl'
        self.data_path.parent.mkdir(parents=True, exist_ok=True)
        self.samples = []
        if self.data_path.exists():
            for line in self.data_path.read_text().splitlines()[-512:]:
                try:
                    row = json.loads(line)
                    self.model.predict(row['x'])
                    if 0 <= row['y'] <= 1:
                        self.samples.append(row)
                except (ValueError, KeyError, TypeError):
                    pass
        if not self.samples and self.get_parameter('bootstrap').value:
            for row in bootstrap(self.robot_id):
                self.save_sample(row)
            self.get_logger().warning('Using SYNTHETIC BOOTSTRAP samples, not real operations')
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.update_pub = self.create_publisher(String, '/federated/model_update', qos)
        self.model_pub = self.create_publisher(String, f'/{self.robot_id}/learned_model', qos)
        self.status_pub = self.create_publisher(String, '/federated/round_status', 10)
        self.create_subscription(String, '/federated/model_update', self.receive_update, qos)
        self.create_subscription(String, f'/{self.robot_id}/assigned_task', self.assignment, 10)
        self.create_subscription(String, '/task/result', self.result, 10)
        self.create_timer(5.0, self.tick)

    def save_sample(self, row):
        self.samples.append(row)
        self.samples = self.samples[-512:]
        with self.data_path.open('a') as stream:
            stream.write(json.dumps(row)+'\n')

    def assignment(self, msg):
        try:
            task = json.loads(msg.data)
            self.model.predict(task['learning_features'])
            self.assignments[task['task_id']] = task['learning_features']
        except (ValueError, KeyError, TypeError):
            return

    def result(self, msg):
        try:
            event = json.loads(msg.data)
            task_id = event['task_id']
            if event['robot_id'] != self.robot_id or task_id not in self.assignments or task_id in self.seen_results:
                return
            target = min(1.0, float(event['duration'])/300) if event['status'] == 'COMPLETED' else 1.0
            self.save_sample({'x': self.assignments.pop(task_id), 'y': target,
                              'source': 'simulation_observation', 'task_id': task_id})
            self.seen_results.add(task_id)
        except (ValueError, KeyError, TypeError):
            return

    def tick(self):
        if self.version >= self.get_parameter('rounds').value or not self.samples:
            return
        if self.outgoing is None:
            local = CostModel(self.model.weights)
            metrics = local.train(self.samples)
            self.outgoing = {'run_id': self.run_id, 'robot_id': self.robot_id,
                'round': self.version+1, 'parent': self.model.digest(),
                'weights': local.weights, 'samples': len(self.samples), **metrics}
            self.get_logger().info(f'LOCAL TRAINING COMPLETE {self.robot_id}: {metrics}')
        self.update_pub.publish(String(data=json.dumps(self.outgoing)))
        self.try_aggregate()

    def receive_update(self, msg):
        try:
            update = json.loads(msg.data)
            if update['run_id'] != self.run_id or update['robot_id'] not in self.members:
                return
            if not isinstance(update['round'], int) or not 1 <= update['round'] <= self.get_parameter('rounds').value:
                return
            fedavg([update])
            key = (update['round'], update['robot_id'])
            if key in self.updates and self.updates[key] != update:
                self.get_logger().error('Conflicting update from the same peer/round; retaining first')
                return
            self.updates[key] = update
            self.try_aggregate()
        except (ValueError, TypeError, KeyError, OverflowError):
            self.get_logger().warning('Rejected malformed federated update')

    def try_aggregate(self):
        round_id = self.version+1
        cohort = [self.updates.get((round_id, rid)) for rid in self.members]
        if not self.outgoing or any(update is None for update in cohort):
            return
        if any(update['parent'] != self.model.digest() for update in cohort):
            self.get_logger().error('Round parent mismatch; keeping last model')
            return
        self.model = fedavg(cohort)
        self.version = round_id
        payload = {'run_id': self.run_id, 'robot_id': self.robot_id, 'version': self.version,
                   'weights': self.model.weights, 'digest': self.model.digest(),
                   'samples': sum(update['samples'] for update in cohort),
                   'synthetic_samples': sum(row['source'] == 'synthetic_bootstrap' for row in self.samples)}
        message = String(data=json.dumps(payload))
        self.model_pub.publish(message)
        self.status_pub.publish(message)
        self.get_logger().info(f'FedAvg received 3 updates; MODEL VERSION {self.version}; digest={self.model.digest()}')
        # Retain each previous update for DDS late peers, and start next local round.
        self.outgoing = None


def main(args=None):
    rclpy.init(args=args)
    node = FederatedAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
