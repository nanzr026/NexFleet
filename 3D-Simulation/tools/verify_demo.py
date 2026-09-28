"""Observe real ROS/Gazebo messages; fail rather than infer a successful demo."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import uuid

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String


class Probe(Node):
    def __init__(self):
        super().__init__('codex_demo_probe')
        self.poses = {}
        self.odom = {}
        self.scans = {}
        self.telemetry = {}
        self.results = {}
        self.attachment = None
        self.bids = {}
        self.claims = {}
        self.history = []
        self.last_sample = 0.0
        self.create_timer(0.5, self.sample)
        for rid in ('amr_01', 'amr_02', 'amr_03'):
            self.create_subscription(String, f'/{rid}/task_claim',
                lambda msg, key=rid: self.claims.update({key: json.loads(msg.data)}), 10)
        for rid in ('amr_01', 'amr_02', 'amr_03', 'pallet_box_mobile'):
            self.create_subscription(PoseStamped, f'/model/{rid}/pose',
                lambda msg, key=rid: self.poses.update({key: (msg.pose.position.x, msg.pose.position.y)}) if msg.header.frame_id == 'world_demo' else None, 10)
            if rid.startswith('amr'):
                self.create_subscription(Odometry, f'/model/{rid}/odometry',
                    lambda msg, key=rid: self.odom.update({key: True}), 10)
                self.create_subscription(LaserScan, f'/model/{rid}/scan',
                    lambda msg, key=rid: self.scans.update({key: len(msg.ranges)}), 10)
                self.create_subscription(String, f'/{rid}/telemetry',
                    lambda msg, key=rid: self.telemetry.update({key: json.loads(msg.data)}), 10)
                self.create_subscription(String, f'/{rid}/task_bid',
                    lambda msg, key=rid: self.bids.update({key: json.loads(msg.data)}), 10)
        self.create_subscription(String, '/amr_02/product_01/attached', self.attached, 10)
        self.create_subscription(String, '/task/result', self.result, 10)
        self.detach = self.create_publisher(Empty, '/amr_02/product_01/detach', 10)
        self.assignment = self.create_publisher(String, '/amr_02/assigned_task', 10)
        self.announcement = self.create_publisher(String, '/task/announcement', 10)

    def sample(self):
        self.history.append({'time': time.time(), 'poses': dict(self.poses),
                             'telemetry': dict(self.telemetry)})
        if self.attachment is None:
            self.attachment = self.telemetry.get('amr_02', {}).get('attachment')

    def attached(self, msg):
        self.attachment = msg.data

    def result(self, msg):
        event = json.loads(msg.data)
        self.results[event['task_id']] = event
        print('TASK RESULT', json.dumps(event), flush=True)

    def wait(self, condition, seconds, label, repeat=None):
        deadline = time.monotonic()+seconds
        next_repeat = 0
        next_log = 0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if condition():
                return
            now = time.monotonic()
            if repeat and now > next_repeat:
                repeat()
                next_repeat = now+1
            if now > next_log:
                print(label, json.dumps({'poses': self.poses, 'amr_02': self.telemetry.get('amr_02')}), flush=True)
                next_log = now+15
        raise RuntimeError(f'TIMEOUT: {label}; poses={self.poses}; telemetry={self.telemetry}')

    def task(self, kind, pickup, delivery=(-5.0, 10.0), auction=False, timeout=900):
        tid = 'DEMO_'+uuid.uuid4().hex[:8]
        task = {'task_id': tid, 'kind': kind, 'gazebo_model': 'pallet_box_mobile',
                'pickup_x': pickup[0], 'pickup_y': pickup[1],
                'delivery_x': delivery[0], 'delivery_y': delivery[1]}
        pub = self.announcement if auction else self.assignment
        self.wait(lambda: pub.get_subscription_count() > 0, 30, 'Waiting for task subscribers')
        pub.publish(String(data=json.dumps(task)))
        print('TASK SENT', json.dumps(task), flush=True)
        self.wait(lambda: tid in self.results, timeout, 'Awaiting measured task completion')
        result = self.results[tid]
        if auction:
            assert all(self.bids.get(r, {}).get('task_id') == tid
                       for r in ('amr_01', 'amr_02', 'amr_03')), 'Missing decentralized bids'
            expected = sorted((b for b in self.bids.values() if b['available']),
                              key=lambda b: (-b['bid'], b['robot_id']))[0]['robot_id']
            assert result['robot_id'] == expected
            assert self.claims.get(expected, {}).get('task_id') == tid
        if result['status'] != 'COMPLETED':
            raise RuntimeError(f'Task failed: {result}')
        return result


def set_pose(name, x, y, z):
    request = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}'
    command = ['gz', 'service', '-s', '/world/world_demo/set_pose', '--reqtype', 'gz.msgs.Pose',
               '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request]
    result = subprocess.run(command, capture_output=True, text=True, timeout=15)
    print('RESET', name, result.stdout, result.stderr, flush=True)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError(f'Set-pose failed: {command}: {result.stdout} {result.stderr}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['connectivity', 'reset', 'navigation', 'pickup', 'auction'])
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    report = {'mode': args.mode, 'started': time.time()}
    try:
        robots = ['amr_01', 'amr_02', 'amr_03']
        node.wait(lambda: all(r in node.poses and r in node.odom and node.scans.get(r, 0) > 0 for r in robots)
                  and 'pallet_box_mobile' in node.poses, 90, 'Checking all three robots pose/odometry/LiDAR')
        report['initial_poses'] = dict(node.poses)
        if args.mode == 'reset':
            if any(t.get('current_task') for t in node.telemetry.values()):
                raise RuntimeError('Stop and restart fleet before resetting an active task')
            node.wait(lambda: node.attachment == 'detached', 20, 'Detaching PRODUCT01',
                      lambda: node.detach.publish(Empty()))
            for rid, y in zip(robots, [-10.6, -8.0, -5.4]):
                set_pose(rid, 13.9, y, 0.10)
            set_pose('pallet_box_mobile', 4.4161, 14.6952, 0.17)
            node.wait(lambda: math.dist(node.poses['pallet_box_mobile'], (4.4161, 14.6952)) < 0.15
                      and all(math.dist(node.poses[rid], (13.9, y)) < 0.15
                              for rid, y in zip(robots, [-10.6, -8.0, -5.4])), 30, 'Verifying reset poses')
        elif args.mode == 'navigation':
            start = node.poses['amr_02']
            report['result'] = node.task('navigate', (12.4, -8.0), timeout=180)
            report['displacement'] = math.dist(start, node.poses['amr_02'])
            assert report['displacement'] > 1.0
        elif args.mode in ('pickup', 'auction'):
            initial_pallet = node.poses['pallet_box_mobile']
            assert math.dist(initial_pallet, (4.4161,14.6952)) < 0.15
            report['result'] = node.task('pickup', (4.4161, 14.6952), delivery=(8.0, 18.5),
                                         auction=args.mode == 'auction')
            assert report['result']['carry_verified']
            approach_samples = [h for h in node.history
                if h['telemetry'].get('amr_02',{}).get('state') == 'MOVING_TO_PICKUP']
            assert approach_samples, 'No physical approach samples'
            drift = max(math.dist(h['poses']['pallet_box_mobile'], initial_pallet)
                        for h in approach_samples)
            assert drift < 0.15, f'Pallet moved during approach: {drift}'
            report['pallet_drift_during_approach'] = drift
            report['bids'] = node.bids
            report['claims'] = node.claims
            node.wait(lambda: node.telemetry.get('amr_02', {}).get('state') == 'IDLE',
                      10, 'Waiting for IDLE')
        report['status'] = 'PASS'
    except Exception as error:
        report['status'] = 'FAIL'
        report['error'] = str(error)
        raise
    finally:
        report['elapsed'] = time.time()-report['started']
        report['history'] = node.history
        report['final_poses'] = dict(node.poses)
        output = Path(__file__).resolve().parents[1]/'demo_logs'
        output.mkdir(exist_ok=True)
        (output/f'{args.mode}_result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k:v for k,v in report.items() if k != 'history'}, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
