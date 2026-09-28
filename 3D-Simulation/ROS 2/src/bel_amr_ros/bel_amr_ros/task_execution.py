"""Controller integration for the reusable task lifecycle."""

import json
import time

from std_msgs.msg import String


class TaskExecutionMixin:
    def publish_task_stage(self):
        self.robot_state = self.lifecycle.state
        event = {
            'robot_id': self.robot_id, 'task_id': self.current_task_id,
            'state': self.robot_state, 'timestamp': time.time(),
            'elapsed': time.monotonic() - self.lifecycle.started,
            'task': self.lifecycle.task,
        }
        self.task_event_pub.publish(String(data=json.dumps(event)))
        self.get_logger().info(f'{self.robot_state} | {self.current_task_id}')
        self.publish_status()
        self.publish_navigation_intent()

    def finish_navigation_leg(self):
        self.stop_robot()
        self.goal_x = self.goal_y = None
        self.waypoints = []
        self.world_path = []
        self.current_waypoint_index = 0
        self.lifecycle.arrived(time.monotonic())
        self.publish_task_stage()

    def advance_task_lifecycle(self):
        if self.lifecycle is None:
            return
        if self.robot_state in ('LOW_BATTERY', 'BLOCKED'):
            self.stop_robot()
            return
        if self.lifecycle.state == 'COMPLETED':
            self.completed_task_ids.add(self.current_task_id)
            self.current_task_id = None
            self.lifecycle = None
            self.robot_state = 'IDLE'
            self.publish_status()
            self.publish_navigation_intent()
            return
        if not self.lifecycle.tick(time.monotonic()):
            return
        if self.lifecycle.state == 'TO_DELIVERY':
            self.approach_world_x = self.lifecycle.task['delivery_x']
            self.approach_world_y = self.lifecycle.task['delivery_y']
            world_x, world_y = self.local_to_world(self.x, self.y)
            if not self.replan_around_dynamic_obstacle(
                    world_x, world_y, 'delivery-leg'):
                # Retain ownership and cargo until explicit recovery/release.
                self.lifecycle.state = 'BLOCKED'
        self.publish_task_stage()
