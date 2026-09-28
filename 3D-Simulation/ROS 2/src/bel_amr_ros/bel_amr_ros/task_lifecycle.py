"""Pickup/delivery state machine independent of ROS and the simulator.

Service durations model station handling for the prototype, not a physical
gripper. A hardware adapter should advance these stages on station feedback.
"""

import math


NAVIGATING = ('MOVING', 'TO_PICKUP', 'TO_DELIVERY')


def normalize_task(data, home):
    task = dict(data)
    task['task_id'] = str(task['task_id'])
    if not task['task_id']:
        raise ValueError('empty task ID')
    if ('delivery_x' in task) != ('delivery_y' in task):
        raise ValueError('delivery requires both x and y')
    task.setdefault('delivery_x', home[0])
    task.setdefault('delivery_y', home[1])
    task.setdefault('pickup_seconds', 1.0)
    task.setdefault('drop_seconds', 1.0)
    task.setdefault('pickup_approach_offset', 1.2952)
    for key in ('pickup_x', 'pickup_y', 'delivery_x', 'delivery_y',
                'pickup_seconds', 'drop_seconds', 'pickup_approach_offset'):
        task[key] = float(task[key])
        if not math.isfinite(task[key]):
            raise ValueError(f'{key} must be finite')
    if min(task['pickup_seconds'], task['drop_seconds'],
           task['pickup_approach_offset']) < 0:
        raise ValueError('durations and approach offset must be nonnegative')
    task['priority'] = int(task.get('priority', 1))
    return task


class TaskLifecycle:
    def __init__(self, task, now):
        self.task = task
        self.state = 'ASSIGNED'
        self.started = now
        self.deadline = None

    def start(self):
        if self.state != 'ASSIGNED':
            raise ValueError('task already started')
        self.state = 'TO_PICKUP'

    def arrived(self, now):
        if self.state == 'TO_PICKUP':
            self.state = 'PICKING'
            self.deadline = now + self.task['pickup_seconds']
        elif self.state == 'TO_DELIVERY':
            self.state = 'DROPPING'
            self.deadline = now + self.task['drop_seconds']
        else:
            raise ValueError('arrival outside navigation stage')

    def tick(self, now):
        if self.deadline is None or now < self.deadline:
            return False
        if self.state == 'PICKING':
            self.state = 'TO_DELIVERY'
        elif self.state == 'DROPPING':
            self.state = 'COMPLETED'
        else:
            return False
        self.deadline = None
        return True
