"""Bounded PIBT and lazy constraint-addition configuration search.

Each caller owns a copy. No ROS, command output, or central fleet authority.
LaCAM structure follows Okumura (AAAI 2023): DFS over configurations,
BFS over partial successor constraints, PIBT as configuration generator.
Bounds deliberately sacrifice completeness on timeout; failure means STOP.
"""
from collections import deque
import math
import time


def swept_distance(a, b, c, d):
    """Minimum separation for two simultaneous straight-line transitions."""
    x, y = a[0]-c[0], a[1]-c[1]
    vx, vy = b[0]-a[0]-d[0]+c[0], b[1]-a[1]-d[1]+c[1]
    den = vx*vx+vy*vy
    t = max(0., min(1., -(x*vx+y*vy)/den)) if den else 0.
    return math.hypot(x+t*vx, y+t*vy)


def point_segment_distance(p, a, b):
    return swept_distance(a, b, p, p)


class ConfigurationSearch:
    def __init__(self, nodes, edges, goals, priorities, clearance=1.3):
        self.nodes = tuple(tuple(p) for p in nodes)
        self.adj = [set([i]) for i in range(len(nodes))]
        for a, b in edges:
            if a == b or not (0 <= a < len(nodes) and 0 <= b < len(nodes)):
                raise ValueError('Invalid graph edge')
            self.adj[a].add(b)
            self.adj[b].add(a)
        self.goals = tuple(goals)
        self.priorities = tuple(priorities)
        self.clearance = clearance
        self.distances = []
        for goal in goals:
            distance = {goal: 0}
            queue = deque([goal])
            while queue:
                v = queue.popleft()
                for u in sorted(self.adj[v]):
                    if u not in distance:
                        distance[u] = distance[v]+1
                        queue.append(u)
            self.distances.append(distance)
        self.inheritances = 0
        self.backtracks = 0

    def candidates(self, q, i):
        # Equal-cost sidesteps precede waiting, avoiding unnecessary local stops.
        return sorted(self.adj[q[i]], key=lambda v:
                      (v == q[i] and q[i] != self.goals[i], self.distances[i].get(v, math.inf), v))

    def compatible(self, q, next_q):
        return all(swept_distance(self.nodes[q[i]], self.nodes[next_q[i]],
                                  self.nodes[q[j]], self.nodes[next_q[j]]) >= self.clearance
                   for i in range(len(q)) for j in range(i))

    def pibt(self, q, constraints=()):
        """Recursive priority inheritance with assignment rollback/backtracking."""
        fixed = dict(enumerate(constraints))
        if any(v not in self.adj[q[i]] for i, v in fixed.items()):
            return None
        assigned = {}
        visiting = set()

        def assign(i):
            if i in assigned:
                return True
            if i in visiting:
                return False
            visiting.add(i)
            choices = [fixed[i]] if i in fixed else self.candidates(q, i)
            for v in choices:
                before = dict(assigned)
                if any(swept_distance(self.nodes[q[i]], self.nodes[v],
                                      self.nodes[q[j]], self.nodes[u]) < self.clearance
                       for j, u in assigned.items()):
                    continue
                assigned[i] = v
                # An occupied target inherits the caller's priority recursively.
                blockers = [j for j in range(len(q)) if j != i and j not in assigned
                            and math.dist(self.nodes[v], self.nodes[q[j]]) < self.clearance]
                good = True
                for j in blockers:
                    self.inheritances += 1
                    if not assign(j):
                        good = False
                        break
                if good:
                    visiting.remove(i)
                    return True
                assigned.clear()
                assigned.update(before)
                self.backtracks += 1
            visiting.remove(i)
            return False

        for i in sorted(range(len(q)), key=lambda i: (-self.priorities[i], i)):
            if not assign(i):
                return None
        result = tuple(assigned[i] for i in range(len(q)))
        return result if self.compatible(q, result) else None

    def lacam(self, start, max_expansions=20000, timeout=3.0):
        """Lazy successor constraints, not a priority-sort labelled LaCAM."""
        start = tuple(start)
        stack = [start]
        parent = {start: None}
        queues = {start: deque([()])}
        deadline = time.monotonic()+timeout
        expansions = 0
        while stack and expansions < max_expansions and time.monotonic() < deadline:
            q = stack[-1]
            if q == self.goals:
                path = []
                while q is not None:
                    path.append(q)
                    q = parent[q]
                return list(reversed(path)), expansions
            queue = queues[q]
            if not queue:
                stack.pop()
                continue
            constraints = queue.popleft()
            expansions += 1
            depth = len(constraints)
            if depth < len(q):
                for v in self.candidates(q, depth):
                    queue.append(constraints+(v,))
            successor = self.pibt(q, constraints)
            if successor is not None and successor not in parent:
                parent[successor] = q
                queues[successor] = deque([()])
                stack.append(successor)
        return None, expansions
