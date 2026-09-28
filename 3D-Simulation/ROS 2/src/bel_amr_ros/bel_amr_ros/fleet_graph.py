"""Measured fleet graphs and bounded advice; no motion authority."""
import hashlib
import json
import math

IDS = ("amr_01", "amr_02", "amr_03")
SCHEMA = "fleet_xy_velocity_goal_v1"
RADIUS = 6.0

def snapshot(states, timestamp, goals, source):
    """Require every measured pose/velocity; absent goals mean idle, not missing data."""
    nodes = {}
    for rid in IDS:
        s = states[rid]
        x, y, vx, vy = (float(s[k]) for k in ("x", "y", "vx", "vy"))
        if not all(math.isfinite(v) for v in (x, y, vx, vy, timestamp)):
            raise ValueError("Non-finite measurement")
        goal = goals.get(rid)
        if goal is not None and (len(goal) != 2 or not all(math.isfinite(v) for v in goal)):
            raise ValueError("Invalid measured task goal")
        distance = math.dist((x, y), goal) if goal is not None else 0.0
        speed = math.hypot(vx, vy)
        nodes[rid] = dict(x=x, y=y, vx=vx, vy=vy, speed=speed, goal=goal,
                          task_active=goal is not None, distance_to_goal=distance,
                          features=[x/20, y/16, vx/.45, vy/.45, speed/.45,
                                    distance/50, float(goal is not None)])
    edges = []
    for a, na in nodes.items():
        for b, nb in nodes.items():
            if a == b:
                continue
            dx, dy = nb["x"]-na["x"], nb["y"]-na["y"]
            dvx, dvy = nb["vx"]-na["vx"], nb["vy"]-na["vy"]
            distance = math.hypot(dx, dy)
            if distance <= RADIUS:
                relative_speed = math.hypot(dvx, dvy)
                closing_speed = -(dx*dvx+dy*dvy)/max(distance, 1e-6)
                edges.append(dict(source=a, target=b, distance=distance,
                                  relative_speed=relative_speed, closing_speed=closing_speed,
                                  features=[distance/RADIUS, relative_speed/.9, closing_speed/.9]))
    return dict(schema=SCHEMA, timestamp=timestamp, source=source, nodes=nodes, edges=edges)

def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()

def valid_advice(advice, task_id, now):
    try:
        if advice["task_id"] != task_id or not 0 <= now-advice["time"] <= 2:
            return False
        if advice["version"] < 1 or advice["schema"] != SCHEMA:
            return False
        return all(math.isfinite(float(advice[k])) and 0 <= float(advice[k]) <= 1
                   for k in ("suitability", "risk"))
    except (KeyError, TypeError, ValueError):
        return False

def advisory_bid(base, advice, task_id, now):
    """Availability is checked by the existing bidder before this function."""
    if not valid_advice(advice, task_id, now):
        return base
    return min(1., max(0., base + .08*(float(advice["suitability"])-.5)
                       - .05*float(advice["risk"])))
