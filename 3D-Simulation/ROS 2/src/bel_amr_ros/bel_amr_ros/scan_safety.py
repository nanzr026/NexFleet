"""Conservative scan interpretation and rotation-only recovery decisions."""

import math


def summarize_scan(ranges, angle_min, angle_increment, range_max):
    front = []
    all_ranges = []
    front_valid = True
    all_valid = True
    front_count = 0
    for index, distance in enumerate(ranges):
        angle = angle_min + index * angle_increment
        is_front = abs(angle) <= math.pi / 6
        usable = (not math.isnan(distance) and distance >= 0
                  and (math.isfinite(distance) or distance == math.inf))
        all_valid &= usable
        if is_front:
            front_count += 1
            front_valid &= usable
        if usable:
            # A positive infinite return means no hit within sensor range.
            distance = min(distance, range_max)
            all_ranges.append(distance)
            if is_front:
                front.append((distance, angle))
    distance, beam = min(front, default=(0.0, 0.0))
    coverage = abs(angle_increment) * max(0, len(ranges) - 1)
    return {
        'distance': distance, 'beam': beam,
        'front_valid': front_valid and front_count > 0,
        'rotation_clearance': min(all_ranges, default=0.0),
        'rotation_valid': all_valid and coverage >= 2 * math.pi - 0.1,
    }


def recovery_rotation(angle_error, obstacle_beam, clearance, valid,
                      required_clearance=0.60, speed=0.25):
    if not valid or clearance < required_clearance:
        return 0.0
    target_separation = abs(math.atan2(
        math.sin(obstacle_beam - angle_error),
        math.cos(obstacle_beam - angle_error)))
    if abs(angle_error) <= math.pi / 6 + 0.1:
        return 0.0
    if target_separation <= abs(obstacle_beam) + 0.1:
        return 0.0
    direction = math.copysign(1.0, angle_error)
    # Initially turn away from an off-centre obstacle, even if that takes the
    # longer rotation to the waypoint. This never grants forward motion.
    if abs(obstacle_beam) > 0.05 and obstacle_beam * direction > 0:
        direction *= -1
    return direction * speed
