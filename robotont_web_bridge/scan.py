"""Directional scan projection, independent of ROS transport."""
import math

DIRECTIONS = {'front': 0.0, 'left': math.pi / 2, 'back': math.pi, 'right': -math.pi / 2}


def project_scan(ranges, angle_min, angle_increment, range_min, range_max, translation, quaternion):
    """Nearest horizontal distance in ±20° base-frame sectors; inf means clear."""
    qx, qy, qz, qw = quaternion
    r00, r01 = 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw)
    r10, r11 = 2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz)
    readings = {side: {'available': False, 'reason': 'Outside current sensor coverage'} for side in DIRECTIONS}
    for i, distance in enumerate(ranges):
        clear = distance == math.inf
        if not clear and (not math.isfinite(distance) or not range_min <= distance <= range_max):
            continue
        distance = range_max if clear else distance
        angle = angle_min + i * angle_increment
        sx, sy = distance * math.cos(angle), distance * math.sin(angle)
        x = translation[0] + r00 * sx + r01 * sy
        y = translation[1] + r10 * sx + r11 * sy
        bearing, horizontal = math.atan2(y, x), math.hypot(x, y)
        for side, direction in DIRECTIONS.items():
            delta = math.atan2(math.sin(bearing - direction), math.cos(bearing - direction))
            if abs(delta) <= math.radians(20):
                old = readings[side]
                if not old['available'] or horizontal < old['value']:
                    readings[side] = {'available': True, 'value': horizontal, 'clear': clear}
    return readings
