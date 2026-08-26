import math
from dataclasses import dataclass


@dataclass
class Point2D:
    x: float
    y: float

    def distance_to(self, other: "Point2D") -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def __repr__(self):
        return f"({self.x:.3f}, {self.y:.3f})"


@dataclass
class HitPoint:
    position: Point2D
    distance_to_goal: float

    def __repr__(self):
        return f"HitPoint(pos={self.position}, dist={self.distance_to_goal:.3f}m)"


def normalize_angle(angle_rad: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def angle_difference(target: float, current: float) -> float:
    """Smallest signed angle difference (target - current) in [-pi, pi]."""
    diff = target - current
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def point_to_line_distance(point: Point2D, line_start: Point2D, line_end: Point2D) -> float:
    """Perpendicular distance from point to infinite line."""
    dx = line_end.x - line_start.x
    dy = line_end.y - line_start.y

    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return point.distance_to(line_start)

    a = dy
    b = -dx
    c = dx * line_start.y - dy * line_start.x
    return abs(a * point.x + b * point.y + c) / math.sqrt(a**2 + b**2)


def compute_heading_to_goal(current_pos: Point2D, goal_pos: Point2D) -> float:
    """Heading angle (radians) from current position to goal."""
    dx = goal_pos.x - current_pos.x
    dy = goal_pos.y - current_pos.y
    return math.atan2(dy, dx)


def is_on_m_line(current_pos: Point2D, goal_pos: Point2D, tolerance: float) -> bool:
    """Check if robot is on the M-line (line from origin to goal)."""
    start = Point2D(0.0, 0.0)
    dist = point_to_line_distance(current_pos, start, goal_pos)
    return dist < tolerance


def is_at_goal(current_pos: Point2D, goal_pos: Point2D, tolerance: float) -> bool:
    """Check if robot has reached the goal."""
    return current_pos.distance_to(goal_pos) < tolerance
