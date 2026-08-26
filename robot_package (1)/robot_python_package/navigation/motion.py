import time
import math

from config import (
    FORWARD_SPEED, FORWARD_MIN_SPEED,
    TURN_SPEED, TURN_MIN_SPEED
)
from utils.math_utils import normalize_angle


class MotionController:
    """
    High-level motion commands.

        motion = MotionController(target_lock, target_linear, target_angular, odometry, lidar)
        motion.move_forward(0.50)   # Move 50cm forward
        motion.turn_right(90)       # Turn 90° clockwise
        motion.turn_left(90)        # Turn 90° counter-clockwise
    """

    def __init__(self, target_lock, target_linear_ref, target_angular_ref, odometry, lidar):
        self._lock = target_lock
        self._lin = target_linear_ref
        self._ang = target_angular_ref
        self._odom = odometry
        self._lidar = lidar

    def _set_targets(self, linear: float, angular: float):
        # Thread-safe target setter
        with self._lock:
            self._lin[0] = linear
            self._ang[0] = angular

    def _stop(self):
        # Stop motors
        self._set_targets(0.0, 0.0)
        time.sleep(0.2)

    def move_forward(self, distance_meters: float, max_speed: float = None,
                     min_speed: float = None, check_obstacle: bool = True) -> bool:

        max_speed = max_speed or FORWARD_SPEED
        min_speed = min_speed or FORWARD_MIN_SPEED
        decel_dist = 0.12
        KP = 3.0

        print(f"   [MOVE] Forward {distance_meters:.2f}m")

        start_x, start_y, start_theta = self._odom.get_pose()

        while True:
            # Check obstacle using YOUR LidarPerception
            if check_obstacle:
                if self._lidar.is_front_blocked():
                    front_dist = self._lidar.get_front_distance()
                    print(f"   OBSTACLE at {front_dist:.2f}m!")
                    self._stop()
                    return False

            x, y, theta = self._odom.get_pose()
            dx = x - start_x
            dy = y - start_y
            traveled = math.sqrt(dx**2 + dy**2)
            remaining = distance_meters - traveled

            if remaining <= 0.01:
                break

            # Speed profile
            if remaining < decel_dist:
                speed = min_speed + (max_speed - min_speed) * (remaining / decel_dist)
            else:
                speed = max_speed

            # Heading correction
            heading_error = normalize_angle(theta - start_theta)
            corrected_angular = -heading_error * KP

            self._set_targets(speed, corrected_angular)
            time.sleep(0.02)

        self._stop()
        return True

    def turn_right(self, degrees: float) -> None:
        #Turn right (clockwise) by specified degrees
        self._turn(-degrees)

    def turn_left(self, degrees: float) -> None:
        #Turn left (counter-clockwise) by specified degrees
        self._turn(degrees)

    def _turn(self, degrees: float) -> None:
        #Turn by specified degrees (positive=CCW, negative=CW)
        direction = "RIGHT" if degrees < 0 else "LEFT"
        print(f"   [TURN {direction}] {abs(degrees):.0f} deg")

        # Tell LiDAR we're rotating
        self._lidar.set_rotating(True)

        target_rad = math.radians(degrees)
        decel_angle = math.radians(20.0)

        _, _, current_theta = self._odom.get_pose()
        target_heading = normalize_angle(current_theta + target_rad)

        while True:
            _, _, theta = self._odom.get_pose()
            error = normalize_angle(target_heading - theta)

            if abs(error) < math.radians(3.0):
                break

            direction_sign = 1.0 if error > 0 else -1.0

            if abs(error) < decel_angle:
                speed = TURN_MIN_SPEED + (TURN_SPEED - TURN_MIN_SPEED) * (abs(error) / decel_angle)
            else:
                speed = TURN_SPEED

            self._set_targets(0.0, speed * direction_sign)
            time.sleep(0.02)

        self._stop()
        # Tell LiDAR we're done rotating
        self._lidar.set_rotating(False)
