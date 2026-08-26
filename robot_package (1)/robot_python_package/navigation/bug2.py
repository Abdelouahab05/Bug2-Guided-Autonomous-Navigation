import time
from enum import Enum, auto
from typing import Optional

from config import (
    GOAL_POSITION, M_LINE_TOLERANCE, GOAL_TOLERANCE,
    WALL_FOLLOW_DISTANCE, WALL_FOLLOW_TOLERANCE,
    WALL_FOLLOW_SPEED, WALL_FOLLOW_TURN,
    WALL_LOST_CONTINUE_DISTANCE,
    FORWARD_SPEED, FORWARD_MIN_SPEED
)
from utils.math_utils import (
    Point2D, HitPoint, normalize_angle,
    compute_heading_to_goal, is_on_m_line, is_at_goal
)


class Bug2State(Enum):
    GO_TO_GOAL = auto()
    WALL_FOLLOW = auto()
    REACHED_GOAL = auto()


class Bug2Navigator:

    def __init__(self, odometry, motion, lidar):
        self._odom = odometry
        self._motion = motion
        self._lidar = lidar

        self.goal = Point2D(*GOAL_POSITION)
        self.state = Bug2State.GO_TO_GOAL
        self.hit_point: Optional[HitPoint] = None
        self._loop_count = 0

    def _get_pose(self) -> Point2D:
        x, y, _ = self._odom.get_pose()
        return Point2D(x, y)

    def _is_closer_than_hit(self, current_pos: Point2D) -> bool:
        if self.hit_point is None:
            return False
        current_dist = current_pos.distance_to(self.goal)
        return current_dist < self.hit_point.distance_to_goal - 0.05

    def _can_leave_wall(self, current_pos: Point2D) -> bool:
        return is_on_m_line(current_pos, self.goal, M_LINE_TOLERANCE) and                self._is_closer_than_hit(current_pos)

    def run(self):
        # Main Bug2 navigation loop

        print("\n" + "=" * 55)
        print("BUG2 NAVIGATION STARTING")
        print(f"   Goal: {self.goal}")
        print("   Using YOUR LidarPerception obstacle states")
        print("=" * 55)

        self._odom.reset()
        time.sleep(0.5)

        while self.state != Bug2State.REACHED_GOAL:
            self._loop_count += 1
            pos = self._get_pose()
            _, _, theta = self._odom.get_pose()

            # Get LidarPerception data
            states = self._lidar.get_obstacle_states()
            ranges = self._lidar.get_ranges()

            print(f"\n[Loop {self._loop_count}] {self.state.name} | "
                  f"Pos: {pos} | θ: {math.degrees(theta):.1f}°")
            print(f"   LiDAR: F={ranges['FRONT']:.2f} L={ranges['LEFT']:.2f} "
                  f"R={ranges['RIGHT']:.2f} B={ranges['BACK']:.2f}")
            print(f"   States: FRONT={states['FRONT_BLOCKED']} LEFT={states['LEFT_BLOCKED']} "
                  f"RIGHT={states['RIGHT_BLOCKED']} BACK={states['BACK_BLOCKED']}")

            # Check goal reached
            if is_at_goal(pos, self.goal, GOAL_TOLERANCE):
                print("\n GOAL REACHED!")
                self.state = Bug2State.REACHED_GOAL
                break

            # --- GO TO GOAL ---
            if self.state == Bug2State.GO_TO_GOAL:
                heading = compute_heading_to_goal(pos, self.goal)
                error = normalize_angle(heading - theta)

                print(f"   -> Heading: {math.degrees(heading):.1f}° | Error: {math.degrees(error):.1f}°")

                # Align to goal heading
                if abs(error) > math.radians(5.0):
                    if error > 0:
                        self._motion.turn_left(math.degrees(abs(error)))
                    else:
                        self._motion.turn_right(math.degrees(abs(error)))
                    continue

                # Move forward
                success = self._motion.move_forward(0.50, max_speed=FORWARD_SPEED,
                                                     min_speed=FORWARD_MIN_SPEED)

                if not success:
                    # Hit obstacle! Record and switch to wall-follow
                    hit_pos = self._get_pose()
                    hit_dist = hit_pos.distance_to(self.goal)
                    self.hit_point = HitPoint(hit_pos, hit_dist)

                    print(f"\n HIT POINT: {self.hit_point}")
                    print("   -> WALL_FOLLOW mode")

                    self._motion.turn_right(90)
                    self.state = Bug2State.WALL_FOLLOW
                    time.sleep(0.3)

            # --- WALL FOLLOW ---
            elif self.state == Bug2State.WALL_FOLLOW:
                if self.hit_point is None:
                    print("ERROR: No hit point!")
                    self.state = Bug2State.GO_TO_GOAL
                    continue

                # Check departure condition
                if self._can_leave_wall(pos):
                    print(f"\n M-LINE RE-INTERCEPTED at {pos}")
                    heading = compute_heading_to_goal(pos, self.goal)
                    _, _, theta = self._odom.get_pose()
                    error = normalize_angle(heading - theta)
                    if error > 0:
                        self._motion.turn_left(math.degrees(abs(error)))
                    else:
                        self._motion.turn_right(math.degrees(abs(error)))

                    self.state = Bug2State.GO_TO_GOAL
                    self.hit_point = None
                    time.sleep(0.3)
                    continue

                states = self._lidar.get_obstacle_states()
                ranges = self._lidar.get_ranges()

                # Front obstacle during wall-follow: turn right
                if states["FRONT_BLOCKED"]:
                    print(f"   Front blocked! Turning right...")
                    self._motion.turn_right(90)
                    time.sleep(0.2)
                    continue

                # Wall lost on left: continue 40cm then re-acquire
                if not states["LEFT_BLOCKED"]:
                    print(f"   Wall lost (left clear). "
                          f"Continuing {WALL_LOST_CONTINUE_DISTANCE}m...")

                    self._motion.move_forward(
                        WALL_LOST_CONTINUE_DISTANCE,
                        max_speed=WALL_FOLLOW_SPEED,
                        min_speed=WALL_FOLLOW_SPEED * 0.7,
                        check_obstacle=False
                    )

                    print("   Turning left to re-acquire wall...")
                    self._motion.turn_left(90)
                    time.sleep(0.2)
                    continue

                # Wall following maintain distance 
                left_dist = ranges["LEFT"]
                wall_error = left_dist - WALL_FOLLOW_DISTANCE

                if abs(wall_error) < WALL_FOLLOW_TOLERANCE:
                    angular_cmd = 0.0
                    linear_cmd = WALL_FOLLOW_SPEED
                elif wall_error > 0:
                    # Too far, turn left toward wall
                    angular_cmd = WALL_FOLLOW_TURN
                    linear_cmd = WALL_FOLLOW_SPEED * 0.7
                else:
                    # Too close, turn right away
                    angular_cmd = -WALL_FOLLOW_TURN
                    linear_cmd = WALL_FOLLOW_SPEED * 0.7

                self._motion._set_targets(linear_cmd, angular_cmd)
                time.sleep(0.02)

            # Safety
            if self._loop_count > 1000:
                print("\n Max iterations reached.")
                break

        self._motion._stop()

        if self.state == Bug2State.REACHED_GOAL:
            print("\n" + "=" * 55)
            print(" MISSION ACCOMPLISHED")
            x, y, theta = self._odom.get_pose()
            print(f"   Final: ({x:.3f}, {y:.3f}) | θ: {math.degrees(theta):.1f}°")
            print("=" * 55)
        else:
            print("\n Mission terminated before goal.")


import math  
