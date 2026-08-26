"""
RPLIDAR A2M8 Perception Module (PyRPlidar Integration)

Angle convention (YOUR system):
    0°   = Front
    90°  = Left
    180° = Back
    270° = Right (or -90°)
"""

import threading
import time
import sys

from pyrplidar import PyRPlidar

from config import (
    LIDAR_PORT, LIDAR_BAUD, LIDAR_WARMUP_SEC,
    ROBOT_LENGTH, ROBOT_WIDTH, D_MIN, CORNER_MARG, SECTOR_WIDTH
)


class LidarPerception:

    def __init__(self):
        self._running = False
        self._thread = None
        self._lidar = None

        # Shared state (Unchanged to maintain compatibility with linked codes)
        self._obstacle_states = {
            "FRONT_BLOCKED": False,
            "LEFT_BLOCKED": False,
            "BACK_BLOCKED": False,
            "RIGHT_BLOCKED": False,
        }
        self._closest_ranges = {
            "FRONT": float('inf'),
            "LEFT": float('inf'),
            "BACK": float('inf'),
            "RIGHT": float('inf'),
        }
        self._thresholds = {
            "FRONT": 0.0,
            "BACK": 0.0,
            "LEFT": 0.0,
            "RIGHT": 0.0,
        }
        self._is_rotating = False

        self._lock = threading.Lock()

    def set_rotating(self, rotating: bool):
        # Tell LiDAR if robot is turning 
        with self._lock:
            self._is_rotating = rotating

    def _compute_dynamic_thresholds(self):
        extra_padding = CORNER_MARG if self._is_rotating else 0.0

        front_thresh = (ROBOT_LENGTH / 2.0) + D_MIN + extra_padding
        back_thresh = (ROBOT_LENGTH / 2.0) + D_MIN + extra_padding
        side_thresh = (ROBOT_WIDTH / 2.0) + D_MIN + extra_padding

        return {
            "FRONT": front_thresh,
            "BACK": back_thresh,
            "LEFT": side_thresh,
            "RIGHT": side_thresh,
        }

    def _process_single_point(self, scan):
        # Processes an individual scan point yielded 
        try:
            distance_mm = scan.distance
            if distance_mm == 0:
                return

            # Apply your required angle and orientation correction
            angle = (180 - scan.angle) % 360
            distance_m = distance_mm / 1000.0

            thresholds = self._compute_dynamic_thresholds()
            
            # Local tracking variables for this processing cycle
            closest_ranges = {
                "FRONT": float('inf'),
                "LEFT": float('inf'),
                "BACK": float('inf'),
                "RIGHT": float('inf'),
            }

            # Sector Classification 
            if (angle <= SECTOR_WIDTH) or (angle >= 360.0 - SECTOR_WIDTH):
                if distance_m < closest_ranges["FRONT"]:
                    closest_ranges["FRONT"] = distance_m

            elif abs(angle - 90.0) <= SECTOR_WIDTH:
                if distance_m < closest_ranges["LEFT"]:
                    closest_ranges["LEFT"] = distance_m

            elif abs(angle - 180.0) <= SECTOR_WIDTH:
                if distance_m < closest_ranges["BACK"]:
                    closest_ranges["BACK"] = distance_m

            elif abs(angle - 270.0) <= SECTOR_WIDTH:
                if distance_m < closest_ranges["RIGHT"]:
                    closest_ranges["RIGHT"] = distance_m

            # Evaluate safety faults against adaptive geometry
            obstacle_states = {
                "FRONT_BLOCKED": closest_ranges["FRONT"] <= thresholds["FRONT"],
                "LEFT_BLOCKED": closest_ranges["LEFT"] <= thresholds["LEFT"],
                "BACK_BLOCKED": closest_ranges["BACK"] <= thresholds["BACK"],
                "RIGHT_BLOCKED": closest_ranges["RIGHT"] <= thresholds["RIGHT"],
            }

            with self._lock:
                # Update shared states 
                for key in closest_ranges:
                    if closest_ranges[key] < self._closest_ranges[key]:
                        self._closest_ranges[key] = closest_ranges[key]
                
                self._obstacle_states.update(obstacle_states)
                self._thresholds = thresholds

        except Exception:
            pass

    def _worker_loop(self):
        # Background scanning loop 
        try:
            self._lidar = PyRPlidar()
            print(f"Connecting to A2M8 LiDAR on {LIDAR_PORT}...")
            self._lidar.connect(port=LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=3)
            
            print("Starting motor...")
            self._lidar.set_motor_pwm(660)
            time.sleep(LIDAR_WARMUP_SEC)
            print("[INFO] LiDAR successfully initialized")

            scan_generator = self._lidar.force_scan()

            for scan in scan_generator():
                if not self._running:
                    break
                self._process_single_point(scan)

        except Exception as e:
            print(f"[LIDAR ERROR] {e}")
        finally:
            self._shutdown()

    def _shutdown(self):
        # Stop LiDAR 
        print("[CLEANUP] Stopping LiDAR...")
        try:
            if self._lidar:
                self._lidar.stop()
                self._lidar.set_motor_pwm(0)
                self._lidar.disconnect()
                print("[CLEANUP] LiDAR safe state completed.")
        except Exception as e:
            print(f"[WARNING] Cleanup error: {e}")

    # -------------------------------------------------------------------------
    # Public API for navigation 
    # -------------------------------------------------------------------------

    def start(self):
        # Start LiDAR background thread
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def stop(self):
        # Stop LiDAR background thread
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def get_obstacle_states(self) -> dict:
        # Get current obstacle detection states
        with self._lock:
            return dict(self._obstacle_states)

    def get_ranges(self) -> dict:
        # Get raw closest distances per quadrant
        with self._lock:
            return dict(self._closest_ranges)

    def get_thresholds(self) -> dict:
        # Get current dynamic thresholds
        with self._lock:
            return dict(self._thresholds)

    def is_front_blocked(self) -> bool:
        # Quick check: is front obstacle detected?
        with self._lock:
            return self._obstacle_states["FRONT_BLOCKED"]

    def is_left_blocked(self) -> bool:
        # Quick check: is left obstacle detected?
        with self._lock:
            return self._obstacle_states["LEFT_BLOCKED"]

    def get_front_distance(self) -> float:
        # Get front closest range 
        with self._lock:
            return self._closest_ranges["FRONT"]

    def get_left_distance(self) -> float:
        # Get left closest range 
        with self._lock:
            return self._closest_ranges["LEFT"]