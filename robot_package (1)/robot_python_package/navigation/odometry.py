import threading
import time
import math

from utils.math_utils import normalize_angle


class Odometry:

    def __init__(self):
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        self._lock = threading.Lock()
        self._last_time = time.time()
        self._packet_count = 0

    def update(self, delta_s: float, theta_fused: float):

        # Update pose with fused odometry from Stm32 
        """
        Args:
            delta_s: Distance traveled since last packet (meters)
            theta_fused: Absolute orientation from STM32 filter (radians)
        """
        with self._lock:
            self._theta = normalize_angle(theta_fused)

            # Project delta_s onto global frame using fused heading
            self._x += delta_s * math.cos(self._theta)
            self._y += delta_s * math.sin(self._theta)

            self._packet_count += 1

    def get_pose(self) -> tuple:
        # Thread-safe pose getter. Returns (x, y, theta)
        with self._lock:
            return self._x, self._y, self._theta

    def reset(self):
        # Reset pose to origin
        with self._lock:
            self._x = 0.0
            self._y = 0.0
            self._theta = 0.0
            self._last_time = time.time()
            self._packet_count = 0

    @property
    def packet_count(self) -> int:
        return self._packet_count
