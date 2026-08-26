#!/usr/bin/env python3
"""
Architecture:
    main.py          : Robot class 
    config.py        : constants
    hardware/
        imu.py       : MPU6050 
        lidar.py     : RPlidar library
        comms.py     : ASCII protocol
    navigation/
        odometry.py  : delta_s,theta_fused
        motion.py 
        bug2.py  
    utils/
        math_utils.py : angle utilities

Serial Protocol:
    Stm32 -> Python: "data , 0.012 , 1.5708\n"
    Stm32 <- Python: "cmd , 0.550 , 0.000\n" 
"""

import threading
import time
import signal

from hardware.imu import IMU
from hardware.lidar import LidarPerception
from hardware.comms import STM32Comms
from navigation.odometry import Odometry
from navigation.motion import MotionController
from navigation.bug2 import Bug2Navigator
from hardware.aruco_scanner import scan_for_marker
from config import SENSOR_STABILIZE_SEC


class Robot:

    def __init__(self):
        self.running = True

        # Shared motion targets (mutable containers for thread-safe sharing)
        self._target_linear = [0.0]
        self._target_angular = [0.0]
        self._target_lock = threading.Lock()

        # Hardware modules
        self.imu = IMU()          
        self.lidar = LidarPerception() 
        self.comms = STM32Comms()    

        # Navigation modules
        self.odometry = Odometry()   
        self.motion = MotionController(
            self._target_lock,
            self._target_linear,
            self._target_angular,
            self.odometry,
            self.lidar
        )
        self.navigator = Bug2Navigator(self.odometry, self.motion, self.lidar)

        # Threads
        self._hardware_thread = None

    def _hardware_loop(self):

        # 50Hz hardware control loop.

        self.comms.flush()

        while self.running:
            start = time.time()

            # Read targets
            with self._target_lock:
                lin = self._target_linear[0]
                ang = self._target_angular[0]

            # Send command to Stm32 
            self.comms.send_command(lin, ang)

            # Read fused odometry from Stm32
            delta_s, theta_fused = self.comms.read_odometry()

            # Update odometry with Stm32 fused data
            if delta_s is not None and theta_fused is not None:
                self.odometry.update(delta_s, theta_fused)

            # Maintain 50Hz
            elapsed = time.time() - start
            sleep_time = max(0.0, 0.02 - elapsed)
            time.sleep(sleep_time)

    def start(self):
        # Initialize and start all systems
        print("=" * 55)
        print("ROBOT INITIALIZING")
        print("   Serial: STM32 ASCII protocol (DATA/CMD)")
        print("   LiDAR: rplidar library (rectangular footprint)")
        print("   Odometry: STM32 fused (delta_s, theta_fused)")
        print("=" * 55)

        # Calibrate IMU 
        self.imu.calibrate()

        # Start LiDAR 
        self.lidar.start()

        # Start hardware control loop
        self._hardware_thread = threading.Thread(target=self._hardware_loop, daemon=True)
        self._hardware_thread.start()

        # Wait for stabilization
        print(f"\nWaiting {SENSOR_STABILIZE_SEC}s for sensors to stabilize...")
        time.sleep(SENSOR_STABILIZE_SEC)

        print("\n All systems ready.")

    def run(self):
        
        # Run the main navigation sequence
        try:
            self.navigator.run()
        except KeyboardInterrupt:
            print("\n Interrupted by user.")
            return

        # Mission finished (goal reached) -> one-shot ArUco check
        print("\n Checking for ArUco marker...")
        marker_id = scan_for_marker()
        if marker_id is not None:
            print(f" Marker found: ID {marker_id}")
        else:
            print(" No marker found.")

    def shutdown(self):
        # Shutdown all systems

        print("\n Shutting down...")
        self.running = False

        # Stop motors
        with self._target_lock:
            self._target_linear[0] = 0.0
            self._target_angular[0] = 0.0

        self.comms.emergency_stop()

        # Stop LiDAR
        self.lidar.stop()

        # Close serial
        self.comms.close()

        print("Shutdown complete.")


def main():
    robot = Robot()

    # Handle Ctrl+C
    def signal_handler(sig, frame):
        robot.shutdown()
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Initialize
    robot.start()

    # Run navigation
    robot.run()

    # Shutdown
    robot.shutdown()


if __name__ == '__main__':
    main()