import math

# =============================================================================
# SERIAL / HARDWARE
# =============================================================================
SERIAL_PORT = '/dev/ttyUSB0'      # Stm32 motor controller
SERIAL_BAUD = 921600               # matches STM32 binary protocol
LIDAR_PORT = '/dev/ttyUSB1'       # RPLIDAR A2M8
LIDAR_BAUD = 115200               
DB_FILE = 'robot_telemetry.db'

# =============================================================================
# ROBOT GEOMETRY 
# =============================================================================
ROBOT_LENGTH = 0.35               # meters (Front-to-Back)
ROBOT_WIDTH = 0.28                # meters (Left-to-Right)

# =============================================================================
# LIDAR SAFETY & WALL FOLLOWING 
# =============================================================================
D_MIN = 0.25                      # Base minimum clearance to wall 
CORNER_MARG = 0.12                # Extra margin during rotations
SECTOR_WIDTH = 15.0               # +/- degrees tolerance window

# Derived thresholds 
FRONT_THRESH_BASE = (ROBOT_LENGTH / 2.0) + D_MIN       # ~0.425m
BACK_THRESH_BASE = (ROBOT_LENGTH / 2.0) + D_MIN       # ~0.425m
SIDE_THRESH_BASE = (ROBOT_WIDTH / 2.0) + D_MIN        # ~0.390m

# =============================================================================
# BUG2 / NAVIGATION
# =============================================================================
GOAL_POSITION = (1.0, 0.0)        
M_LINE_TOLERANCE = 0.03           #
GOAL_TOLERANCE = 0.02           

# Wall-following parameters
WALL_FOLLOW_DISTANCE = 0.30     
WALL_FOLLOW_TOLERANCE = 0.04    

# When wall disappears, continue this far then re-acquire
WALL_LOST_CONTINUE_DISTANCE = 0.40  

# =============================================================================
# SPEEDS
# =============================================================================
FORWARD_SPEED = 0.55
FORWARD_MIN_SPEED = 0.40
TURN_SPEED = 2.5
TURN_MIN_SPEED = 2.0

WALL_FOLLOW_SPEED = 0.45
WALL_FOLLOW_TURN = 1.8

# =============================================================================
# TIMING
# =============================================================================
CONTROL_HZ = 50                   # Hardware loop frequency
LIDAR_WARMUP_SEC = 1.0            
SENSOR_STABILIZE_SEC = 4.0

# =============================================================================
# ARUCO 
# =============================================================================
ARUCO_CAMERA_INDEX = 0
ARUCO_DICT_NAME = 'DICT_6X6_250'  # resolved against cv2.aruco in aruco_scanner.py
ARUCO_SCAN_DURATION_SEC = 3.0
ARUCO_FRAME_WIDTH = 640
ARUCO_FRAME_HEIGHT = 480