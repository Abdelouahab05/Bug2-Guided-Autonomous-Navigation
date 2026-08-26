# Embedded Autonomous Navigation Robot 

A differential-drive mobile robot that autonomously drives from a start point to a goal, avoids
obstacles it discovers along the way with a **Bug2** reactive strategy, and confirms it reached the
right target by reading an **ArUco marker** with a camera. Built as a second-year engineering
project at ENP Algiers.

This README explains what the system is, how the two boards talk to each other, how to wire
and flash it, and how to run it — so you can rebuild the same robot from this repo.

---

## 1. What this robot does

1. Drives in a straight line toward a goal coordinate.
2. If the LiDAR detects an obstacle blocking the way, it stops, turns, and follows the
   obstacle's boundary until it can safely rejoin its original path (this is the Bug2 algorithm).
3. Once it estimates it has reached the goal, it stops and scans for an **ArUco marker** with its
   camera to confirm it arrived at the right place.

---

## 2. Two-board architecture — why, and how it splits

The robot is split across two processors, each doing the job it's actually good at:

| Domain | Board | Job | Runs at |
|---|---|---|---|
| Low-level | STM32F429I-DISC1  | Encoder decoding, per-wheel PID speed control, PWM to motors | 20 Hz fixed loop |
| High-level | Raspberry Pi 4  | LiDAR obstacle processing, sensor fusion / odometry, Bug2 decision logic, ArUco vision | as fast as it can, tolerant of jitter |

## 3. Why not one board for everything?

A microcontroller can't run OpenCV/vision fast enough,
and a Linux SBC can't guarantee a jitter-free 20 Hz control loop because the OS scheduler will
occasionally delay it which corrupts PID tracking. Splitting the work removes both problems.
This tradeoff is discussed in depth (with a 3-way comparison against Arduino and ESP32 for the
low-level role) 

The two boards are connected by a single UART link at 921,600 baud, exchanging small fixed-size
binary frames every control cycle — nothing else. This keeps the interface simple and fast to parse.

```
Pi 4  ──USART3 @ 921600 baud──  STM32F429I-DISC1
 │                                     │
 ├─ RPLIDAR A2M8 (USB)                 ├─ 2× DC motor + quadrature encoder (TIM3/TIM4)
 ├─ MPU6050 IMU (I2C)                  └─ L298N H-bridge → motors (PWM on TIM2)
 └─ Pi Camera v2 (CSI, for ArUco)
```

---

## 4. Bill of materials

| Part | Notes |
|---|---|
| STM32F429I-DISC1 discovery board | Cortex-M4F @ 180 MHz, low-level motor control |
| Raspberry Pi 4 (4 GB recommended) | High-level perception/navigation |
| RPLIDAR A2M8 | 360° LiDAR, USB-serial |
| MPU6050 | I2C 6-axis IMU, only gyro Z is used |
| Pi Camera v2 (IMX219) | CSI-2, ArUco goal validation only |
| L298N dual H-bridge | motor driver |
| 2× geared DC motors + quadrature encoders | ~1900 pulses/rev effective resolution used in firmware |
| Differential-drive chassis | robot length 0.35 m, width 0.28 m, wheelbase 0.345 m, wheel diameter 0.075 m in this build, edit `config.py` / `main.c` constants if yours differ |
| Battery + regulation, E-stop switch | shared power for both domains |

---

## 5. Repo layout

```
main.c            STM32 firmware (bare-metal C, STM32CubeIDE / HAL project)
main.py            Raspberry Pi entry point — brings all Python modules together
config.py          ALL tunable constants (ports, geometry, speeds, thresholds, goal)
comms.py           Binary UART protocol implementation (Pi side)
odometry.py        Pose [x, y, theta] integration from STM32 fused odometry
motion.py           High-level move_forward()/turn_left()/turn_right() primitives
bug2.py             The Bug2 state machine (GO_TO_GOAL / WALL_FOLLOW / REACHED_GOAL)
lidar.py            RPLIDAR driver + sector/obstacle-threshold processing
imu.py              MPU6050 wrapper (only used for gyro calibration currently)
aruco_scanner.py    OpenCV ArUco detection at the goal
math_utils.py       Point2D, angle helpers, M-line / goal-tolerance geometry
```

If you want to see exactly how a piece works, go to the source file, don't re-derive it from
this README the code is the ground truth and is commented with `[MOVE]`, `[TURN]`, `[ARUCO]`
etc. print tags you can watch live over SSH/serial when bringing your own robot up.

---

## 6. STM32 side 

This is a STM32CubeIDE-generated HAL project, open it in CubeIDE / CubeMX to see and regenerate the peripheral configuration (clocks, GPIO,
timer prescalers) rather than hand-editing the `MX_*_Init()` functions.

What to check in `main.c` if you're rebuilding this:

- **`MX_TIM3_Init()` / `MX_TIM4_Init()`** — the two timers configured in **hardware encoder mode**
  to decode the left/right quadrature encoders with zero CPU overhead (X4 decoding). This is why
  encoder reading doesn't need an interrupt per pulse.
- **`MX_TIM2_Init()`** — generates the PWM used to drive the L298N (channels 1 and 2 = right/left
  motor speed).
- **`Set_Motor_Direction_Left()` / `Set_Motor_Direction_Right()`** — sets direction via the
  `ML_IN1/IN2` and `MR_IN1/IN2` GPIOs and writes the signed PWM compare value.
- **The PID gains** (`Kp_l, Ki_l, Kd_l` / `Kp_r, Ki_r, Kd_r`, near the top of the file) — tune
  these for your own motors/gearbox; they will not transfer directly to different hardware.
- **`WHEEL_DIAMETER`, `WHEEL_DISTANCE`, `ENCODER_PPR`** constants near the top — must match your
  physical robot.
- **`MX_USART3_UART_Init()` + `HAL_UART_RxCpltCallback()`** — the serial link to the Pi. This is
  where downlink velocity commands are received.
- **`while (1)` main loop** — the 50 ms (20 Hz) periodic control task: read encoders → compute
  RPM → run PID → set PWM → send uplink telemetry.

If you're new to STM32CubeIDE/HAL, open this file in the IDE, look at the **Pinout & Configuration** view for the timer/UART setup, and read the `USER
CODE` sections for the actual control logic.

---

## 7. Raspberry Pi side 

### 7.1 Install

```bash
sudo apt install python3-opencv python3-serial python3-smbus2 python3-pip
pip install pyrplidar
```

You'll also need the `mpu6050` Python driver for `imu.py` — install whatever MPU6050 library your
system provides and matching import path.

### 7.2 Configure

Everything tunable lives in **`config.py`** — serial ports, robot geometry, safety distances,
Bug2 goal position, speeds, ArUco settings. Start here before touching any other file:

```python
SERIAL_PORT = '/dev/ttyUSB0'   # STM32
LIDAR_PORT  = '/dev/ttyUSB1'   # RPLIDAR
GOAL_POSITION = (1.0, 0.0)     # meters, in the robot's start frame
```

### 7.3 Run

```bash
python3 main.py
```

`main.py` (see `Robot` class) does, in order: initializes IMU/LiDAR/comms → calibrates the
gyroscope (**keep the robot still during this step**) → starts the LiDAR spin-up and the 50 Hz
hardware loop thread → waits `SENSOR_STABILIZE_SEC` → runs the Bug2 navigator → on goal reached,
runs one ArUco scan → shuts everything down safely (motors to zero, LiDAR motor off) on
completion or Ctrl+C.

---

## 8. The communication protocol

Fixed-length binary frames, little-endian floats, no text parsing. Full detail (including why the
uplink has a sync byte and the downlink doesn't) is implemented in **`comms.py`** on the Pi side
and the UART section of `main.c` on the STM32 side.

**Uplink (STM32 → Pi), 9 bytes:**

| Byte(s) | Field |
|---|---|
| 0 | header `0xAA` |
| 1–4 | `RPM_L` (float32) |
| 5–8 | `RPM_R` (float32) |

**Downlink (Pi → STM32), 8 bytes, no header:**

| Byte(s) | Field |
|---|---|
| 0–3 | `v_target` linear m/s (float32) |
| 4–7 | `omega_target` rad/s (float32) |

The Pi needs the header byte because the Linux serial stack buffers bytes without preserving
frame boundaries — it has to scan for `0xAA` to resynchronize. The STM32 doesn't need one: it
reads bare-metal at a fixed cadence, so the fixed frame length is enough to delimit messages.

---

## 9. How the navigation logic works 

Three states:

- **`GO_TO_GOAL`** — turn to face the goal, then drive forward (`motion.move_forward`). If LiDAR
  reports the front blocked, record the current position as the **hit point** and switch to wall
  following.
- **`WALL_FOLLOW`** — follow the obstacle boundary (left-hand wall following, maintaining
  `WALL_FOLLOW_DISTANCE`) until the robot re-crosses the **M-line** (the straight line from start
  to goal) *closer to the goal than the hit point was* — then go back to `GO_TO_GOAL`.
- **`REACHED_GOAL`** — stop, hand off to `aruco_scanner.py`.

All the geometry (M-line test, goal tolerance, heading-to-goal) is in `math_utils.py`. All the
obstacle-distance thresholds (front/back/left/right, widened while rotating) are computed in
`lidar.py::_compute_dynamic_thresholds()` — they scale with the robot's physical footprint
(`ROBOT_LENGTH`, `ROBOT_WIDTH` in `config.py`), not a single circular safety radius, since the
chassis is longer than it is wide.

---

## 10. LiDAR angle convention

`lidar.py` assumes:

```
0°   = front
90°  = left
180° = back
270° = right
```

The correction `angle = (180 - scan.angle) % 360` in `lidar.py::_process_single_point()` maps
the RPLIDAR's raw angle to this convention for **this specific mounting orientation**. If you
mount your RPLIDAR differently, you'll need to change this line — check the comment at the top
of `lidar.py` and re-derive the offset for your own scanner orientation before trusting the
obstacle-avoidance behavior.

---

## 11. Odometry: how position is estimated

Odometry is **not** computed on the Pi from encoder counts directly. The STM32 sends fused
`(delta_s, theta_fused)` values (distance since last packet, and absolute fused heading), and
`odometry.py::update()` projects that onto the global `(x, y, theta)` frame every cycle. The
heading fusion itself (gyroscope + encoder heading via a complementary filter) happens as
described in the report's sensor-fusion chapter — if you're implementing your own version of this
on the STM32 side, that's the algorithm to replicate; it isn't in the Python code because it runs
on the microcontroller.

---

## 12. Safety notes

- The firmware and `main.py::shutdown()` both zero the motor targets and call
  `comms.emergency_stop()` (which spams a zero-velocity packet 5×) on Ctrl+C don't remove this
  if you're modifying the shutdown path.
- Fit a physical emergency-stop switch on the power rail. Software stops are not a substitute.
- Calibrate the gyro (`imu.py::calibrate()`) with the robot completely still, it's blocking and
  runs at the start of every `main.py` execution.

---

## 13. Known limitations 

- Pure dead-reckoning: position error is small over a single mission but grows unbounded over
  long runs, there's no absolute position reference (no SLAM).
- No global map: Bug2 guarantees reaching a reachable goal, not an efficient path around complex
  obstacle layouts.
- Vision is used only for the final goal-marker check, not for navigation, so lighting only
  matters at the very end of a run.



Academic project, ENP Algiers, Electronics Department, 2025–2026. Feel free to fork and adapt —
if you rebuild this, start by editing `config.py` and the geometry constants in `main.c` to match
your own chassis before trusting any of the tuned speeds or PID gains.
