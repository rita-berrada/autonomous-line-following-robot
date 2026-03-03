# Autonomous Line-Following Robot — Robotics Bootcamp

An autonomous wheeled robot built on a **Raspberry Pi** using **Object-Oriented Programming**. The project progressed through four challenges: PID line-following, an infinity/figure-8 circuit, a Manhattan grid navigation problem, and live obstacle avoidance.

---

## Hardware

| Component | Role |
|---|---|
| Raspberry Pi | Main controller (GPIO, I2C, PWM) |
| 2× DC Motors | Differential drive (left/right wheels) |
| 3× Servo Motors | Steering + scanning head (via PCA9685) |
| Adafruit PCA9685 | 16-channel PWM controller for servos |
| 3× IR Line Sensors | Line detection (left, middle, right) |
| Ultrasonic Sensor | Distance measurement for obstacle detection |
| SSD1306 OLED (128×64) | Real-time status display |
| 3× LEDs | Obstacle direction indicators (front, left, right) |
| MPU6050 IMU | Accelerometer / gyroscope (optional) |

---

## Project Progression

### 1. PID Line-Following — Infinity/Figure-8 Circuit
**File:** `navigation/infinity_circuit.py`

The robot follows a white line on a dark surface using three IR sensors. A **PID controller** computes a steering correction at every loop iteration:

| Sensor pattern `(L, M, R)` | Interpretation | Error |
|---|---|---|
| `(1, 0, 1)` | Centred | 0.0 |
| `(1, 0, 0)` | Slight right deviation | +1.0 |
| `(1, 1, 0)` | Sharp right deviation | +2.0 |
| `(0, 0, 1)` | Slight left deviation | −1.0 |
| `(0, 1, 1)` | Sharp left deviation | −2.0 |
| `(0, 0, 0)` | Lost line — use previous error | prev |

**Tuned PID parameters:**
- `Kp = 7.5` — proportional gain
- `Ki = 2.5` — integral gain (history window of 10 samples, clamped to ±40)
- `Kd = 2.5` — derivative gain
- Max angle correction: ±25°

The robot was tested on a figure-8/infinity circuit to validate PID tuning through both left and right turns.

---

### 2. Manhattan Phase 1 — A→B Navigation
**File:** `navigation/manhattan_phase1.py`

The robot navigates a **5×5 Manhattan grid** from a start node to a target node. Approach:

- **BFS pathfinding** (`algorithms/bfs_pathfinding.py`) computes the shortest action sequence (`forward`, `left`, `right`, `stop`) taking current orientation into account.
- Edges can be statically marked as blocked (simulating walls).
- The robot detects **intersections** via the `(0,0,0)` sensor pattern, then executes the next turn maneuver.
- Turning maneuvers use servo steering + forward/backward pulses to achieve clean 90° turns.

---

### 3. Manhattan Phase 2 — Live Obstacle Avoidance
**File:** `navigation/manhattan_phase2.py`

Same Manhattan navigation but with **dynamic obstacle detection and path recalculation**:

- At each intersection the robot scans **three directions** (front, left, right) by rotating its servo head and measuring ultrasonic distances.
- Edges closer than **70 cm** are flagged as blocked.
- BFS is **re-run** with the updated blocked-edge set; if the original path is blocked the robot finds an alternate route.
- A `PositionTracker` class maintains the robot's current grid position and heading.
- LEDs indicate which direction(s) are blocked; the OLED shows scan results and available paths.

---

## Repository Structure

```
robotics-bootcamp/
├── hardware/               # Hardware abstraction layer (drivers)
│   ├── dc_motor.py         # DC motor PWM control
│   ├── servo.py            # Calibrated 3-servo controller (PCA9685)
│   ├── line_sensor.py      # 3-IR line sensor reader
│   ├── oled_display.py     # SSD1306 OLED display
│   ├── imu.py              # MPU6050 accelerometer / angle calculator
│   └── leds.py             # Threaded LED blink controller
│
├── algorithms/             # Core algorithms (hardware-independent)
│   ├── pid_controller.py   # Reusable PID class
│   └── bfs_pathfinding.py  # BFS on a 5×5 grid with orientation tracking
│
├── navigation/             # Main challenge scripts
│   ├── infinity_circuit.py # PID line-following on figure-8 circuit
│   ├── manhattan_phase1.py # A→B grid navigation (static obstacles)
│   └── manhattan_phase2.py # A→B grid navigation + live obstacle avoidance
│
├── utils/
│   └── obstacle_detection.py  # Intersection-level obstacle scanning helper
│
├── examples/               # Concurrency / OOP demos
│   ├── led_class.py        # OOP LED controller with threading.Event
│   └── threading_led_demo.py  # Demo: safe multi-threaded LED control
│
└── docs/
    └── presentation.pdf    # Project presentation slides
```

---

## Setup & Dependencies

Install dependencies on the Raspberry Pi:

```bash
pip install RPi.GPIO
pip install adafruit-circuitpython-pca9685
pip install mpu6050-raspberrypi
pip install luma.oled
```

Run a navigation script (e.g., the infinity circuit):

```bash
python navigation/infinity_circuit.py
```

---

## Team

**Team 01** — Robotics Bootcamp
