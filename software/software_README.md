# Software Architecture & Autonomous Navigation

This document describes the complete software architecture of the **YBR-SUNFLOWER WRO 2026 Future Engineers robot**, including code organization, controller communication, localization, path generation, Pure Pursuit steering, computer vision, traffic-pillar handling, sensor fusion, parking, failure handling, testing, tuning, simulation, and software engineering decisions.

The purpose of this document is not only to describe what the code does, but also to explain:

- why the final software architecture was selected,
- how the software is divided into independent modules,
- how the Raspberry Pi and Arduino divide responsibilities,
- how the robot estimates its position on the WRO field,
- why we use localization and path following instead of relying only on reactive camera steering,
- why Pure Pursuit is used for path tracking,
- how red and green traffic pillars are detected and incorporated into the navigation path,
- how information from LiDAR, IMU, camera, and drivetrain motion is combined,
- how abnormal and low-confidence conditions are handled,
- how simulation and real-world testing are used for tuning,
- which performance metrics are used to evaluate the system,
- what limitations remain,
- and how another team can run and understand the competition software.

---

# Contents

1. [Software Engineering Overview](#1-software-engineering-overview)
2. [Software Requirements, Constraints and Design Philosophy](#2-software-requirements-constraints-and-design-philosophy)
3. [Codebase Architecture](#3-codebase-architecture)
4. [Competition Runtime and Control Flow](#4-competition-runtime-and-control-flow)
5. [Raspberry Pi ↔ Arduino Communication](#5-raspberry-pi--arduino-communication)
6. [Localization — Knowing Where We Are](#6-localization--knowing-where-we-are)
7. [Path Generation and Track Following](#7-path-generation-and-track-following)
8. [Obstacle Detection and Traffic-Pillar Strategy](#8-obstacle-detection-and-traffic-pillar-strategy)
9. [Parking Strategy](#9-parking-strategy)
10. [Sensor Fusion and Information Arbitration](#10-sensor-fusion-and-information-arbitration)
11. [Edge Cases, Safety and Failure Handling](#11-edge-cases-safety-and-failure-handling)
12. [Testing, Simulation and Tuning](#12-testing-simulation-and-tuning)
13. [Performance Metrics and Validation](#13-performance-metrics-and-validation)
14. [Software Development and Engineering Decisions](#14-software-development-and-engineering-decisions)
15. [Known Limitations and Future Improvements](#15-known-limitations-and-future-improvements)
16. [Running and Reproducing the Software](#16-running-and-reproducing-the-software)
17. [Final Software Architecture](#17-final-software-architecture)

---

# 1. Software Engineering Overview

The final YBR-SUNFLOWER software is built around one central idea:

> **Instead of continuously asking "what should the robot steer away from?", the robot asks "where am I on the field, where should I be, and how should I get there?"**

Our earlier approach relied more heavily on immediately reacting to what the camera could see.

The limitation of a camera-dominant reactive approach is that the robot mainly knows what is visible **at the current moment**.

Changes in:

- shadows,
- reflections,
- lighting,
- camera visibility,
- or temporary obstruction

can therefore affect the information available to the navigation system.

<img width="500" alt="robot_sim" src="https://github.com/user-attachments/assets/af5d16d1-98a7-4732-ac49-425d32fd90b3" />

The WRO field has known geometry.

The final software therefore uses this knowledge to estimate the robot's position on the field and drive a planned geometric path.

This changes the navigation problem into three primary questions:

1. **Where am I?**
2. **Where should I be?**
3. **How should I steer from my current position toward that target?**

The final system answers them using:

```text
Where am I?
→ LiDAR + IMU + motion model
→ Localization

Where should I be?
→ Racing Line

How do I get there?
→ Pure Pursuit
```

The camera has a more specialized role. It identifies red and green traffic pillars and estimates their position. Instead of switching the robot into a completely different obstacle controller, the software modifies the existing racing line around the detected pillar. The same Pure Pursuit controller then follows the modified path.

---

## 1.1 Final Autonomous Software Concept

```text
                    ENVIRONMENT
                         |
         +---------------+---------------+
         |               |               |
         v               v               v
       LiDAR           Camera           IMU
         |               |               |
         |               v               |
         |        Pillar Detection       |
         |               |               |
         +-------> Localization <---------+
                         |
                         v
                 Robot Pose Estimate
                 (x, y, heading)
                         |
                         +-------------------------+
                         |                         |
                         v                         v
                  Racing Line                Block Map
                         |                         |
                         +------------+------------+
                                      |
                                      v
                              Modified Path
                                      |
                                      v
                                Pure Pursuit
                                      |
                                      v
                             Steering + Speed
                                      |
                               USB Serial
                                      |
                                      v
                              Arduino UNO R4
                                  |       |
                                  v       v
                               Motor    Steering
```

---

## 1.2 Criterion 3 Evidence Map

| Level 6 Requirement | Evidence in This Document |
|---|---|
| Software architecture and modularity | Sections 3–4 |
| Control / state flow | Section 4 |
| Algorithm reasoning | Sections 6–10 |
| Localization strategy | Section 6 |
| Lane / path-following strategy | Section 7 |
| Obstacle strategy | Section 8 |
| Correct red / green passing logic | Section 8 |
| Parking strategy | Section 9 |
| Sensor fusion | Section 10 |
| Edge-case handling | Section 11 |
| Testing and tuning process | Section 12 |
| Performance metrics | Section 13 |
| Engineering trade-offs | Section 14 |
| Known limitations | Section 15 |
| Reproducibility | Section 16 |

---

# 2. Software Requirements, Constraints and Design Philosophy

The software must control a fully autonomous vehicle under randomized WRO competition conditions.

The system therefore cannot depend on:

- one manually selected starting heading,
- one fixed obstacle arrangement,
- one fixed camera image,
- or continuous human correction.

---

## 2.1 Main Software Requirements

The final software must:

- initialize all required hardware,
- wait for the physical competition start button,
- estimate the vehicle's position,
- determine the required driving direction,
- follow a repeatable path,
- respond to localization uncertainty,
- detect red and green traffic pillars,
- pass each pillar on the required side,
- maintain sufficient wall clearance,
- execute the parking sequence,
- stop when the round is completed,
- respect the competition time limit,
- and stop the actuators if the software exits unexpectedly.

---

## 2.2 Software Constraints

| Constraint | Software Response |
|---|---|
| Random starting orientation | Localization + automatic direction selection |
| Known field geometry | Field map used for localization |
| Limited camera reliability under lighting variation | Camera used mainly for colored traffic pillars |
| LiDAR cannot determine pillar color | Camera and LiDAR assigned different roles |
| IMU can drift | LiDAR geometry provides precise heading correction |
| Localization can become uncertain | Pose confidence tracked |
| Steering actuator has physical lag | Steering-rate handling and measured response |
| Camera processing is expensive | Camera pipeline runs at a slower cadence |
| Limited competition runtime | 180-second software timeout |
| Real robot differs from ideal geometry | Steering calibration tools |
| Field testing is time-consuming | Simulator runs real navigation code without hardware |
| Many parameters require tuning | TOML configuration files |
| Software may terminate unexpectedly | `finally` cleanup stops motor and centers steering |

---

## 2.3 Software Design Philosophy

The main software principles are:

### 1. Separate hardware access from competition behavior

Tasks should not directly construct hardware interfaces. Instead, managers and context objects isolate hardware-specific behavior.

### 2. Represent the robot in field coordinates

Whenever possible, decisions are based on:

```text
x
y
heading
```

rather than raw pixels or one distance reading.

### 3. Use one path-following controller

Normal navigation and obstacle avoidance should use the same steering architecture whenever possible.

### 4. Measure physical behavior instead of relying only on assumptions

Servo response and steering geometry are measured using dedicated tools.

### 5. Make tuning parameters editable without changing Python code

Competition tuning belongs in configuration files.

### 6. Fail safely

If execution terminates, the motor should not intentionally continue running.

---

# 3. Codebase Architecture

The competition software is divided between:

- **Arduino UNO R4 Minima**
- **Raspberry Pi 5**

The Arduino performs low-level actuator and encoder operations, while the Raspberry Pi performs the higher-level autonomous logic.

---

## 3.1 Controller Responsibility

```text
Raspberry Pi 5
────────────────────────
Perception
Localization
Field Model
Object Detection
Path Generation
Pure Pursuit
Speed Selection
Competition Logic
        |
        | USB Serial
        v
Arduino UNO R4 Minima
────────────────────────
Motor Output
Servo Output
Encoder Reading
Start Button
Distance-Based Motor Move
```

---

## 3.2 Arduino Program

The Arduino competition firmware is stored in:

```text
src/Arduino/Main.ino
```

Its responsibilities include:

- reading the quadrature encoder,
- controlling the drive motor,
- controlling the steering servo,
- monitoring the competition start button,
- receiving commands from the Raspberry Pi,
- and returning status responses.

The Arduino intentionally does not handle the high-level competition strategy. This keeps the low-level controller focused on reliable and deterministic hardware operations.

---

## 3.3 Raspberry Pi Program Structure

The Raspberry Pi software is stored in:

```text
src/Raspberrypi/
```

The code is separated so that:

> **"How the hardware works" and "what the robot should do" do not need to exist in the same module.**

The main structure is:

```text
src/Raspberrypi/
│
├── main.py
│
├── tasks/
│   ├── base_task.py
│   ├── path_task.py
│   ├── qualification/
│   └── final/
│
├── classes/
│   ├── task_context.py
│   ├── motor_manager.py
│   ├── lidar_manager.py
│   ├── compass_manager.py
│   ├── navigation_manager.py
│   ├── field_map.py
│   ├── racing_line.py
│   ├── pure_pursuit.py
│   ├── object_solver.py
│   ├── block_map.py
│   ├── steering_calibrator.py
│   └── debug_view.py
│
├── utils/
│
├── test_navigation.py
├── test_driving.py
├── test_steering.py
├── test_color_picker.py
│
├── pyproject.toml
└── uv.lock
```

---

### 3.3.1 Main Entry Point

### `main.py`

The main entry point selects which competition task to run.

Example:

```bash
uv run python main.py qualification
```

or:

```bash
uv run python main.py final
```

---

### 3.3.2 Task Layer

### `tasks/base_task.py`

Contains the common competition lifecycle:

```text
Wait for Start
      ↓
Initialize Run
      ↓
Setup Task
      ↓
Run Control Loop
      ↓
Finish Safely
```

### `tasks/path_task.py`

Contains the common path-driving logic shared by the competition rounds.

Responsibilities include:

- lap progress,
- localization,
- target-point selection,
- steering,
- speed selection,
- lost-pose handling,
- and safety.

### `tasks/qualification/`

Open Challenge behavior.

The qualification task largely reuses the standard path-driving architecture with its own configuration.

### `tasks/final/`

Obstacle Challenge behavior.

This adds:

- traffic-pillar detection,
- block mapping,
- racing-line deformation,
- and final-round behavior

on top of the common path-driving system.

---

### 3.3.3 Configuration Files

Each competition task uses a configuration file:

```text
tasks/*/config.toml
```

Tunable parameters include values such as:

- speed,
- lookahead,
- clearances,
- path settings,
- obstacle settings,
- and controller limits.

This means competition tuning can normally be performed without editing the Python implementation.

---

## 3.4 Core Software Classes

| File | Responsibility |
|---|---|
| `task_context.py` | Creates and initializes subsystems in the correct order and performs shutdown |
| `motor_manager.py` | Raspberry Pi ↔ Arduino serial communication |
| `lidar_manager.py` | RPLiDAR communication and 360-degree distance representation |
| `compass_manager.py` | BNO055 heading interface |
| `navigation_manager.py` | Field localization / particle filter |
| `field_map.py` | Geometric representation of the WRO field |
| `racing_line.py` | Desired closed path around the field |
| `pure_pursuit.py` | Converts robot pose + target point into steering |
| `object_solver.py` | Camera-based traffic-pillar detection and range estimation |
| `block_map.py` | Stores detected traffic pillars in field coordinates |
| `steering_calibrator.py` | Estimates real steering geometry |
| `debug_view.py` | Live visualization of localization and navigation |

---

## 3.5 Software-Layer Architecture

```text
                   COMPETITION TASK
                         |
                         v
                  Qualification / Final
                         |
                         v
                      PathTask
                         |
        +----------------+----------------+
        |                |                |
        v                v                v
   Localization     Racing Line      Object Map
        |                |                |
        +----------------+----------------+
                         |
                         v
                    Pure Pursuit
                         |
                         v
                  Motor Manager
                         |
                         v
                      Arduino
```

---

## 3.6 Active Competition Code vs Legacy / Experimental Code

### Active competition paths

```text
main.py
tasks/
classes/
utils/
tasks/qualification/
tasks/final/
```
---

# 4. Competition Runtime and Control Flow

The robot does not use one large function containing all competition logic. Instead, every competition round uses the same lifecycle.

---

## 4.1 Main Competition State Flow

```text
POWER ON
    |
    v
INITIALIZE HARDWARE
    |
    v
WAIT_FOR_START
    |
    v
CAPTURE INITIAL HEADING
    |
    v
SETUP
    |
    v
RUN CONTROL LOOP
    |
    +--------------------+
    |                    |
    | Finished?          | Timeout?
    |                    |
    +---------+----------+
              |
              v
            FINISH
              |
              v
      STOP MOTOR / CENTER
```

This state flow is implemented through the task architecture rather than through one monolithic state machine.

---

## 4.2 Base Task Runtime

The common runtime is implemented in `tasks/base_task.py`.

```python
def run(self):
    self.context.wait_for_start()
    self.context.compass.set_initial_heading()
    self.start_time = time.monotonic()

    try:
        self.setup()
        while not self.is_finished():
            if self.timed_out:
                print(f"Time limit ({self.max_runtime_s}s) reached - stopping.")
                break

            self.tick += 1
            self.step()

            if self.status_every and self.tick % self.status_every == 0:
                print(self.status())

            time.sleep(LOOP_DELAY_S)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        self.finish()
```

The loop delay is:

```text
LOOP_DELAY_S = 0.02 s
```

which gives a theoretical loop frequency of approximately:

```text
1 / 0.02 = 50 Hz
```

Actual iteration time can be lower depending on sensor and processing workload.

---

### 4.2.1 Why the `finally` Block Matters

The `finally` block is an important safety feature.

Whether the run finishes because of:

- normal task completion,
- the 180-second timeout,
- Ctrl-C,
- or a Python exception,

the cleanup code is still executed.

The final cleanup stops the drivetrain and returns the steering system to its safe state. This prevents a software exception from intentionally leaving the previous drive command active.

---

## 4.3 One Path-Following Tick

The normal path-driving loop performs the following sequence:

```python
def step(self):
    # 1. estimate how far the robot moved
    distance = self._travelled(dt)
    context.nav.report_motion(distance, self._turned(distance))

    # 2. obtain the current pose
    pose = context.nav.get_pose()

    # 3. update track progress and localization state
    self._track_progress(pose)
    self._update_lost_state(pose, now)

    # 4. choose the target point
    self.target = self.target_point(pose)

    # 5. calculate and rate-limit steering
    wanted = self.pursuit.steering(self._lead_pose(pose), self.target)
    self.steer_command = self._limit_steer_rate(wanted, dt)

    # 6. choose safe driving speed
    self.speed = self._choose_speed(pose)

    # 7. send one drive command
    context.motor.drive(self.steer_command, self.speed)
```

Conceptually:

```text
Motion Estimate
      ↓
Localization
      ↓
Track Progress
      ↓
Pose Confidence
      ↓
Target Point
      ↓
Pure Pursuit
      ↓
Steering Rate Limit
      ↓
Speed Selection
      ↓
Arduino Command
```

---

## 4.4 Obstacle-Round Processing Cadence

Camera processing is significantly more computationally expensive than a simple steering-control loop. Therefore, traffic-pillar detection does not need to run on every loop iteration. The final task runs camera detection at a slower rate and stores the detected results in the block map for use by the navigation system.

Example structure:

```python
def step(self):
    if self.tick % CAMERA_EVERY_N_TICKS == 0:
        self._update_detections()

    super().step()
```

This separates:

```text
High-rate path control
```

from:

```text
Lower-rate camera perception
```

The block map persists between camera frames, so the steering system does not lose a known pillar simply because the camera pipeline is not running on the current tick.

---

## 4.5 Competition Timeout

The software enforces the WRO run time limit through:

```text
max_runtime_s = 180 s
```

If the software reaches this limit, the control loop exits and the final cleanup is performed. This provides an explicit upper limit on autonomous execution.

---

# 5. Raspberry Pi ↔ Arduino Communication

The Raspberry Pi and Arduino run separate programs and communicate through USB Serial at 115200 baud. The communication protocol is intentionally kept small and human-readable, making it easier to understand, test, and debug.

---

## 5.1 Command Format

```text
<steering angle>,<speed>,<distance>\n
```

Example:

```text
30,55,0
```

---

## 5.2 Command Fields

### Steering Angle

```text
Negative = Left
0        = Center
Positive = Right
```

### Speed

```text
Positive = Forward
Negative = Reverse
0        = Stop
```

### Distance

The distance field represents a requested amount of motor-shaft rotation.

```text
distance = 0
```

means continuous driving until another command changes the drivetrain.

A non-zero value requests a finite encoder-controlled movement.

---

## 5.3 Arduino Response Messages

| Response | Meaning |
|---|---|
| `READY` | Arduino initialized |
| `Start` | Competition start switch triggered |
| `OK` | Continuous command accepted |
| `t` | Fixed-distance movement completed |
| `ERR` | Invalid command |

---

## 5.4 Why Use a Small Text Protocol?

Advantages include:

- easy debugging in a terminal,
- easy logging,
- human-readable commands,
- simple Arduino parser,
- low implementation complexity.

The trade-off is that text serialization is less compact than a binary protocol. For the relatively small amount of control data exchanged between the boards, simplicity was prioritized.

Detailed pin and electrical information is documented in:

[`../elec/elec_README.md`](../elec/elec_README.md)

---

# 6. Localization — Knowing Where We Are

The WRO field has known geometry. Rather than detecting only immediate wall position, the robot estimates its pose on this known field.

The pose is represented as:

```text
Pose
├── X position
├── Y position
├── Heading
└── Confidence
```

---

## 6.1 Localization Principle

For a given robot position and orientation, the surrounding field geometry produces a characteristic LiDAR distance pattern.

The software reverses this relationship:

> Given the LiDAR scan that we actually observe, which possible robot position best explains those measurements?

The final implementation uses a **particle filter**.

---

## 6.2 Particle Filter

The localization system begins with multiple possible robot poses.

The current implementation uses approximately:

```text
500 particles
```

Each particle represents a hypothesis:

```text
(x, y, heading)
```

During an update:

1. each particle moves according to the estimated robot motion,
2. expected field measurements are compared with the real LiDAR scan,
3. particles that better match the scan receive higher weight,
4. unlikely particles are removed,
5. high-probability particles are resampled.

Conceptually:

```text
Initial hypotheses
. . . . . . . . . . .
. . . . . . . . . . .

        ↓ motion
        ↓ scan comparison
        ↓ weighting
        ↓ resampling

     . . .
      ...
       .
       ↓

Converged robot pose
```

The particle cloud normally converges around the real position after the system receives enough useful measurements.

---

### 6.2.1 Localization Visualization

| Particles Scattered | Particles Converged |
|:---:|:---:|
| <img width="760" height="760" alt="Point Cloud scattered" src="https://github.com/user-attachments/assets/f5e9c17d-f9f0-44cf-8a63-f246ad602407" /> | <img width="760" height="760" alt="Point Cloud converged" src="https://github.com/user-attachments/assets/9c6c275c-b07b-4db6-acb4-3db6f86927ec" /> |

---

## 6.3 Field Map

`field_map.py` represents the known WRO geometry mathematically.

The map contains information such as:

- outer walls,
- centre block,
- legal start areas,
- and field boundaries.

The localizer can therefore evaluate whether a hypothetical pose agrees with the measured LiDAR scan.

---

## 6.4 Heading Fusion

The IMU provides a coarse orientation reference. LiDAR wall geometry provides a more precise orientation modulo the repeated 90° field geometry. The two sources are therefore combined to improve the robot’s overall orientation estimate.

The current implementation follows this concept:

```python
def _resolve_heading(self, scan):
    coarse = self.field_heading()
    fine, quality = self.scan_orientation(scan)

    if fine is None:
        return coarse

    delta = (angle_difference(fine, coarse) + 45.0) % 90.0 - 45.0

    return normalize_angle(coarse + delta)
```

The idea is:

```text
IMU
→ Which field quadrant are we facing?

LiDAR
→ Precise wall orientation inside that quadrant
```

Neither source alone provides exactly the same information.

---

## 6.5 Why the IMU Is Not Used as the Only Heading Source

The BNO055 is initialized relative to the robot's starting direction rather than relying on a fully calibrated geographic heading.

IMU measurements can also experience:

- drift,
- magnetic interference,
- and absolute-heading uncertainty.

The field geometry visible in the LiDAR therefore provides an additional heading reference.

---

## 6.6 Pose Confidence

Localization produces a confidence value between:

```text
0.0 and 1.0
```

The current documented threshold for a sufficiently trustworthy pose is approximately:

```text
0.35
```

Downstream behavior checks this confidence before making decisions that depend strongly on absolute field position.

This is important because:

> **A wrong position estimate that looks valid can be more dangerous than explicitly recognizing that the robot is temporarily uncertain.**

---

## 6.7 Motion Model

During the normal racing loop, the Raspberry Pi reports estimated movement to the localizer.

An important current limitation is:

> **The high-level Raspberry Pi localization does not yet use returned encoder counts as its primary odometry input.**

The Arduino already reads the encoder and can use it for finite distance-based movements. However, the Raspberry Pi path-following motion model currently estimates movement from commanded driving behavior. Integrating encoder telemetry directly into localization remains a future improvement.

---

## 6.8 Localization Pipeline

```text
Previous Pose
     |
Motor Motion Estimate
     |
     v
Particle Motion Update
     |
     +--------------------+
     |                    |
     v                    v
LiDAR Scan             Field Map
     |                    |
     +---------+----------+
               |
               v
          Scan Scoring
               |
               v
         Particle Weights
               |
               v
           Resampling
               |
               +---------- IMU Heading
               |
               v
      Pose + Confidence
```

---

# 7. Path Generation and Track Following

Once the robot knows approximately where it is, it needs a target path. The final system generates a complete loop before driving. This path is called the **racing line**.

---

## 7.1 Racing Line

The racing line is a rounded closed path positioned between the outer field wall, and the central block.

It is stored in two useful forms:

### 7.1.1 Mathematical geometry

Allows the software to evaluate properties such as:

- path curvature,
- distance along the path,
- and future path direction.

### 7.1.2 Sampled points

The path is also sampled approximately every:

```text
20 mm
```

This allows efficient calculations such as:

> How far along the lap is the robot?

---

## 7.2 Starting Direction

The robot does not require the blue or orange track line to determine its driving direction. During setup, the navigation system can select the path direction requiring the smaller initial rotation. This limits the required initial path alignment.

The intended result is that the initial heading difference is no greater than approximately:

```text
90°
```

---

## 7.3 Why Pure Pursuit?

The final path-following system uses Pure Pursuit instead of a steering PID based only on the robot’s offset from the wall. Pure Pursuit is a geometric path-tracking algorithm.

It asks:

> If the robot is here and the target path is there, what circular arc would connect the vehicle toward a point further ahead on that path?

<img width="579" height="189" alt="Pure Pursuit geometry" src="https://github.com/user-attachments/assets/97c75c68-0340-4ea8-b675-555a94420d5b" />

The main calculation is approximately:

```python
curvature = 2.0 * math.sin(math.radians(alpha)) / distance

road_wheel = math.degrees(
    math.atan(self.wheelbase_mm * curvature)
)

command = (
    road_wheel
    / self.max_road_wheel_deg
    * self.max_steer_command
)
```

Where:

- `alpha` is the angular difference to the lookahead target,
- `distance` is the distance to the target point,
- `wheelbase_mm` represents vehicle geometry,
- `road_wheel` is the required physical wheel angle.

---

## 7.4 Why Not Use Only a Wall-Offset PID?

A wall-following PID can be effective when the goal is simply to maintain a fixed distance from one wall.

Our final software already contains a geometric representation of:

- robot pose,
- field geometry,
- desired path,
- obstacle positions.

Pure Pursuit allows these components to work together within the same coordinate system. The controller therefore follows an explicit planned path instead of only reacting to the latest side-distance error. However, we do not claim that Pure Pursuit was experimentally proven to be better than every possible PID controller.

The choice was primarily architectural:

> Once a reliable field pose and target path existed, a geometric path-following algorithm matched the structure of the problem directly.

---

## 7.5 Lookahead Distance

Pure Pursuit depends heavily on the **lookahead distance**.

### Short Lookahead

Advantages:

- tighter tracking,
- faster response to path curvature.

Disadvantages:

- more sensitive to localization noise,
- more likely to oscillate at higher speed.

### Long Lookahead

Advantages:

- smoother steering,
- greater stability.

Disadvantages:

- increased corner cutting,
- slower correction to path deviation.

The software therefore scales lookahead with driving speed.

<img width="1000" height="720" alt="Pure Pursuit result" src="https://github.com/user-attachments/assets/3d993cf3-0534-49f1-8d91-8b340fb0c89d" />

---

## 7.6 Steering Rate Limiting

Even if a calculated steering target changes instantly, the physical servo cannot. The control system therefore limits how quickly the steering command can change. This is important because the measured steering actuator has non-zero response time.

A software controller that assumes instantaneous steering can produce:

- oscillation,
- overshoot,
- or unrealistic commands.

---

## 7.7 Speed Selection

The path controller does not always use one fixed motor command.

Speed selection can consider several limits, such as:

- cornering requirement,
- localization uncertainty,
- and LiDAR clearance.

Conceptually:

```text
Corner-Speed Limit
        |
Lost-Pose Speed Limit
        |
LiDAR Clearance Limit
        |
        v
Choose Safest / Lowest Required Speed
```

This allows driving behavior to become more conservative when the environmental or localization condition is less reliable.

---

# 8. Obstacle Detection and Traffic-Pillar Strategy

The Obstacle Challenge requires the robot to distinguish between:

- red traffic pillars,
- green traffic pillars.

The passing rule is:

```text
RED   → pass on its RIGHT
GREEN → pass on its LEFT
```

The final software uses the camera to identify pillar color and estimate its physical position.

---

## 8.1 Obstacle-Handling Pipeline

```text
Camera Frame
     |
     v
Color Conversion
     |
     v
HSV Threshold
     |
     v
Candidate Region
     |
     v
Bounding Rectangle
     |
     +----------> Pillar Color
     |
     +----------> Pixel Position
     |
     +----------> Apparent Height
                       |
                       v
                Distance Estimate
                       |
                       v
                 Camera Bearing
                       |
                       v
                 Robot Pose
                       |
                       v
                Field Coordinate
                       |
                       v
                   Block Map
                       |
                       v
              Racing-Line Offset
                       |
                       v
                 Pure Pursuit
```

---

## 8.2 Why HSV Instead of Raw RGB Thresholding?

Lighting intensity changes RGB values strongly. Hue provides a more useful representation for separating color from brightness. The final code therefore converts visual information to HSV and uses tuned color ranges.

Current configuration:

```python
COLOR_SPECS = {
    #                anchor RGB     hue_tol  saturation   value
    Color.GREEN:  ((70, 120, 60),        9,   (90, 230),  (20, 250)),
    Color.RED:    ((140, 30, 30),        6,  (125, 210),  (55, 200)),
}
```

The values are tuned using:

```text
test_color_picker.py
```

rather than selecting thresholds only from theoretical colors.

---

## 8.3 Distance from Apparent Pillar Height

The traffic pillars have a known physical height.

The current software uses:

```text
BOX_HEIGHT_CM = 10.0
```

The apparent pixel height can therefore be converted to an angular height and then to an estimated distance.

Current implementation concept:

```python
BOX_HEIGHT_CM = 10.0

DEG_PER_PX = (
    CAPTURED_HORIZONTAL_FOV_DEG
    / CAPTURED_FRAME_WIDTH_PX
)

angular_height_deg = max(rect[1]) * DEG_PER_PX

distance_cm = (
    (BOX_HEIGHT_CM / 2.0)
    / math.tan(math.radians(angular_height_deg / 2.0))
)

bearing_deg = self._horizontal_pixel_to_angle(center_x)
```


### 8.3.1 Camera FOV Verification

The current software uses an approximately:

```text
80° horizontal FOV calibration value
```

for this calculation.

The software constant should be treated as a **calibration parameter**, not automatically as the manufacturer's optical specification.

---

## 8.4 Converting a Detection to Field Coordinates

A camera detection by itself is temporary.

Once the camera looks away, the image no longer contains the pillar.

The software therefore combines:

```text
Robot Pose
+
Pillar Bearing
+
Pillar Distance
```

to calculate a pillar position in field coordinates.

That position is stored in the `block_map`.

This converts:

> **a temporary camera observation**

into:

> **a persistent environmental object**

---

## 8.5 Passing Side

The current side-selection logic is:

```python
SIDE_FOR_COLOR = {
    Color.GREEN: -1.0,
    Color.RED:   +1.0
}
```

where the sign represents lateral position relative to travel direction.

Conceptually:

```text
GREEN
→ move target path to pillar's left side

RED
→ move target path to pillar's right side
```

---

## 8.6 Required Clearance

The target path is not shifted by one arbitrary constant from the center line.

The required offset accounts for:

- requested clearance,
- robot half-width,
- optional color-specific padding,
- pillar half-width.

Current concept:

```python
def _required_offset_mm(self, color):
    return (
        self.setting("blocks.clearance_mm")
        + self.setting("blocks.robot_half_width_mm")
        + self.setting(extra_key)
        + BLOCK_SIZE_MM / 2.0
    )
```

Current documented values include approximately:

```text
Base clearance       = 140 mm
Robot half-width     = 160 mm
Additional red space = 80 mm
```

---

## 8.7 Smooth Path Deformation

The racing line does not jump instantly sideways around a detected obstacle. A sudden path discontinuity would create a large steering request. The obstacle offset therefore fades in and out smoothly.

Conceptually:

```text
Normal Racing Line
───────────────╮
               ╰──────╮
                      │ Pillar
               ╭──────╯
───────────────╯
Normal Racing Line
```

The current implementation uses a smoothstep-style transition.

Documented tuning includes approximately:

```text
Begin offset ~600 mm before pillar
Hold / continue until ~450 mm after pillar
```

The modified path is also clamped so that avoiding a traffic pillar cannot intentionally push the robot beyond the available corridor.

<img width="1345" height="576" alt="Obstacle path" src="https://github.com/user-attachments/assets/a84a6d0e-dd47-4796-9998-58df81fc10c8" />

---

## 8.8 Why Bend the Existing Racing Line?

An alternative architecture would be:

```text
Normal Driving Controller
        ↓ obstacle
Switch Controller
        ↓
Obstacle Avoidance Controller
        ↓
Switch Back
        ↓
Normal Controller
```

Our final architecture instead uses:

```text
Normal Racing Line
        ↓ obstacle
Modify Racing Line
        ↓
Same Pure Pursuit Controller
```

Advantages include:

- one steering-controller architecture,
- smoother transition into obstacle passing,
- less duplicated steering logic,
- easier reasoning about path behavior.

The trade-off is that obstacle position must be estimated accurately enough to modify the path in field coordinates.

---

## 8.9 Obstacle Edge Cases

The software includes explicit checks for unreliable detections.

### Low-confidence robot pose

If the robot's own pose is unreliable, the system should not place a camera detection into the permanent block map.

Reason:

> A pillar stored at the wrong field location can remain believable after the original bad camera frame has disappeared.

### Detection inside a field wall

If the transformed detection appears inside known wall geometry, it is rejected.

This can occur because of:

- reflections,
- inaccurate range estimate,
- incorrect pose,
- or false visual detection.

### LiDAR reports blocked path while camera reports no pillar

The LiDAR clearance information is treated as the higher-priority safety signal.

Camera absence is not proof that physical space is clear.

---

# 9. Parking Strategy

The current parking strategy uses a fixed, multi-step maneuver. The original software documentation identifies **four main parking steps**.

---

## 9.1 High-Level Parking Sequence

```text
PHASE 1
Position Robot
and Align Rear
      |
      v
PHASE 2
Turn + Reverse
into Parking Area
      |
      v
PHASE 3
Reverse Straight
      |
      v
PHASE 4
Final Reverse Turn
until Parallel
      |
      v
STOP
```

---

### Phase 1 — Initial Positioning

The robot moves to a position where the rear section is appropriately aligned with the parking area.

### Phase 2 — Reverse Entry

The steering is turned and the vehicle reverses into the parking space.

### Phase 3 — Straight Reverse

The steering is adjusted and the robot continues backwards.

### Phase 4 — Final Alignment

A final steering correction is performed while reversing so that the robot ends approximately parallel with the outer field wall.

---


# 10. Sensor Fusion and Information Arbitration

The robot uses several sensors, but it does not treat all measurements as equally appropriate for every task.

---

## 10.1 Sensor Responsibilities

| Sensor / Source | Trusted For | Not Primarily Used For |
|---|---|---|
| **RPLiDAR C1** | Position, wall geometry, precise field orientation, clearance | Pillar color |
| **BNO055 IMU** | Relative heading / field quadrant | Absolute position |
| **Camera** | Pillar color, bearing, apparent-size distance | Wall localization |
| **Motor command / motion model** | Estimated motion between localization updates | Absolute position |
| **Encoder** | Arduino-side motor rotation / finite-distance movement | Currently not primary Pi localization odometry |

---

## 10.2 Fused Heading

Heading is one genuinely fused quantity.

```text
IMU
   |
   | coarse direction / quadrant
   v
Heading Fusion
   ^
   | precise wall orientation
   |
LiDAR
```

The IMU provides the large-scale orientation reference, while LiDAR geometry refines it relative to the rectangular field.

---

## 10.3 Fused Traffic-Pillar Position

The other major fused result is the persistent block map.

```text
Camera
→ Color + Bearing + Range

Robot Localization
→ x + y + heading

Combined
→ Pillar field coordinate
```

Without the current robot pose, a camera detection is only a location relative to the camera. Combining the detection with localization allows the obstacle to become part of the field model.

---

## 10.4 When Sensors Disagree

The software uses explicit arbitration rules.

### Geometry

**LiDAR / field geometry wins.**

If the robot pose is not reliable enough, a camera detection is not inserted into the block map.

### Heading

The disagreement between LiDAR orientation and the IMU reference is folded into the expected 90° field symmetry.

### Camera says clear, LiDAR says blocked

**LiDAR wins.**

A missing camera detection does not prove that physical space is clear.

### Detection falls inside wall geometry

The detection is discarded.

---

## 10.5 Sensor-Fusion Visualization

<img width="1080" height="700" alt="Sensor Fusion" src="https://github.com/user-attachments/assets/7cc79c9f-e000-4f05-85a6-48cf61c505c4" />

---

# 11. Edge Cases, Safety and Failure Handling

## 11.1 Software Failure / Exception

```text
Unexpected Python Error
        |
        v
finally:
    finish()
        |
        v
Stop Motor
Center Steering
```

This prevents an exception from intentionally leaving the vehicle driving with the previous command.

---

## 11.2 Competition Timeout

If the task exceeds the configured:

```text
180 s
```

the loop exits and performs normal cleanup.

---

## 11.3 Low Localization Confidence

When localization confidence decreases, the path-driving system can switch to more conservative behavior. The software monitors whether the robot is effectively “lost” instead of assuming that every pose estimate is reliable. The exact speed response depends on the system configuration.

---

## 11.4 Invalid Pillar Position

If a pillar is detected while the robot's own position is unreliable:

```text
Bad Pose
+
Camera Detection
=
Do Not Store Detection
```

This avoids contaminating the persistent map with incorrect world coordinates.

---

## 11.5 Pillar Position Inside Wall

```text
Detected Pillar
      |
Transform to Field
      |
      v
Inside Known Wall?
   /        \
 Yes        No
  |          |
Drop       Store
```

---

## 11.6 Insufficient Obstacle Corridor

The current final-round configuration performs a startup feasibility check. The existing documentation notes that, under one configuration, a pillar placed directly on the racing line could require an offset greater than the available corridor. This condition is intentionally detected rather than being silently ignored.

---

## 11.7 IMU Unavailable

The navigation architecture is not intended to depend on one sensor alone.


---

## 11.8 LiDAR Unavailable

LiDAR is central to the localization architecture.

If LiDAR initialization fails, the robot cannot provide normal field localization.


---

## 11.9 Camera Unavailable

The Open Challenge does not require traffic-pillar color recognition in the same way as the Obstacle Challenge.

The final Obstacle Challenge behavior, however, depends on camera detection.

---

# 12. Testing, Simulation and Tuning

Testing is a central part of the software architecture.

The most important development tool this season is not physically mounted on the robot:

> **A simulation and test environment that runs the real navigation code without requiring the physical track for every change.**

---

## 12.1 Testing Without the Robot

Several scripts execute the real navigation modules against a simulated robot.

Examples:

```bash
uv run python test_navigation.py
```

Purpose:

- visualize localization convergence,
- test particle behavior,
- verify field geometry.

---

```bash
uv run python test_driving.py --trials 24
```

Purpose:

- run repeated driving trials,
- randomize starting placement,
- report pass / fail behavior,
- evaluate minimum clearance.

---

```bash
uv run python test_steering.py --sweep speed.corner 40,50,60,70
```

Purpose:

- compare parameter values,
- evaluate trade-offs,
- produce tuning tables.

---

## 12.2 Why Use Simulation?

Physical WRO track testing requires:

- full robot preparation,
- battery preparation,
- field setup,
- manual reset,
- observation,
- and significant time.

Simulation allows clearly incorrect configurations to be rejected before physical testing.

However, simulation cannot perfectly reproduce:

- glare,
- shadows,
- camera exposure,
- floor friction,
- wheel slip,
- dust,
- battery sag,
- cable movement,
- or mechanical wear.

Therefore:

> **Simulation reduces wasted track time; it does not replace real-world validation.**

---

## 12.3 Simulated Steering Lag

A simulator with instantaneous steering would make the controller appear unrealistically stable. The simulator therefore includes steering lag.

This helps reproduce the weaving behavior that can occur when:

```text
Controller Request
changes faster than
Physical Servo Response
```

---

## 12.4 Steering Calibration

Two steering characteristics cannot be reliably obtained only from the nominal servo command.

They must be measured on the real vehicle.

## 12.4.1 Maximum Steering Geometry

The steering-calibration tool drives arcs and estimates the real turning geometry.

Testing found that the actual robot turned approximately:

> **21% more than the original 40° full-lock assumption.**

This is important because Pure Pursuit converts required curvature into a physical steering model. An incorrect maximum steering assumption produces an incorrect command scale.


### 12.4.2 Steering Response Time

The steering-response tool measured approximately:

> **0.35 s to reach 63% of the steering response.**

This demonstrates that the actuator cannot be modeled as instantaneous. The software therefore accounts for physical steering behavior rather than treating the servo as an ideal mathematical device.

---

## 12.5 Why Integrate Steering Measurements Over a Run?

The `SteeringCalibrator` can estimate steering geometry during a normal lap.

It does not simply compare:

```text
Command at time t
vs.
Wheel behavior at time t
```

because the steering mechanism has delay.

The physical response belongs to an earlier command. Integrating the motion over a longer interval avoids directly comparing a cause with an effect that has not yet occurred.

---

## 12.6 Debug Visualization

The robot exposes several debugging tools.

### Status Output

A status line can include:

- elapsed time
- lap progress
- position
- path offset
- steering
- wheel angle
- speed
- localization confidence

Example:

```text
[ 12.4s] lap 0.87/3  (+102.4,-140.1)cm
off-line +1.2cm
steer=-8.3
wheel=-4.5deg
speed=70
conf=0.78
```

### 12.6.1 `--debug`

Displays a live top-down visualization containing:

- field
- particle cloud
- LiDAR points
- racing line
- obstacle-modified path
- robot pose
- Pure Pursuit target point


### 12.6.2 `--ascii`

Provides debugging information in a terminal-only environment such as SSH.


### 12.6.3 Object-Detection Debugging

The object solver provides visual debugging for:

- camera frame
- traffic-pillar bounding boxes
- detected color
- estimated position
- top-down radar visualization

### 12.6.4 Color Picker

```text
test_color_picker.py
```

allows a team member to select a real camera pixel and inspect its:

- RGB value,
- HSV value.

This makes threshold tuning traceable to real competition images.


### 12.6.5 Run Recording

The camera system can record processed video for later review.

Recording may be disabled during final competition execution when reducing processing overhead is more important.


### 12.6.6 Existing Debug View

<img width="1120" height="560" alt="Debug View" src="https://github.com/user-attachments/assets/64f29e14-323e-4c5f-b2cd-3190da0c62e1" />

---

## 12.7 Testing Workflow

```text
Code / Config Change
        |
        v
Static / Dry-Run Check
        |
        v
Simulation
        |
        v
Debug Visualization
        |
        v
Parameter Sweep
        |
        v
Physical Robot Test
        |
        v
Observe Failure
        |
        v
Adjust Config / Algorithm
        |
        +------------------+
        |                  |
        └------ Repeat <---+
```

---

# 13. Performance Metrics and Validation

Testing should produce measurable results that can be compared. The simulator and debugging tools already provide several useful metrics.

---

## 13.1 Existing Software Metrics

| Metric | Purpose |
|---|---|
| Localization confidence | Determine whether pose should be trusted |
| Minimum wall / obstacle clearance | Detect unsafe path behavior |
| Lap progress | Evaluate completion |
| Off-line distance | Evaluate path tracking |
| Steering command | Observe controller behavior |
| Physical wheel-angle estimate | Compare command vs robot geometry |
| Speed command | Evaluate driving profile |
| Localization convergence | Evaluate particle filter |
| Simulation pass / fail | Compare configuration choices |

---

## 13.2 Structured Software Test Results

Known measured results currently include:

| Test | Method | Result | Engineering Effect |
|---|---|---|---|
| Steering full-lock geometry | Arc / steering calibration | ~21% greater turn than original 40° assumption | Update steering model |
| Steering response | Lag / step-response test | ~0.35 s to 63% response | Account for actuator dynamics |
| Localization | Particle-filter simulation | Converges from scattered hypotheses | Used as final localization strategy |
| Color calibration | Real-image HSV picker | Tuned red / green ranges | Used by object detector |

---

# 14. Software Development and Engineering Decisions

The final architecture is the result of several software-level decisions.

---

## 14.1 Camera-Reactive Concept → Localization-Based Navigation

### Earlier Concept

```text
Camera / Local Sensors
        |
        v
React to Current Environment
```

### Final Concept

```text
LiDAR + IMU
      |
      v
Localization
      |
      v
Field Pose
      |
      v
Planned Racing Line
      |
      v
Pure Pursuit
```

### Reason

A localization-based system can reason about:

- where the vehicle is,
- where the path should be,
- where known obstacles are,

even when those objects are not visible in the current camera frame.

### Trade-off

Localization requires:

- more computation,
- a field model,
- and more complex software

than a simple reactive controller.

---

## 14.2 LiDAR + IMU Heading Fusion

**Alternative:** Use only IMU heading.

**Final Decision:** Use IMU for coarse orientation and LiDAR geometry for precise field-relative orientation.

**Reason:** The field itself provides strong geometric information.

**Trade-off:** LiDAR-based heading depends on sufficient visible wall geometry and correct scan alignment.

---

## 14.3 Persistent Block Map

**Alternative:** React only while a pillar is currently visible.

**Final Decision:** Transform detections into field coordinates and retain them.

**Reason:** The robot continues to know about a pillar after the camera turns away.

**Trade-off:** A bad robot pose can place the obstacle incorrectly.

**Mitigation:** Do not store detections when localization confidence is insufficient.

---

## 14.4 One Controller for Normal Driving and Obstacle Passing

**Alternative:** Separate obstacle-steering controller.

**Final Decision:** Modify the path and retain Pure Pursuit.

**Reason:** This avoids duplicated steering logic and provides smoother transitions.

**Trade-off:** Obstacle mapping must be accurate.

---

## 14.5 Slower Camera Cadence

**Alternative:** Run computer vision every path-control tick.

**Final Decision:** Run camera processing less frequently.

**Reason:** Image processing is significantly more expensive than one steering-control update.

**Trade-off:** Visual information updates less frequently.

**Mitigation:** The persistent block map retains detected obstacles between frames.

---

## 14.6 Configuration Files Instead of Hard-Coded Tuning

**Alternative:** Edit Python source at the field.

**Final Decision:** Keep tuning values in:

```text
config.toml
```

**Reason:** 

- faster adjustment,
- reduced chance of damaging algorithm logic,
- easier comparison between settings,
- more reproducible competition configuration.

---

## 14.7 Simulation Before Track Testing

**Alternative:** Test every parameter change physically.

**Final Decision:** Use simulator / test scripts first.

**Reason:** Reduces time spent on configurations that are already clearly unstable.

**Trade-off:** Simulation cannot perfectly reproduce the real robot.

---

## 14.8 Commanded-Motion Odometry vs Encoder Odometry

**Current Architecture:** The high-level motion model estimates distance primarily from commanded driving behavior.

**Available Hardware:** The motor already includes an encoder.

**Trade-off:** Command-based odometry is simpler but does not directly measure:

- wheel slip,
- load variation,
- battery-related speed change.

Encoder odometry should provide a more direct motion estimate once integrated into the Raspberry Pi localization pipeline.

---

## 14.9 Software Decision Summary

| Decision | Alternative | Main Benefit | Trade-off |
|---|---|---|---|
| Localization-based navigation | Camera-only reactive driving | Field awareness | More computation |
| Particle filter | Single deterministic pose estimate | Represents localization uncertainty | More CPU usage |
| Pure Pursuit | Pure wall-offset steering | Geometric path tracking | Requires good localization |
| Persistent block map | Frame-only obstacle response | Pillars remain known after leaving camera view | Depends on pose accuracy |
| HSV detection | Raw RGB thresholding | More robust to brightness change | Still affected by lighting / exposure |
| Path deformation | Separate obstacle controller | Reuse same steering architecture | Requires mapped obstacle |
| Slower camera cadence | Vision every tick | Lower processing load | Slower visual updates |
| TOML configuration | Hard-coded constants | Easier tuning | Configuration must be managed carefully |
| Simulation | Physical testing only | Fast repeatable iteration | Model is imperfect |
| Relative IMU heading + LiDAR | IMU-only heading | Less dependence on magnetometer absolute heading | Requires usable LiDAR geometry |

---

# 15. Known Limitations and Future Improvements

## 15.1 Encoder Odometry

### Current Limitation

The Pi localization motion model currently estimates movement from drive commands instead of directly receiving wheel-encoder telemetry.

### Improvement

Send encoder delta / velocity information from Arduino to Raspberry Pi and integrate it into the particle-filter motion model.

This could improve movement estimation under:

- wheel slip,
- battery-voltage changes,
- mechanical load.

---

## 15.2 Camera Exposure

### Current Limitation

Automatic exposure can change the apparent color of traffic pillars during a run.

### Improvement

Lock or control camera exposure so that the same physical pillar produces more consistent image values. This may improve color-detection repeatability without simply widening HSV thresholds.

---

## 15.3 Camera FOV Calibration

### Current Limitation

Current electrical and software documentation contain different camera-FOV values.

### Improvement

Measure the final camera's effective horizontal field of view and use that calibration consistently for bearing / distance calculations.

---

## 15.4 Parking Validation

### Current Limitation

The parking documentation currently contains the maneuver sequence but does not yet provide complete quantitative repeatability evidence or exact state transition documentation.

### Improvement

Document:

- exact state conditions
- exact exit conditions
- distance values
- steering commands
- parking success rate

---

## 15.5 Final-Round Corridor Feasibility

The current software contains a validation check that can warn if the requested traffic-pillar clearance is physically larger than the available corridor. This is useful because it identifies an impossible configuration before the robot attempts it.

---

# 16. Running and Reproducing the Software

## 16.1 Raspberry Pi Requirements

The current software targets:

```text
Raspberry Pi 5
Python 3.11+
```

Python dependencies are managed using:

```text
uv
```

---

## 16.2 Install Dependencies

From the Raspberry Pi software directory:

```bash
cd src/Raspberrypi
uv sync
```

The repository contains:

```text
pyproject.toml
uv.lock
```

so that package versions can be reproduced.

---

## 16.3 Run Open Challenge

```bash
uv run python main.py qualification
```

---

## 16.4 Run Obstacle Challenge

```bash
uv run python main.py final
```

Debug mode:

```bash
uv run python main.py final --debug
```

---

## 16.5 Dry Run

A dry run can inspect the planned task without accessing the hardware.

```bash
uv run python main.py qualification --dry-run
```

This is useful for checking configuration and task initialization before operating the physical robot.

---

## 16.6 Arduino Setup

Open:

```text
src/Arduino/Main.ino
```

using the Arduino IDE.

Target board:

```text
Arduino UNO R4 Minima
```

Required libraries include:

```text
Servo
PID_v2
```

The repository currently contains copies / versions of the required libraries.

---

## 16.7 Software Verification Sequence

Before a complete autonomous test:

```text
1. Arduino Upload
        |
        v
2. Serial Connection Test
        |
        v
3. Motor Direction Test
        |
        v
4. Steering Center Test
        |
        v
5. Encoder Test
        |
        v
6. LiDAR Test
        |
        v
7. IMU Test
        |
        v
8. Camera / Color Test
        |
        v
9. Localization Test
        |
        v
10. Dry Run
        |
        v
11. Low-Speed Autonomous Test
        |
        v
12. Full Competition Test
```

Detailed hardware setup is documented in:

[`../BUILD.md`](../BUILD.md)

---

## 16.8 Test Commands

Examples:

```bash
uv run python test_navigation.py
```

```bash
uv run python test_driving.py --trials 24
```

```bash
uv run python test_steering.py --sweep speed.corner 40,50,60,70
```

```bash
uv run python test_color_picker.py
```

These tools are part of the engineering workflow rather than unrelated example programs.

---

## 16.9 Code Documentation Philosophy

Every important module should explain **why it exists**, not only what functions it contains.

Non-obvious constants should explain where their values came from.

Example:

```python
# Real-world HEIGHT of the boxes. The WRO traffic signs are 5cm x 5cm on the
# floor and 10cm tall - this is the 10, not the 5. Distance scales linearly
# with it, so the 5.0 that used to be here reported every pillar at half its
# true range.
BOX_HEIGHT_CM = 10.0
```

---

# 17. Final Software Architecture

The final competition software can be summarized as:

```text
                       COMPETITION START
                              |
                              v
                      Initialize Systems
                              |
                              v
                   Capture Heading Reference
                              |
                              v
                         Build Path
                              |
                              v
                       LOCALIZATION
                  LiDAR + IMU + Motion
                              |
                              v
                      Pose + Confidence
                              |
          +-------------------+-------------------+
          |                                       |
          v                                       v
     Racing Line                           Camera Detection
          |                                       |
          |                                  Red / Green
          |                                       |
          |                                  Block Map
          |                                       |
          +-------------------+-------------------+
                              |
                              v
                       Modified Path
                              |
                              v
                         Pure Pursuit
                              |
                              v
                        Speed Selection
                              |
                              v
                       Steering + Speed
                              |
                         USB Serial
                              |
                              v
                      Arduino UNO R4
                         |       |
                         v       v
                      Motor    Servo
                         |
                      Encoder
```

---

## 17.1 Final Competition Software Components

### High-Level Competition Control

- `BaseTask`
- `PathTask`
- Qualification task
- Final / obstacle task

### Localization

- `NavigationManager`
- `FieldMap`
- Particle filter
- LiDAR scan matching
- IMU heading fusion
- pose confidence

### Path Following

- `RacingLine`
- `PurePursuit`
- steering-rate limiting
- adaptive speed selection

### Obstacle Management

- camera image processing
- HSV traffic-pillar detection
- apparent-height range estimation
- camera bearing
- field-coordinate conversion
- persistent block map
- racing-line deformation
- correct red / green passing side

### Parking

- multi-step reverse parking sequence

### Low-Level Control

- Arduino serial protocol
- motor control
- steering servo
- encoder
- start button

### Development Tools

- navigation simulator
- driving trials
- steering calibration
- parameter sweeps
- color picker
- live debug visualization
- dry-run mode

---

# Final Software Summary

The YBR-SUNFLOWER software evolved from a more immediately reactive sensing concept into a field-localization and path-following architecture.

The final development path can be summarized as:

```text
Reactive Perception Concept
          |
          v
Need Better Field Awareness
          |
          v
LiDAR-Based Localization
          |
          v
Robot Pose (x, y, heading)
          |
          v
Geometric Racing Line
          |
          v
Pure Pursuit
          |
          +-----------------------+
          |                       |
          |                    Camera
          |                       |
          |                 Pillar Mapping
          |                       |
          +-----------+-----------+
                      |
                      v
               Modified Racing Line
                      |
                      v
                Pure Pursuit
                      |
                      v
               Autonomous Driving
```

The important engineering decisions were not limited to choosing algorithms.

They also included:

- separating high-level and low-level control,
- assigning each sensor a specific information role,
- maintaining localization confidence,
- rejecting unreliable obstacle observations,
- preserving detected pillars in field coordinates,
- reducing camera-processing frequency,
- measuring actual steering behavior,
- simulating the real navigation system before track testing,
- moving tuning parameters into configuration files,
- and defining cleanup behavior for unexpected software termination.

The software engineering process therefore follows:

> **Design → Simulate → Measure → Test → Identify Failure → Tune → Validate**

rather than treating the final source code as the only evidence of software development.
