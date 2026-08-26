# Electrical & Sensor System

This document describes the electrical architecture, power system, controller interfaces, sensors, wiring, calibration, testing, development history, and engineering decisions of our WRO Future Engineers 2026 robot.

The purpose of this documentation is not only to describe which components are used, but also to explain:

* why each component was selected,
* how the electrical and sensing systems are integrated,
* how the Raspberry Pi and Arduino communicate,
* how the Raspberry Pi I/O Expansion HAT is used as an interface layer,
* how the sensors are positioned and initialized,
* what problems were identified during development,
* how the electrical and sensing architecture evolved,
* and how the final system can be reproduced.

---

# 1. Electrical Hardware Overview

## 1.1 Final Hardware Layout

<img width="700" height="600" alt="Base Plate View" src="../other/ComponentsImage1.png" />

<img width="700" height="600" alt="Right View" src="../other/ComponentsImage2.png" />

<img width="700" height="600" alt="Left View" src="../other/ComponentsImage3.png" />

<img width="700" height="600" alt="Front View 1" src="../other/ComponentsImage4.png" />

<img width="700" height="600" alt="Front View 2" src="../other/ComponentsImage5.png" />

The annotated views show the main electrical, control, and sensing components installed on the final robot, including:

* Raspberry Pi 5
* DFR0566 IO Expansion HAT for Raspberry Pi
* Arduino UNO R4 Minima
* RPLiDAR C1
* BNO055 IMU
* Raspberry Pi Night Vision Camera
* CHP-20GP-180 drive motor with encoder
* L298P Motor Shield
* GEEKSERVO steering servo
* Helix 1100 mAh 11.1 V 3S LiPo battery
* LM2596 step-down converter
* XL4015 step-down converter
* D1-2 positive power distribution
* PCT-21 negative / ground distribution
* SPST ON/OFF switch
* ZX-Switch01 competition start switch

The electrical system is divided into four main functional groups:

1. **High-level computing** — Raspberry Pi 5
2. **Low-level control** — Arduino UNO R4 Minima
3. **Sensing** — Camera, LiDAR, BNO055 IMU, and encoder feedback
4. **Power and actuation** — Battery, converters, motor driver, drive motor, steering servo, and supporting interface electronics

The Raspberry Pi performs high-level perception, localization, navigation, and driving decisions. The DFR0566 IO Expansion HAT provides an organized interface layer for Raspberry Pi-side GPIO and peripheral connections. The Arduino handles low-level actuator control, encoder feedback, and the physical start switch.

---

## 1.2 Hardware Roles

| Component                        | Role                                                            |
| -------------------------------- | --------------------------------------------------------------- |
| Raspberry Pi 5                   | High-level processing, perception, localization, and navigation |
| DFR0566 IO Expansion HAT         | Raspberry Pi I/O expansion and peripheral interface layer       |
| Arduino UNO R4 Minima            | Low-level motor, steering, encoder, and start-button control    |
| RPLiDAR C1                       | 2D distance and environmental sensing                           |
| Raspberry Pi Night Vision Camera | Visual sensing and field-feature detection                      |
| Gravity BNO055 IMU               | Relative heading and orientation reference                      |
| CHP-20GP-180                     | Drive motor with integrated encoder                             |
| L298P Motor Shield               | Drive motor control                                             |
| GEEKSERVO                        | Steering control                                                |
| Helix 1100 mAh 11.1 V 3S LiPo    | Main power source                                               |
| LM2596                           | Raspberry Pi power conversion                                   |
| XL4015                           | Motor/control power conversion                                  |
| D1-2                             | Positive power distribution                                     |
| PCT-21                           | Negative / ground distribution                                  |
| SPST ON/OFF Switch               | Main power control                                              |
| ZX-Switch01                      | Competition start switch                                        |

---

# 2. Power System

## 2.1 Battery

| Specification         | Value               |
| --------------------- | ------------------- |
| Model                 | Helix 1100 mAh LiPo |
| Configuration         | 3S                  |
| Nominal Voltage       | 11.1 V              |
| Fully Charged Voltage | 12.6 V              |
| Capacity              | 1100 mAh            |

The robot is powered by a single 3S LiPo battery with a nominal voltage of 11.1 V.

The 11.1 V value is the nominal voltage rather than a constant output voltage. A fully charged 3S LiPo reaches approximately 12.6 V, and its voltage decreases during operation.

The battery capacity is 1100 mAh. Actual operating time depends on motor load, steering activity, sensor usage, and Raspberry Pi workload.

---

## 2.2 Power Distribution Architecture

The battery output is divided into two main regulated power branches.

### Computing Branch

```text
3S LiPo Battery
      |
      v
    LM2596
      |
   ~5.1 V
      |
 Raspberry Pi
      |
      v
DFR0566 IO Expansion HAT
```

### Motor / Control Branch

```text
3S LiPo Battery
      |
      v
    XL4015
      |
    ~5 V
      |
 Arduino / Control Electronics
```

The two branches are separated to reduce the effect of rapid motor-current changes on the Raspberry Pi and other sensitive computing electronics.

The DFR0566 is powered as part of the Raspberry Pi-side electronics and provides the physical interface for the Raspberry Pi's peripheral connections.

The power distribution uses:

* **D1-2** for positive power distribution
* **PCT-21** for negative / common-ground distribution

All subsystems share a common electrical reference through the ground connection.

---

## 2.3 Voltage Conversion

| Converter | Input                                            | Output              | Main Load              |
| --------- | ------------------------------------------------ | ------------------- | ---------------------- |
| LM2596    | Approximately 9.6–12.6 V during normal operation | Approximately 5.1 V | Raspberry Pi branch    |
| XL4015    | Approximately 9.6–12.6 V during normal operation | Approximately 5 V   | Motor / control branch |

The LM2596 is adjusted to approximately 5.1 V for the Raspberry Pi and its associated Pi-side interface electronics.

The XL4015 provides the regulated low-voltage supply used by the motor/control branch according to the final wiring design.

The converters protect the low-voltage electronics from the much higher and variable LiPo battery voltage.

---

## 2.4 Power Budget

| Component                |                       Voltage |    Typical / Reference Current | Notes                                                  |
| ------------------------ | ----------------------------: | -----------------------------: | ------------------------------------------------------ |
| Raspberry Pi 5           |                           5 V |                 Load dependent | Power system designed with up to 5 A supply capability |
| Camera                   |                           5 V | Approximately 250 mA reference | Load dependent                                         |
| RPLiDAR C1               |                           5 V | Approximately 290 mA reference | Load dependent                                         |
| BNO055 IMU               |                       3.3–5 V |   Approximately 5 mA reference | DFRobot SEN0253 module                                 |
| Arduino UNO R4 Minima    |                           5 V |                 Load dependent | Depends on connected hardware                          |
| DFR0566 IO Expansion HAT |                           5 V |                 Load dependent | Pi-side interface electronics                          |
| Drive Motor              | Motor configuration dependent |                 Load dependent | Measure experimentally                                 |
| Steering Servo           |           Approximately 4.8 V |  Approximately 70 mA reference | Can reach approximately 0.8–0.9 A at stall             |

The values above are reference values rather than exact measurements of the completed robot.

The drive motor is particularly load-dependent. Current can increase significantly during acceleration, high mechanical load, or near-stall conditions.

For final validation, the team should measure the actual current and voltage of the completed robot under representative operating conditions.

---

## 2.5 Power Reliability

The main power-reliability decisions are:

* Use one LiPo battery as the main power source.
* Regulate the battery voltage before supplying low-voltage electronics.
* Separate the computing branch from the motor/control branch.
* Adjust the Raspberry Pi supply to approximately 5.1 V.
* Use the DFR0566 to organize Raspberry Pi-side peripheral connections.
* Use dedicated power and ground distribution.
* Verify converter output voltage under maximum expected load.

This architecture is intended to reduce the possibility of computing instability caused by rapid changes in actuator current.

---

# 3. Controllers and Communication

## 3.1 Raspberry Pi 5

**Purpose:** High-level computing and navigation.

The Raspberry Pi 5 processes information from the camera, LiDAR, and BNO055 IMU.

Its main responsibilities include:

* Sensor processing
* Visual processing
* LiDAR-based environmental sensing
* Localization
* Navigation and decision-making
* Generating steering and drive commands
* Communicating commands to the Arduino

The Raspberry Pi's GPIO and peripheral interfaces are organized through the DFR0566 IO Expansion HAT.

---

## 3.2 DFR0566 IO Expansion HAT

**Purpose:** Raspberry Pi I/O expansion and peripheral interface.

The **DFR0566 IO Expansion HAT for Raspberry Pi** acts as the main interface layer around the Raspberry Pi.

The board exposes Raspberry Pi GPIO functions and provides convenient access to:

* Digital I/O
* Analog input
* PWM
* I²C
* UART
* SPI
* IIS

It is also compatible with DFRobot Gravity-style connections, which simplifies the physical connection of supported sensors and modules. DFRobot specifies a 5 V operating voltage and a 65 × 56 mm board size. The manufacturer documentation also specifies the I²C, UART, SPI, PWM, digital, and analog interfaces provided by the board. ([DFRobot Product](https://www.dfrobot.com/product-1930.html), [DFRobot Wiki](https://wiki.dfrobot.com/dfr0566/docs/22892))

In our robot, the HAT is used to organize the Raspberry Pi-side peripheral connections and reduce direct wiring to the Raspberry Pi GPIO header.

The camera and LiDAR use their dedicated Raspberry Pi interfaces, while the HAT provides the structured interface for the Raspberry Pi's other required peripheral connections.

This interface layer is important for reproducibility because the electrical system can be reproduced using the same expansion board, connector layout, and interface structure rather than requiring undocumented direct GPIO wiring.

---

## 3.3 Arduino UNO R4 Minima

**Purpose:** Low-level real-time control.

The Arduino UNO R4 Minima handles:

* Drive motor control
* Steering servo control
* Quadrature encoder feedback
* Competition start switch
* Low-level actuator commands received from the Raspberry Pi

This separation allows the Raspberry Pi to focus on high-level decisions while the Arduino performs deterministic low-level actuator control.

---

## 3.4 Raspberry Pi ↔ Arduino Communication

The Raspberry Pi communicates with the Arduino through **USB Serial**.

### Serial Configuration

| Parameter              | Value                             |
| ---------------------- | --------------------------------- |
| Physical connection    | USB                               |
| Device on Raspberry Pi | `/dev/ttyACM0` by default         |
| Baud rate              | `115200`                          |
| Protocol               | Newline-terminated ASCII          |
| Message format         | `<servoAngle>,<speed>,<distance>` |

The Raspberry Pi sends one command per line.

Example:

```text
30,55,0
```

This command means:

* steering angle = `30°`
* motor speed = `55`
* motor distance = `0`

---

## 3.5 Serial Command Protocol

The command format is:

```text
<servoAngle>,<speed>,<distance>\n
```

### Parameters

| Parameter    | Meaning                                 |
| ------------ | --------------------------------------- |
| `servoAngle` | Steering request in degrees             |
| `speed`      | Signed motor PWM command                |
| `distance`   | Requested motor-shaft travel in degrees |

### Steering Angle

The Raspberry Pi uses a centered steering convention:

```text
Negative = Left
0        = Center
Positive = Right
```

The Arduino converts the requested steering angle to the servo's usable range before applying it.

The software currently limits steering requests to approximately:

```text
-50° to +50°
```

---

## 3.6 Motor Speed

The wire protocol accepts a signed PWM command.

```text
Positive value = Forward
Negative value = Reverse
0              = Stop
```

The Arduino accepts values within the PWM range:

```text
-255 to +255
```

The Raspberry Pi navigation code typically uses smaller command values for normal driving.

---

## 3.7 Motor Distance

The `distance` field is **not an absolute motor position**.

It represents the requested amount of **motor-shaft rotation relative to the encoder position when the command starts**.

For example:

```text
30,50,360
```

means:

1. Set steering to approximately `30°`.
2. Drive at the requested speed.
3. Travel approximately `360°` of motor-shaft rotation from the encoder position at the beginning of the command.
4. Stop the motor when the requested travel is reached.

A distance of:

```text
0
```

means continuous driving until another command changes or stops the motor.

The encoder provides the feedback used to determine how far the motor shaft has moved.

---

## 3.8 Arduino Response Messages

The Arduino returns simple status messages.

| Response | Meaning                                              |
| -------- | ---------------------------------------------------- |
| `READY`  | Arduino initialized                                  |
| `Start`  | Competition start switch has been triggered          |
| `OK`     | Continuous-drive command accepted                    |
| `t`      | Requested non-zero motor-distance movement completed |
| `ERR`    | Malformed command                                    |

The Raspberry Pi waits for the appropriate response when using operations that require completion confirmation, such as distance-based movement or the start condition.

---

# 4. Interface and Pin Assignment

## 4.1 Arduino Pin Assignment

The final Arduino-side interface is:

| Function                 | Arduino Pin |
| ------------------------ | ----------- |
| Encoder A                | D2          |
| Encoder B                | D3          |
| Motor PWM                | D11         |
| Motor Direction          | D13         |
| Steering Servo           | D9          |
| Competition Start Switch | A0          |
| USB Serial               | USB         |

The encoder uses quadrature decoding through interrupts on both encoder channels.

The drive motor is controlled using:

* one PWM output for motor power,
* one digital output for direction.

---

## 4.2 Raspberry Pi Device Interfaces

| Device                | Interface                        | Default Device / Bus                |
| --------------------- | -------------------------------- | ----------------------------------- |
| Arduino UNO R4 Minima | USB Serial                       | `/dev/ttyACM0`                      |
| RPLiDAR C1            | USB Serial                       | `/dev/ttyUSB0`                      |
| BNO055                | I²C                              | Raspberry Pi hardware I²C / DFR0566 |
| DFR0566 HAT           | GPIO / I²C / peripheral breakout | Raspberry Pi header                 |
| Camera                | CSI                              | Raspberry Pi camera interface       |

The software performs device detection using USB identification hints to distinguish the Arduino from the LiDAR adapter.

---

# 5. Sensors and Hardware Placement

## 5.1 Camera — Raspberry Pi Night Vision Camera

<img width="213" height="213" alt="Camera" src="https://github.com/user-attachments/assets/d7751474-46d0-48a2-b5eb-e41289d2c9b4" />

**Purpose:** Visual sensing.

The camera provides visual information used by the robot's perception and navigation system.

### Placement

The camera is mounted on the upper section of the robot.

### Reason for Placement

The upper position provides a wider view of the field and reduces obstruction from the robot's own structure.

### Development Problem

The original camera lens had a relatively narrow field of view, limiting the visible area.

### Final Solution

The original lens was replaced with a wider-angle lens providing an approximately:

```text
60° FOV
```

This increased the visible field area available to the visual-processing system.

---

## 5.2 Lidar - RPLiDAR C1

<img width="358" height="355" alt="LiDAR" src="https://github.com/user-attachments/assets/04993f0c-df40-4ea0-91fa-db8690b847c1" />

**Purpose:** 2D distance and environmental sensing.

The RPLiDAR C1 provides distance measurements around the robot.

The information is used for:

* Wall detection
* Environmental awareness
* Navigation
* Obstacle / boundary detection
* LiDAR-based localization

### Interface

```text
USB
/dev/ttyUSB0
460800 baud
```

The software stores a rolling 360-degree distance representation, with one value for each whole-degree direction.

The scan is a rolling snapshot rather than one perfectly synchronized 360-degree sweep.

### Placement

The LiDAR is mounted at the front section of the robot.

### Development Problem

The previous robot had the LiDAR mounted at an excessive angle relative to the ground.

Because the LiDAR produces a 2D scan, this caused the surrounding environment to be represented incorrectly and distorted the resulting 2D map.

### Final Solution

The LiDAR was repositioned to be more parallel to the ground.

This significantly improved the quality of the 2D environmental representation.

---

## 5.3 BNO055 IMU - Gravity 10 DOF IMU AHRS (BNO055 + BMP280)

<img width="282" height="239" alt="IMU" src="https://github.com/user-attachments/assets/903cb715-8ac6-4622-b98d-e1724477d0ca" />

**Purpose:** Relative heading and orientation reference.

The robot uses the **DFRobot Gravity: 10 DOF IMU AHRS BNO055 + BMP280 (SEN0253)**.

The BNO055 provides fused orientation information from the accelerometer, gyroscope, and magnetometer. DFRobot states that the module provides fused outputs such as quaternions, Euler angles, rotation vector, linear acceleration, gravity, and heading. ([DFRobot Product](https://www.dfrobot.com/product-1793.html))

### Interface

The BNO055 is connected to the Raspberry Pi through I²C using the system's SDA and SCL lines. In the final electrical architecture, the Raspberry Pi-side I²C connection is exposed through the DFR0566 interface layer.

### Placement

The BNO055 is mounted **underneath the RPLiDAR**.

### Reason for Placement

In the previous robot, the IMU was mounted beside the Raspberry Pi I/O Expansion HAT.

This occupied useful space around the electronics and made the wiring more crowded.

In the redesigned robot, the IMU was moved underneath the LiDAR to:

* improve mechanical layout,
* reduce wiring congestion,
* use the available space more efficiently,
* and create a cleaner final electrical arrangement.

### Heading Strategy

The navigation system does not require an absolute heading.

Instead:

```text
Initial IMU Heading = Local 0° Reference
```

After startup, the robot records the current heading as `initial_heading`.

Subsequent turns are calculated relative to that reference.

For example:

```text
Initial heading = 37°
90° clockwise turn
Target heading = approximately 127°
```

The numerical absolute heading is therefore less important than the change in heading relative to the start orientation.

### Development Problem

A full BNO055 magnetometer calibration before every competition run would require moving the robot through multiple orientations and waiting for calibration status to reach the required level.

This is impractical in the competition start area and is unnecessary for the robot's navigation strategy.

### Final Solution

The robot performs a short startup settling period:

```text
BNO055 Connected
       |
       v
Wait approximately 1 second
       |
       v
Wait for competition start
       |
       v
Capture current heading
       |
       v
Initial heading = local 0°
```

The code uses approximately:

```text
BOOT_SETTLE_S = 1.0 s
```

The system also applies a compass offset and sign correction to match the robot's physical orientation and field coordinate system.

### Limitation

Because the robot does not perform a complete magnetometer calibration before each run, the heading may drift over longer runs.

For this reason, the IMU is not treated as the only navigation source.

The navigation system can use LiDAR-based localization together with the IMU heading reference.

---

## 5.4 Encoder - CHP-20GP-180 DC 12V (Gear Ratio 1:19)™

<img width="299" height="280" alt="Motor Encoder" src="https://github.com/user-attachments/assets/8a20d44b-b5ab-425f-985f-b26f82f73660" />

**Purpose:** Drive-motor movement feedback.

The CHP-20GP-180 includes a dual-channel encoder.

The Arduino reads the encoder using quadrature decoding.

### Encoder Configuration

```text
Gear ratio = 19:1
Encoder PPR = 11
Quadrature decoding = x4
```

The software therefore uses:

```text
Ticks per motor-shaft revolution
= 19 × 11 × 4
= 836 ticks/revolution
```

The encoder allows the system to estimate motor-shaft rotation rather than relying only on commanded PWM.

This is particularly important for distance-based movement.

---

## 5.5 DFR0566 IO Expansion HAT for Raspberry Pi

<img width="500" alt="DFR0566 IO Expansion HAT" src="https://www.dfrobot.com/product-1930.html" />

**Purpose:** Raspberry Pi I/O expansion and peripheral interfacing.

The DFR0566 provides an interface layer between the Raspberry Pi and its external GPIO-based and peripheral connections.

The board exposes:

* Digital I/O
* Analog input
* PWM
* I²C
* UART
* SPI
* IIS

and supports DFRobot Gravity-compatible connections. ([DFRobot Product](https://www.dfrobot.com/product-1930.html), [DFRobot Wiki](https://wiki.dfrobot.com/dfr0566/docs/22892))

### Placement

The DFR0566 is mounted directly on the Raspberry Pi 5.

### Reason for Use

The HAT was selected to:

* simplify Raspberry Pi-side wiring,
* provide convenient peripheral interfaces,
* reduce direct wiring to the Raspberry Pi header,
* improve organization of the electrical system,
* and make the final wiring easier to reproduce.

### Development Relationship with the IMU

In the earlier robot, the BNO055 was mounted beside the I/O Expansion HAT.

Although this arrangement was functional, it crowded the electronics area.

During the redesign, the BNO055 was moved underneath the LiDAR while the DFR0566 remained in its role as the Raspberry Pi-side interface layer.

This allowed the final system to retain the HAT's interface benefits while improving physical organization.

---

## 5.6 Start / Touch Sensor - ZX-Switch01 by INEX

<img width="234" height="231" alt="Touch Sensor" src="https://github.com/user-attachments/assets/dbe4f1f2-d705-40a1-bc5b-98a68d8a5cdf" />

**Purpose:** Competition start control.

The ZX-Switch01 is used as the physical competition start switch.

It is connected to:

```text
Arduino A0
```

The Arduino waits for the switch condition before allowing the competition sequence to begin.

---

# 6. Sensor Selection and Trade-offs

## 6.1 Selection Criteria

The final sensing architecture was designed to provide:

* Visual information
* Distance information
* Heading information
* Motor movement feedback

No single sensor provides all of these capabilities, so the system combines multiple sensing modalities.

---

## 6.2 Camera

The camera was selected because visual information can provide information that cannot be obtained from distance measurements alone.

The main development improvement was replacing the original narrow-FOV lens with an approximately 60° lens.

---

## 6.3 LiDAR vs. Ultrasonic Sensors

The initial concept used ultrasonic sensors as part of the front sensing system.

During development, the system was changed to the RPLiDAR C1 and the ultrasonic sensors were removed.

The LiDAR provided 2D environmental information that better matched our navigation strategy.

This allowed the front sensing system to become simpler while providing richer spatial information.

---

## 6.4 Light Sensor

The first prototype included a light sensor mounted beneath the front steering structure.

The original concept was to detect blue and orange field markings and help determine when the robot should turn.

After changing to the LiDAR-based navigation architecture, the light sensor was removed because it was no longer required by the final navigation system.

---

## 6.5 IMU

The BNO055 was retained because heading information remained useful even after the LiDAR-based architecture was introduced.

The IMU provides a relative orientation reference that complements LiDAR-based environmental localization.

---

## 6.6 IO Expansion HAT

The DFR0566 was retained because the Raspberry Pi still required a structured interface for its connected peripherals.

The HAT simplified physical wiring and provided a consistent connector and I/O layer between the Raspberry Pi and external electronics.

It therefore remained part of the final electrical architecture even after the sensor architecture changed.

---

# 7. Wiring Architecture

## 7.1 Schematic Diagram

![Schematic Diagram](../schemes/Schematic%20Diagram.png)

The schematic shows the electrical connections between:

* LiPo battery
* Power-distribution connectors
* LM2596
* XL4015
* Raspberry Pi
* DFR0566 IO Expansion HAT
* Arduino
* Motor driver
* Drive motor
* Steering servo
* Encoder
* Sensors
* Start switch
* Common ground

The final distribution uses:

* **D1-2 for positive power**
* **PCT-21 for negative / ground**

---

## 7.2 Physical Wiring Diagram

![Wiring Diagram](../schemes/Wiring%20Diagram.png)

The wiring diagram shows the physical connections and routing used in the final robot.

The DFR0566 is shown as part of the Raspberry Pi-side interface layer.

It should be used together with the component-layout images at the beginning of this document when reproducing the final electrical system.

---

## 7.3 High-Level Electrical Architecture

```text
                         ┌─────────────────────┐
                         │   3S LiPo Battery   │
                         │    11.1 V nominal   │
                         └──────────┬──────────┘
                                    |
                    ┌───────────────┴───────────────┐
                    |                               |
                    v                               v
             ┌────────────┐                  ┌────────────┐
             │   LM2596   │                  │   XL4015   │
             │   ~5.1 V   │                  │    ~5 V    │
             └─────┬──────┘                  └─────┬──────┘
                   |                               |
                   v                               v
           ┌──────────────┐             ┌──────────────────┐
           │ Raspberry Pi │             │ Arduino + Motor  │
           │      5       │             │ Control System   │
           └───────┬──────┘             └────────┬─────────┘
                   |
                   v
           ┌─────────────────┐
           │ DFR0566 IO HAT  │
           │ Pi-side I/O     │
           └───────┬─────────┘
                   |
          ┌────────┼────────┐
          |        |        |
          v        v        v
       BNO055   Pi-side   Other
                Peripherals I/O
```

The camera and LiDAR use their dedicated Raspberry Pi interfaces, while the DFR0566 provides the Raspberry Pi-side I/O expansion and peripheral interface.

The Raspberry Pi and Arduino are connected through USB Serial.

---

# 8. Calibration and Initialization

## 8.1 Startup Sequence

The general startup sequence is:

```text
Power On
   |
   v
Initialize Raspberry Pi systems
   |
   v
Initialize Camera / LiDAR / IMU / Arduino
   |
   v
BNO055 settles for approximately 1 second
   |
   v
Wait for competition start condition
   |
   v
Capture current IMU heading
   |
   v
Initial heading = local 0° reference
   |
   v
Begin autonomous operation
```

---

## 8.2 Camera

The primary camera adjustment is the field of view.

```text
Problem:
Original lens had a narrow FOV

Adjustment:
Replace with wider-angle lens

Final result:
Approximately 60° FOV
```

---

## 8.3 LiDAR

The primary LiDAR adjustment was physical mounting.

```text
Problem:
LiDAR was mounted at an excessive angle

Adjustment:
Reposition LiDAR closer to parallel with the ground

Result:
More accurate 2D environmental representation
```

---

## 8.4 IMU

The BNO055 does not perform a complete magnetometer calibration before every run.

Instead:

```text
BNO055 Connected
       |
       v
Wait ~1 second
       |
       v
Wait for Start
       |
       v
Capture Current Heading
       |
       v
Initial Heading = Local 0°
```

The target heading can then be changed by relative turns.

For example:

```text
90° clockwise turn
→ target heading = initial heading + 90°
```

The software also applies:

* compass offset
* compass sign correction
* heading-error feedback

The main heading proportional gain is currently:

```text
HEADING_KP = 0.5
```

The heading correction is limited by the software before being sent to the steering system.

---

# 9. Development History and Iteration

> V1 did not have a complete autonomous test run because the Raspberry Pi driving software had not yet been implemented. Therefore, no autonomous performance result is claimed for V1.

## 9.1 Version 1 — Initial Prototype

<img width="500" height="550" alt="V1" src="https://github.com/user-attachments/assets/2cd51408-8423-43bb-8a1c-9e98e6d49be4" />

### Initial Design

Version 1 was the first physical robot concept.

It used:

* Ultrasonic sensors at the front
* A light sensor beneath the front steering structure
* A camera-based sensing concept
* BNO055 IMU beside the Raspberry Pi I/O Expansion HAT

The original concept used:

* camera information for visual sensing,
* ultrasonic sensors for front distance,
* and the light sensor for field-marking detection.

At this stage, Arduino-side control software had been developed, but the Raspberry Pi autonomous driving software was not yet implemented.

### Design Decision

The sensing architecture was later changed to a LiDAR-based approach.

The ultrasonic and light-sensor architecture was therefore discontinued.

---

## 9.2 Version 2 — LiDAR-Based Prototype

<img width="355" height="355" alt="V2 Front" src="https://github.com/user-attachments/assets/565e1a1a-5c90-46f8-8a8d-0b25f9c8d971" />

<img width="355" height="355" alt="V2 Back" src="https://github.com/user-attachments/assets/888e36f8-cadd-46e1-8e95-7e89deb52e78" />

<img width="355" height="355" alt="V2 Left" src="https://github.com/user-attachments/assets/455b8f9b-da00-4c4a-b316-7e742cbc7fa5" />

<img width="355" height="355" alt="V2 Right" src="https://github.com/user-attachments/assets/cefe285f-9064-48ea-a7d9-442a564c19d1" />

<img width="355" height="355" alt="V2 Top" src="https://github.com/user-attachments/assets/b6866442-817b-4d32-8025-cc98f1e219f3" />

<img width="355" height="355" alt="V2 Bottom" src="https://github.com/user-attachments/assets/657bd3e9-f3c4-46d8-922b-e58752a9c771" />

### Main Changes

* Added RPLiDAR C1
* Removed ultrasonic sensors
* Removed light sensor
* Moved the BNO055 IMU underneath the LiDAR
* Simplified the front sensing architecture

The LiDAR-based system provided the environmental distance information required by the navigation strategy.

Development then focused increasingly on software tuning and navigation behavior.

---

## 9.3 Version 3 — Final Robot

Version 3 is the current final physical robot.

Unlike V1 and V2, Version 3 was completely disassembled and rebuilt as a new robot.

### Main Changes

* Redesigned mechanical structure
* Improved drivetrain
* Higher motor torque and usable speed
* LiDAR retained as the main distance sensor
* BNO055 retained underneath the LiDAR
* DFR0566 retained as the Raspberry Pi-side I/O interface
* Final electrical layout
* Final component placement
* Final wiring architecture

The navigation concept remained based on the sensing architecture developed in V2, while the physical platform and drivetrain were redesigned for the final robot.

---

# 10. Testing and System Tuning

## 10.1 Software Iteration

The electrical and sensing systems were repeatedly tested together with the navigation software.

Observed problems included:

* Wall collisions
* Incorrect turning direction
* Misalignment during turns
* Continuous rotation in some conditions
* Changes in behavior caused by sensor noise or control parameters

These problems were addressed through repeated testing and parameter adjustment.

The detailed software-control implementation is documented separately in the software documentation.

---

## 10.2 Electrical / Sensor Iteration

The main electrical and sensing improvements were:

1. Replacing the original camera lens with an approximately 60° lens.
2. Correcting the LiDAR mounting angle.
3. Moving the IMU from beside the Raspberry Pi I/O Expansion HAT to underneath the LiDAR.
4. Retaining the DFR0566 as the Pi-side I/O interface during the final redesign.
5. Replacing the ultrasonic/light-sensor architecture with LiDAR-based environmental sensing.
6. Separating the Raspberry Pi power branch from the motor/control branch.

---

# 11. Failure Modes and Reliability

| Failure Mode            | Cause                                      | Effect                            | Mitigation                                           |
| ----------------------- | ------------------------------------------ | --------------------------------- | ---------------------------------------------------- |
| Narrow camera FOV       | Original lens                              | Limited visible field             | Replace lens with wider-angle lens                   |
| Incorrect LiDAR angle   | Sensor not sufficiently parallel to ground | Distorted 2D scan                 | Reposition LiDAR                                     |
| Crowded IMU placement   | IMU mounted beside Raspberry Pi I/O HAT    | Crowded electronics area          | Move IMU underneath LiDAR                            |
| Motor power disturbance | Shared power demand                        | Possible computing instability    | Separate regulated power branches                    |
| Wall collision          | Navigation parameters not fully tuned      | Robot may hit wall                | Continue navigation tuning                           |
| Incorrect turning       | Heading / steering control mismatch        | Robot may turn incorrectly        | Tune control parameters                              |
| Continuous rotation     | Incorrect control behavior or tuning       | Robot may fail to recover heading | Tune heading and navigation behavior                 |
| Sensor unavailable      | Hardware or communication failure          | Reduced navigation capability     | Software handles unavailable devices where supported |

The final robot is designed so that electrical layout, sensor placement, and software behavior can be adjusted independently without requiring a complete redesign of the system.

---

# 12. System-Level Engineering Decisions

## 12.1 Electrical → Mechanical

Electrical component placement directly affected the mechanical design.

The BNO055 was originally mounted beside the Raspberry Pi I/O Expansion HAT.

This occupied useful space around the main electronics and increased wiring congestion.

When the robot was rebuilt, the IMU was moved underneath the LiDAR while the DFR0566 remained as the Raspberry Pi-side interface layer.

The LiDAR itself also required a mechanically stable and level mounting position because its physical orientation directly affects the quality of the 2D scan.

---

## 12.2 Electrical → Software

The software architecture depends strongly on the available sensing information.

Changing from:

```text
Ultrasonic + Light Sensor
```

to:

```text
LiDAR + Camera + IMU + Encoder
```

changed the type and quality of information available to the navigation system.

The DFR0566 also provided a more organized interface structure for Raspberry Pi-side peripherals.

As a result:

* the navigation approach changed,
* wall-following behavior was tuned,
* heading correction became important,
* LiDAR-based localization became available,
* and encoder feedback became useful for distance-based motion.

---

## 12.3 Major System Trade-off

> **We chose LiDAR instead of the original ultrasonic and light-sensor approach because LiDAR provided environmental distance information that better matched our navigation strategy.**

This simplified the front sensing system while providing richer spatial information.

The final architecture combines:

```text
Camera
   +
LiDAR
   +
IMU
   +
Encoder
```

with the DFR0566 providing the Raspberry Pi-side I/O interface rather than relying on one direct connection method.

---

# 13. Final Electrical Configuration

The final electrical and sensing system consists of:

### Computing

* Raspberry Pi 5
* DFR0566 IO Expansion HAT

### Low-Level Control

* Arduino UNO R4 Minima

### Sensors

* RPLiDAR C1
* Raspberry Pi Night Vision Camera
* BNO055 IMU
* CHP-20GP-180 encoder

### Actuators

* CHP-20GP-180 drive motor
* GEEKSERVO steering servo

### Motor Control

* L298P Motor Shield

### Power

* Helix 1100 mAh 11.1 V 3S LiPo
* LM2596 step-down converter
* XL4015 step-down converter
* D1-2 positive distribution
* PCT-21 ground distribution

### Competition Interface

* ZX-Switch01 start switch
* SPST ON/OFF main switch

---

# 14. Reproducibility

A team reproducing this electrical system should use the following documentation together:

1. Final hardware layout images
2. Schematic diagram
3. Wiring diagram
4. Controller pin assignment
5. Raspberry Pi I/O Expansion HAT documentation
6. Communication protocol
7. Sensor placement
8. Power-distribution architecture
9. Source code

### Minimum Reproduction Requirements

```text
Power:
3S LiPo
   |
   +--> LM2596 --> Raspberry Pi branch
   |
   +--> XL4015 --> Motor / Control branch

Controllers:
Raspberry Pi 5
      |
      +--> DFR0566 IO Expansion HAT
      |
      | USB Serial, 115200
      v
Arduino UNO R4 Minima

Sensors:
Camera --> Raspberry Pi
LiDAR  --> Raspberry Pi (/dev/ttyUSB0, 460800)
BNO055 --> Raspberry Pi I²C / DFR0566 interface
Encoder --> Arduino D2/D3

Actuators:
Arduino D11/D13 --> Drive Motor
Arduino D9       --> Steering Servo

Start:
ZX-Switch01 --> Arduino A0
```

The final physical wiring should be reproduced according to the schematic and wiring diagrams rather than inferred only from this overview.

The DFR0566 product documentation should also be consulted when reproducing the Raspberry Pi-side interface layout. ([DFRobot Product](https://www.dfrobot.com/product-1930.html), [DFRobot Wiki](https://wiki.dfrobot.com/dfr0566/docs/22892))

---

# 15. References

| Component / Topic                         | Reference                                                                                                            |
| ----------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| Raspberry Pi 5                            | https://www.raspberrypi.com/products/raspberry-pi-5/                                                                 |
| Arduino UNO R4 Minima                     | https://docs.arduino.cc/hardware/uno-r4-minima                                                                       |
| RPLiDAR C1                                | https://www.slamtec.com/en/C1                                                                                        |
| Gravity 10 DOF IMU AHRS (BNO055 + BMP280) | https://www.dfrobot.com/product-1793.html                                                                            |
| DFR0566 IO Expansion HAT                  | https://www.dfrobot.com/product-1930.html                                                                            |
| DFR0566 Product Wiki / Documentation      | https://wiki.dfrobot.com/dfr0566/docs/22892                                                                          |
| Camera                                    | https://www.waveshare.com/wiki/RPi_Camera_(H)                                                                        |
| CHP-20GP-180                              | https://www.airsoftmotor.com/micro-dc-reduction-motor/planetary-gear-motor/chp-20gp-180-dc-planetary-gear-motor.html |
| LM2596                                    | https://www.ti.com/product/LM2596                                                                                    |
| XL4015                                    | https://www.xlsemi.com/datasheet/XL4015-5A-36V-DC-DC-Converter.pdf                                                   |
| L298P Motor Shield                        | https://wiki.dfrobot.com/dri0017/                                                                                    |

---

# 16. Related Documentation

For a complete understanding of the robot, this document should be read together with:

* Mechanical documentation
* Software documentation
* Navigation documentation
* Schematic diagram
* Wiring diagram
* Source code
* Competition / engineering documentation

The electrical documentation describes the hardware architecture and interfaces, while the software documentation describes how the Raspberry Pi and Arduino use these interfaces during autonomous operation.
