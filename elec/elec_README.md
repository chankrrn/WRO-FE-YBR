# Electrical, Power & Sensor Architecture

This document describes the complete electrical and sensing architecture of the **YBR-SUNFLOWER WRO 2026 Future Engineers robot**, including the power system, controllers, communication interfaces, sensors, physical sensor placement, wiring, calibration and initialization methods, development iterations, failure modes, reliability decisions, and final electrical configuration.

The purpose of this document is not only to identify the components used in the robot, but also to explain why each electrical and sensing component was selected, how power is generated, regulated, and distributed, how current demand influenced the power architecture, why different sensors are used for different types of information, why each sensor is placed in its final physical position, how the sensors are calibrated or initialized, what electrical and sensing problems were discovered during development, how testing changed the final architecture, what failure modes were considered, how electrical, mechanical, and software decisions affect one another, and how the final system can be reproduced.

---

# Contents

1. [Electrical Engineering Overview](#1-electrical-engineering-overview)
2. [Electrical Design Requirements and Constraints](#2-electrical-design-requirements-and-constraints)
3. [Power Architecture](#3-power-architecture)
4. [Power Budget and Electrical Load Analysis](#4-power-budget-and-electrical-load-analysis)
5. [Controllers and Communication Architecture](#5-controllers-and-communication-architecture)
6. [Sensor Architecture and Selection](#6-sensor-architecture-and-selection)
7. [Sensor Placement and Field-Based Reasoning](#7-sensor-placement-and-field-based-reasoning)
8. [Calibration, Initialization and Signal Quality](#8-calibration-initialization-and-signal-quality)
9. [Interface and Pin Assignment](#9-interface-and-pin-assignment)
10. [Wiring Architecture](#10-wiring-architecture)
11. [Electrical and Sensor Development — V1 to V3](#11-electrical-and-sensor-development--v1-to-v3)
12. [Testing and Reliability Iteration](#12-testing-and-reliability-iteration)
13. [Failure Modes, Noise and Risk Mitigation](#13-failure-modes-noise-and-risk-mitigation)
14. [System-Level Engineering Decisions and Trade-offs](#14-system-level-engineering-decisions-and-trade-offs)
15. [Final Electrical Configuration](#15-final-electrical-configuration)
16. [Electrical Reproducibility](#16-electrical-reproducibility)
17. [References](#17-references)

---

# 1. Electrical Engineering Overview

The final electrical system combines high-level computing, low-level actuator control, multiple sensing modalities, and two power branches into one integrated architecture.

The electrical system can be divided into five functional groups: **power generation and distribution, high-level computing, low-level control, sensing, and actuation and feedback**.

The final architecture uses the Raspberry Pi 5, DFR0566 IO Expansion HAT, Arduino UNO R4 Minima, RPLiDAR C1, Raspberry Pi Night Vision Camera, Gravity BNO055 IMU, CHP-20GP-180 motor encoder, L298P Motor Shield, GEEKSERVO steering servo, 3S LiPo battery, LM2596 step-down converter, XL4015 step-down converter, D1-2 positive distribution connector, PCT-21 ground distribution connector, SPST main power switch, and ZX-Switch01 competition start switch.

---

## 1.1 Final Electrical Hardware Layout

| Base Plate | Front Section | Middle Section |
|---|---|---|
| <img width="700" height="600" alt="Base Plate View" src="../other/ComponentsImage1.png" /> | <img width="700" height="600" alt="Right View" src="../other/ComponentsImage2.png" /> | <img width="700" height="600" alt="Left View" src="../other/ComponentsImage3.png" /> |

| Upper Back Section | Lower Back Section |
|---|---|
| <img width="700" height="600" alt="Front View 1" src="../other/ComponentsImage4.png" /> | <img width="700" height="600" alt="Front View 2" src="../other/ComponentsImage5.png" /> |

The annotated photographs show the physical relationship between the controllers, sensors, power electronics, actuators, and wiring in the Version 3 robot.

---

## 1.2 Hardware Roles

| Component | Electrical / System Role |
|---|---|
| Raspberry Pi 5 | High-level perception, localization, navigation, and autonomous decision-making |
| DFR0566 IO Expansion HAT | Raspberry Pi peripheral breakout and organized I/O interface |
| Arduino UNO R4 Minima | Low-level drive, steering, encoder, and start-button control |
| RPLiDAR C1 | 2D environmental-distance sensing |
| Raspberry Pi Night Vision Camera | Visual sensing and traffic-pillar detection |
| Gravity BNO055 IMU | Relative heading and orientation reference |
| CHP-20GP-180 Encoder | Drive-motor rotation feedback |
| CHP-20GP-180 Motor | Main drivetrain actuator |
| L298P Motor Shield | Drive-motor control |
| GEEKSERVO | Steering actuator |
| 3S LiPo Battery | Main onboard electrical energy source |
| LM2596 | Raspberry Pi-side voltage conversion |
| XL4015 | Motor/control-side voltage conversion |
| D1-2 | Positive power distribution |
| PCT-21 | Common negative / ground distribution |
| SPST Switch | Main robot power control |
| ZX-Switch01 | Competition start input |

---

## 1.3 Criterion 2 Evidence Map

| Level 6 Requirement | Evidence in This Document |
|---|---|
| Power-system architecture | Sections 3 and 10 |
| Power-budget analysis | Section 4 |
| Current-distribution reasoning | Sections 3.4 and 4 |
| Sensor-selection trade-offs | Section 6 |
| Field-based sensor placement | Section 7 |
| Calibration / setup methods | Section 8 |
| Noise / interference considerations | Sections 8.6 and 13 |
| Failure-point analysis | Section 13 |
| Reliability-focused iteration | Sections 11 and 12 |
| Wiring reproducibility | Sections 9, 10 and 16 |
| System-level trade-offs | Section 14 |

---

# 2. Electrical Design Requirements and Constraints

The electrical architecture must support the complete autonomous vehicle while operating from one onboard battery.

The system must provide stable power to the Raspberry Pi and sufficient power for the motor and steering system while supporting several sensors with different electrical interfaces. It must maintain a common electrical reference, separate the sensitive computing load from rapidly changing actuator loads as much as practical, and provide reliable Raspberry Pi-to-Arduino communication.

The electrical hardware must also fit inside the compact mechanical structure, remain accessible for testing and maintenance, and initialize safely before the competition start command.

---

## 2.1 Main Electrical Constraints

| Constraint | Engineering Response |
|---|---|
| Single main onboard battery | Use voltage-conversion stages for different electrical requirements |
| Battery voltage changes during discharge | Use step-down conversion instead of supplying sensitive electronics directly |
| Raspberry Pi sensitive to supply instability | Dedicated computing power branch |
| Motor / servo current changes rapidly | Separate actuator-related power branch from Pi branch |
| Limited chassis space | Layered electrical mounting |
| Multiple Pi peripherals | DFR0566 I/O Expansion HAT |
| Different sensor interfaces | CSI, USB Serial and I²C used according to device |
| Competition start procedure | Separate main ON/OFF switch and start button |
| LiDAR requires planar sensing | Rigid, approximately level mounting |
| Camera depends on field visibility | Elevated adjustable camera structure |
| IMU absolute magnetic heading not required | Relative heading initialization |
| Need for repeatable assembly | Documented pin assignments, schematic and wiring diagram |

---

# 3. Power Architecture

## 3.1 Main Battery

The complete robot uses one **3S LiPo battery** as its primary power source.

| Specification | Value |
|---|---|
| Battery type | LiPo |
| Configuration | 3S |
| Nominal voltage | 11.1 V |
| Fully charged voltage | Approximately 12.6 V |
| Capacity | 1100 mAh |
| Manufacturer / model | Helicox |

The nominal 11.1 V value does not mean that the battery remains at exactly 11.1 V throughout a run. A 3S LiPo reaches approximately 12.6 V when fully charged and decreases in voltage as energy is used.

Because the battery voltage is variable and is higher than the supply required by several electronic subsystems, regulated voltage conversion is required.

---

### 3.1.1 Measured Battery Operating Range

The battery voltage changes continuously with state of charge, so it is not treated as one fixed operating value. A fully charged 3S pack measures approximately **12.6 V**.

During robot testing, the team observed that drivetrain performance begins to decrease when the battery falls below approximately **11.1 V**, with the drive motor becoming noticeably slower. For this reason, approximately **11.1 V is used as a practical performance threshold for this robot**, not as the absolute minimum electrical voltage of the battery.

```text
Fully charged battery             ≈ 12.6 V
Practical performance threshold   ≈ 11.1 V
```

This threshold is an observed system-level behavior of the final robot. It should not be interpreted as a separately characterized discharge limit of the LiPo cell chemistry.

---

## 3.2 High-Level Power Distribution

The final design divides the battery supply into two principal branches:

```text
                    3S LiPo Battery
                     11.1 V nominal
                           |
                     Main Power Switch
                           |
                 Positive Distribution
                        (D1-2)
                           |
              +------------+------------+
              |                         |
              v                         v
           LM2596                    XL4015
              |                         |
              v                         v
       Computing Branch          Motor / Control Branch
              |                         |
              v                         v
        Raspberry Pi 5           Motor / Control System
```

The negative side of the electrical system is distributed through the PCT-21 connector so that the required subsystems share a common electrical reference.

---

## 3.3 Raspberry Pi Power Branch — LM2596

The LM2596 supplies the Raspberry Pi-side power branch.

The converter output is adjusted to approximately **5.1 V**. A static measurement on the completed Version 3 robot showed approximately **5.0 V at the Raspberry Pi input**.

```text
LM2596 output          = 5.1 V
Raspberry Pi input     = 5.0 V
Measured voltage drop  ≈ 0.1 V
Test condition         = robot powered, no autonomous run
```

The measured difference between the converter output and the Pi input is approximately **0.1 V**. This supports the decision to set the converter slightly above 5.0 V at its output so that small losses through wiring and connectors do not reduce the voltage at the Raspberry Pi input below the intended value.

The Raspberry Pi supply is separated from the motor/control branch because the Pi is more sensitive to short supply-voltage disturbances than the mechanical actuators.

---

### 3.3.1 Raspberry Pi Supply Validation

The Raspberry Pi 5 has a load-dependent current requirement. Its actual power consumption changes with CPU load, connected USB devices, camera activity, peripherals, and software workload.

The static measurement validates the basic voltage path, but it does not yet validate the Pi rail under the highest competition load.

| Condition | Pi Rail Voltage | Pi Rail Current |
|---|---:|---:|
| Static / idle power-on | **5.0 V measured at Pi input** | **[TODO]** |
| LiDAR + camera active | **[TODO]** | **[TODO]** |
| Full navigation software | **[TODO]** | **[TODO]** |
| Highest observed load | **[TODO]** | **[TODO]** |

> **[TODO: Measure Raspberry Pi-branch voltage and current while running the final Obstacle Challenge software with LiDAR, camera, IMU and Arduino connected.]**

---

## 3.4 Motor / Control Power Branch — XL4015

The XL4015 supplies the motor/control-side power branch.

The final XL4015 setting has been verified directly on the completed Version 3 robot. A static multimeter measurement showed **11.1 V at the XL4015 output** and approximately **11.0 V at the motor/control-side input**.

```text
XL4015 output             = 11.1 V
Motor / control input     = 11.0 V
Measured voltage drop     ≈ 0.1 V
Test condition            = robot powered, no autonomous run
```

This measurement resolves the earlier documentation conflict between an approximately 5 V value and an approximately 11.1 V value. The final Version 3 configuration uses an **11.1 V XL4015 output**.

> **[TODO: Confirm exactly which final devices are powered directly from the XL4015 output: motor-power rail, L298P Motor Shield, Arduino supply, steering-servo rail, or a combination of these.]**

Because the XL4015 is a step-down converter, its available regulation headroom becomes smaller as the battery voltage approaches the 11.1 V output setting. This is consistent with the team's observation that drivetrain performance decreases when the battery falls below approximately 11.1 V. However, this observation is documented as a system-level behavior rather than as a separately isolated converter-efficiency test.

---

## 3.5 Common Ground Architecture

Although the power system uses separate voltage-conversion branches, the controllers and signal interfaces require a shared electrical reference where appropriate.

The PCT-21 connector is used as the common negative / ground distribution point.

```text
Battery Negative
      |
      v
    PCT-21
      |
      +---- LM2596 Ground
      |
      +---- XL4015 Ground
      |
      +---- Controller / Signal Grounds
```

This common reference is necessary because digital signals such as motor control, encoder feedback, sensor communication, and controller interfaces must be interpreted relative to a known ground potential.

---

# 4. Power Budget and Electrical Load Analysis

The power system must support both relatively stable computing loads and rapidly changing actuator loads.

For this reason, the power budget is separated into **reference component requirements** and **measurements from the completed robot**. Manufacturer specifications are useful for planning, while measurements from the final robot provide stronger evidence of real operating behavior.

---

## 4.1 Reference Electrical Loads

| Component | Supply | Reference / Expected Load | Notes |
|---|---:|---:|---|
| Raspberry Pi 5 | 5.0 V class supply | Load dependent | Static input measured at 5.0 V |
| RPLiDAR C1 | 5 V USB | ~290 mA reference | Verify against final device documentation |
| Camera | Pi camera interface | Module dependent | Exact final module current not yet measured |
| BNO055 | Low-current sensor | Small relative to actuators | Exact module current not yet measured |
| CHP-20GP-180 Motor | Motor rail | ≤280 mA no-load, ≤550 mA rated, ≤2.7 A stall | Manufacturer / supplier reference |
| GEEKSERVO | Servo rail | Load dependent | Stall current requires final verification |

These values should not be interpreted as measurements from the assembled robot unless explicitly identified as measured values.

---

## 4.2 Measured Static Power-Path Values

| Measurement Point | Measured Value |
|---|---:|
| Battery when fully charged | ~12.6 V |
| Practical battery performance threshold | ~11.1 V |
| LM2596 output | 5.1 V |
| Raspberry Pi input | 5.0 V |
| Pi-branch static voltage drop | ~0.1 V |
| XL4015 output | 11.1 V |
| Motor/control input | 11.0 V |
| Motor/control static voltage drop | ~0.1 V |

These measurements were taken with the robot powered but without an autonomous run. They validate the static power architecture but do not replace dynamic-load testing.

---

## 4.3 Remaining Dynamic Measurements

### Computing Branch

| Test Condition | Voltage | Current | Power |
|---|---:|---:|---:|
| Static / idle power-on | **5.0 V at Raspberry Pi input** | **[TODO]** | **[TODO]** |
| Sensors active | **[TODO]** | **[TODO]** | **[TODO]** |
| Full competition software | **[TODO]** | **[TODO]** | **[TODO]** |

### Motor / Control Branch

| Test Condition | Voltage | Current | Power |
|---|---:|---:|---:|
| Static / motor stopped | **11.0 V at motor/control input** | **[TODO]** | **[TODO]** |
| Normal straight driving | **[TODO]** | **[TODO]** | **[TODO]** |
| Acceleration | **[TODO]** | **[TODO]** | **[TODO]** |
| Steering while driving | **[TODO]** | **[TODO]** | **[TODO]** |
| Highest observed load | **[TODO]** | **[TODO]** | **[TODO]** |

The static measurements confirm the final converter settings and approximately 0.1 V drop on each branch. Current, power, and dynamic voltage behavior still require measurement under representative competition load.

---

## 4.4 Battery Runtime

The final robot uses a 1100 mAh battery, but battery capacity alone does not determine the real autonomous runtime because current demand changes continuously with CPU load, sensor activity, steering activity, motor speed, acceleration, and drivetrain resistance.

> **[TODO: Record battery voltage before and after a known-duration Obstacle Challenge test and document the test duration.]**

A measured runtime test would provide stronger evidence than estimating runtime from the battery capacity alone.

---

# 5. Controllers and Communication Architecture

The final robot divides control between the Raspberry Pi 5 and Arduino UNO R4 Minima.

```text
High-Level Control
Raspberry Pi 5
     |
     | USB Serial
     | 115200 baud
     v
Low-Level Control
Arduino UNO R4 Minima
     |
     +--> Motor Driver
     +--> Steering Servo
     +--> Encoder
     +--> Start Switch
```

The Raspberry Pi performs perception, localization, navigation, obstacle processing, and autonomous decision-making. The Arduino performs time-sensitive low-level actuator control and encoder handling.

This division prevents the Raspberry Pi from directly managing every motor-control detail while it is also processing camera and LiDAR data.

---

## 5.1 Raspberry Pi 5

The Raspberry Pi 5 is the high-level computing platform.

It processes LiDAR data, camera frames, IMU information, localization, path planning, Pure Pursuit steering calculations, obstacle decisions, and the overall autonomous behavior.

Its interfaces include CSI for the camera, USB for the LiDAR and Arduino, and I²C through the DFR0566 for the BNO055.

---

## 5.2 DFR0566 IO Expansion HAT

The DFR0566 is used as an interface layer on the Raspberry Pi.

It provides organized access to GPIO, I²C and other interfaces, reducing direct loose wiring around the Pi header and making the final sensor connection easier to reproduce.

In the final robot, the BNO055 is connected through the I²C interface provided by this expansion board.

---

## 5.3 Arduino UNO R4 Minima

The Arduino UNO R4 Minima handles low-level hardware control.

Its responsibilities include drive-motor PWM and direction, steering-servo commands, quadrature encoder input, start-switch detection, and communication with the Raspberry Pi.

This division allows the Arduino to handle hardware timing while the Raspberry Pi focuses on navigation and perception.

---

## 5.4 Raspberry Pi ↔ Arduino Communication

Communication uses USB serial at **115200 baud**.

The default device path on the Raspberry Pi is:

```text
/dev/ttyACM0
```

However, Linux device numbering can change depending on connection order. Therefore, this should be treated as the expected default rather than a guaranteed permanent identifier.

The command format is:

```text
<servoAngle>,<speed>,<distance>
```

The Arduino can respond with messages including:

```text
READY
Start
OK
t
ERR
```

The exact command behavior is documented in the software architecture.

---

# 6. Sensor Architecture and Selection

No single sensor can provide every type of information required by the robot.

The final architecture therefore combines several sensors with different strengths.

| Sensor | Main Information | Why It Is Used |
|---|---|---|
| RPLiDAR C1 | Environmental geometry | Field localization and wall geometry |
| Camera | Visual color / image information | Red and green traffic pillars |
| BNO055 | Orientation / heading | Relative heading reference |
| Motor Encoder | Motor rotation | Low-level drivetrain feedback |
| ZX-Switch01 | Human start input | Competition start procedure |

The sensors are complementary rather than redundant. LiDAR is strong at geometric distance measurement but cannot identify traffic-pillar color. The camera can identify color but is more sensitive to lighting. The IMU provides orientation information but does not directly provide the robot's field position.

---

## 6.1 RPLiDAR C1

The RPLiDAR C1 is the primary environmental-geometry sensor.

It provides a 360° two-dimensional scan around the robot and is connected to the Raspberry Pi through USB serial.

The final software uses the scan to compare the observed environment with the known WRO field geometry for localization.

The LiDAR replaced the earlier ultrasonic-sensor concept because a full scan provides substantially richer spatial information than several isolated distance measurements.

---

## 6.2 Raspberry Pi Night Vision Camera

The camera is used primarily for traffic-pillar detection.

The camera provides the visual information required to distinguish red and green pillars, which cannot be determined from LiDAR geometry alone.

The original lens had a narrow field of view, so the lens was replaced with a wider approximately **60° physical lens**.

> **[TODO: Verify the final effective camera FOV. Electrical documentation records approximately 60°, while the current software calibration contains an 80° FOV value.]**

The final documentation should distinguish between the physical lens specification and the effective/calibrated FOV used by the vision model.

---

## 6.3 Gravity BNO055 IMU

The BNO055 combines accelerometer, gyroscope, and magnetometer sensing with onboard sensor fusion.

In the final robot, it is used primarily as a **relative heading reference** rather than as a fully calibrated absolute magnetic compass.

At startup, the system waits briefly for the sensor to stabilize and then treats the initial fused heading as the local zero reference.

This avoids requiring a lengthy full magnetic calibration procedure before every competition run.

---

## 6.4 Motor Encoder

The CHP-20GP-180 includes a dual-phase quadrature encoder.

The encoder is connected to Arduino interrupt-capable pins D2 and D3.

It provides motor-rotation feedback for low-level movement control and finite-distance motor commands.

The Raspberry Pi navigation system does **not** currently use this encoder as its main field-localization odometry source. Field localization is performed primarily using LiDAR-based environmental geometry together with heading information.

> **[TODO: Verify and document the exact meaning of the current `836 counts/revolution` value in the Arduino implementation, distinguishing raw encoder PPR, quadrature edges, and gearbox-output counts.]**

---

## 6.5 ZX-Switch01 Start Button

The ZX-Switch01 provides the competition start input.

It is connected to Arduino analog input A0 and allows the robot to remain powered and initialized before autonomous movement begins.

This separates:

```text
Power ON
```

from:

```text
Start autonomous movement
```

which is useful for competition preparation and sensor initialization.

---

## 6.6 Sensor Selection Trade-offs

| Decision | Alternative | Benefit | Trade-off |
|---|---|---|---|
| LiDAR | Multiple ultrasonic sensors | Rich 360° geometry | More processing and cost |
| Camera | Color/light sensor only | Detects traffic-pillar color and image position | Lighting sensitive |
| BNO055 | No orientation sensor | Provides heading reference | Magnetic environment can affect fused heading |
| Encoder motor | Open-loop motor | Rotation feedback | Additional wiring and software |
| Sensor fusion | One sensor for everything | Complementary information | Higher integration complexity |

The final architecture accepts higher integration complexity because each sensor provides information that the others cannot provide reliably by themselves.

---

# 7. Sensor Placement and Field-Based Reasoning

Sensor placement was treated as an engineering variable rather than only a packaging problem.

---

## 7.1 Camera Placement

The camera is mounted at the upper front section of the robot.

This elevated position gives it a clearer view of traffic pillars and reduces obstruction from the steering mechanism and other electronics.

The camera mount is mechanically adjustable so that its angle can be changed during testing without requiring the complete structure to be redesigned.

---

## 7.2 LiDAR Placement

The LiDAR is mounted near the upper front section and approximately parallel to the field surface.

An earlier angled mounting arrangement caused the nominally two-dimensional scan plane to intersect the environment at inconsistent heights. This distorted the geometry seen by the localization system.

The sensor was therefore remounted approximately level.

```text
Angled LiDAR
     ↓
Distorted 2D field geometry
     ↓
Level mounting
     ↓
More consistent planar scan
```

This is an example where a mechanical change directly improved sensor data quality.

---

## 7.3 IMU Placement

The BNO055 was moved from its earlier position near the Raspberry Pi / I/O Expansion HAT to a position beneath the LiDAR.

This reduced congestion around the Raspberry Pi and provided a dedicated location for the IMU.

The BNO055 contains a magnetometer, so nearby current-carrying wires, motors, and electronics can potentially influence the magnetic component of the fused heading. The final software therefore uses the IMU primarily as a relative heading reference rather than relying entirely on absolute magnetic north.

---

## 7.4 Encoder Placement

The encoder is integrated directly into the drive motor.

This eliminates the need for a separate wheel encoder and allows the Arduino to measure drivetrain rotation close to the actuator.

The trade-off is that the encoder measures motor rotation rather than direct vehicle displacement, so wheel slip and drivetrain compliance are not observed directly.

---

# 8. Calibration, Initialization and Signal Quality

Different sensors require different initialization or calibration approaches.

---

## 8.1 Camera Calibration

The vision system depends on color thresholds and an assumed camera field of view.

The camera pipeline uses HSV-based color detection for red and green traffic pillars.

Calibration therefore involves checking the effective FOV, confirming that the selected HSV ranges detect the real competition pillars, testing under representative lighting, and adjusting the camera angle if necessary.

> **[TODO: Resolve the 60° physical-lens vs 80° software-FOV value and record the final calibrated value.]**

---

## 8.2 LiDAR Setup

The LiDAR does not require the same type of color or magnetic calibration as the camera and IMU, but its physical orientation is critical.

The main setup requirements are that the sensor remains approximately level, the scan is not mechanically obstructed, the USB serial connection is stable, and the expected scan geometry matches the known field.

The physical level-mount correction was one of the most important LiDAR reliability improvements.

---

## 8.3 BNO055 Initialization

The software allows approximately one second for the BNO055 to settle during startup.

After this period, the current fused heading is stored as the initial reference.

```text
Sensor startup
      |
      v
~1 s settling period
      |
      v
Read initial fused heading
      |
      v
Set local heading = 0°
```

This provides a repeatable local heading coordinate for each autonomous run.

---

## 8.4 Encoder Initialization

The Arduino initializes the encoder inputs and then counts transitions from the A/B quadrature signals.

Direction is determined from the phase relationship between the two signals.

Before autonomous operation, the team must verify that forward motor motion produces the expected encoder sign and reverse motion produces the opposite sign.

---

## 8.5 Start-Switch Initialization

The start switch allows all controllers and sensors to initialize before the robot begins moving.

The startup sequence is:

```text
Main Power ON
      |
      v
Raspberry Pi boots
      |
      v
Arduino initializes
      |
      v
Sensors initialize
      |
      v
Robot waits
      |
      v
ZX-Switch01 pressed
      |
      v
Autonomous run starts
```

---

## 8.6 Signal Quality and Interference Considerations

Signal quality can be affected by electrical and mechanical conditions.

The main concerns are supply-voltage drop during actuator load, motor-generated electrical noise, servo-current changes, loose USB connections, LiDAR cable movement, magnetic disturbance near the IMU, camera lighting variation, and encoder signal integrity.

The design therefore uses separate power conversion branches, common ground where required, fixed sensor mounts, organized wiring, and software checks for unavailable or invalid sensor data.

---

# 9. Interface and Pin Assignment

## 9.1 Arduino Pin Assignment

| Function | Arduino Pin |
|---|---|
| Encoder A | D2 |
| Encoder B | D3 |
| Motor PWM | D11 |
| Motor Direction | D13 |
| Steering Servo | D9 |
| Start Switch | A0 |

---

## 9.2 Raspberry Pi Interfaces

| Device | Interface | Expected Linux / Hardware Path |
|---|---|---|
| Arduino UNO R4 | USB Serial | `/dev/ttyACM0` |
| RPLiDAR C1 | USB Serial | `/dev/ttyUSB0` |
| BNO055 | I²C through DFR0566 | I²C bus |
| Camera | CSI | Raspberry Pi camera interface |

The `/dev/ttyACM0` and `/dev/ttyUSB0` names are expected defaults rather than guaranteed permanent identifiers.

---

## 9.3 Encoder Wiring

| Motor Wire | Function | Connection |
|---|---|---|
| Red | Motor positive | Motor driver |
| White | Motor negative | Motor driver |
| Blue | Hall sensor supply | 5 V |
| Black | Hall sensor ground | GND |
| Green | Encoder A | Arduino D2 |
| Yellow | Encoder B | Arduino D3 |

---

## 9.4 Raspberry Pi ↔ Arduino Protocol

The Raspberry Pi sends commands in the form:

```text
<servoAngle>,<speed>,<distance>
```

The protocol uses human-readable ASCII because it is simple to debug in a serial terminal and straightforward to parse on the Arduino.

The detailed command interpretation belongs to the software documentation:

[`../software/software_README.md`](../software/software_README.md)

---

# 10. Wiring Architecture

## 10.1 Schematic Diagram

The schematic diagram shows the logical electrical connections between the major components.

<img src="../schemes/Schematic%20Diagram.png" width="1000">

---

## 10.2 Physical Wiring Diagram

The wiring diagram shows the physical connection arrangement used on the robot.

<img src="../schemes/Wiring%20Diagram.png" width="1000">

The schematic and wiring diagram serve different purposes. The schematic explains electrical relationships, while the wiring diagram helps another builder reproduce the physical connections.

---

## 10.3 Power Wiring Summary

```text
Battery +
   |
SPST Main Switch
   |
D1-2 Positive Distribution
   |
   +---- LM2596 ---- 5.1 V OUT ---- 5.0 V at Raspberry Pi input
   |
   +---- XL4015 ---- 11.1 V OUT --- 11.0 V at Motor / Control input

Battery -
   |
PCT-21 Ground Distribution
   |
   +---- LM2596 -
   +---- XL4015 -
   +---- Relevant controller / signal grounds
```

---

## 10.4 Signal Wiring Summary

```text
Raspberry Pi
   |
   +---- CSI -------- Camera
   |
   +---- USB -------- RPLiDAR C1
   |
   +---- USB -------- Arduino UNO R4
   |
   +---- DFR0566
           |
           +---- I2C ---- BNO055


Arduino UNO R4
   |
   +---- D2 -------- Encoder A
   +---- D3 -------- Encoder B
   +---- D11 ------- Motor PWM
   +---- D13 ------- Motor Direction
   +---- D9 -------- Steering Servo
   +---- A0 -------- Start Switch
```

---

# 11. Electrical and Sensor Development — V1 to V3

The final architecture was developed through several sensing and electrical iterations.

---

## 11.1 Version 1 — Initial Sensor Concept

The first robot concept used the camera, ultrasonic sensing, a front-facing light sensor, and the BNO055.

The light sensor was mounted beneath the front steering mechanism and pointed toward the field surface to detect colored lines.

At this stage, the Raspberry Pi autonomous architecture was not yet complete.

The main importance of V1 was that it established the initial sensing concept and revealed the limitations of relying on isolated distance measurements.

---

## 11.2 Version 2 — LiDAR Architecture

The team then changed the sensing architecture significantly.

The ultrasonic sensors and light sensor were removed, while the RPLiDAR C1 was added. The BNO055 was relocated beneath the LiDAR.

This changed the robot from a system based on several local measurements into one capable of observing a much richer two-dimensional representation of the surrounding field.

The change also enabled the localization-first software architecture used later.

---

## 11.3 Version 3 — Final Electrical Layout

Version 3 involved a complete physical rebuild of the robot.

The sensing concept from V2 was retained, but the power electronics, controllers, sensor mounts, and wiring were reorganized around the new mechanical structure.

The final architecture uses the Raspberry Pi and DFR0566 on the upper computing layer, Arduino and motor-control hardware on the lower control layer, dedicated LiDAR/IMU mounting, separate LM2596 and XL4015 power-conversion paths, and centralized positive/negative distribution.

---

## 11.4 Electrical Development Summary

| Version | Main Electrical / Sensor Architecture | Main Change |
|---|---|---|
| V1 | Camera + Ultrasonic + Light Sensor + BNO055 | Initial sensing concept |
| V2 | Camera + LiDAR + BNO055 + Encoder | LiDAR replaces ultrasonic / light sensor |
| V3 | Same core sensors with redesigned power and physical layout | Final integrated architecture |

---

# 12. Testing and Reliability Iteration

Testing changed several parts of the final electrical and sensing system.

---

## 12.1 LiDAR Alignment Test

An angled LiDAR mount caused the scan plane to intersect the environment at inconsistent heights.

The problem appeared in software as distorted environmental geometry, but the root cause was mechanical sensor orientation.

The solution was to remount the LiDAR approximately parallel to the field.

This is an important example of system-level debugging because the apparent software/localization problem was solved through a mechanical sensor-placement change.

---

## 12.2 Camera Field-of-View Test

The original camera lens provided a narrower view than required.

This limited how much of the field and traffic pillars could be observed.

The lens was therefore replaced with a wider approximately 60° lens.

The physical lens value still needs to be reconciled with the 80° value currently used in software calibration.

---

## 12.3 IMU Position Change

The BNO055 was initially positioned near the Raspberry Pi and I/O Expansion HAT.

It was later moved beneath the LiDAR to reduce congestion and provide a more dedicated sensor location.

The final software also avoids depending entirely on absolute magnetic heading by using the startup heading as a relative reference.

---

## 12.4 Static Power Validation

The final Version 3 power system was checked using a multimeter while the robot was powered but not performing an autonomous run.

The LM2596 measured **5.1 V at its output**, while the Raspberry Pi received approximately **5.0 V**. The XL4015 measured **11.1 V at its output**, while the downstream motor/control input measured approximately **11.0 V**.

```text
LM2596:  5.1 V OUT  →  5.0 V Pi input
XL4015: 11.1 V OUT  → 11.0 V Motor / Control input
```

Both paths therefore showed an observed static voltage difference of approximately **0.1 V** between the converter output and downstream input.

The battery was also observed to provide approximately **12.6 V when fully charged**. During robot testing, drivetrain performance became noticeably lower below approximately **11.1 V**, so the team treats this as a practical performance threshold.

---

## 12.5 Remaining Dynamic Power Test

Static voltage measurements confirm the converter settings, but they do not show the largest voltage drop that may occur during acceleration and steering.

> **[TODO: Measure Pi rail voltage and motor/control rail voltage during a representative Obstacle Challenge run, especially during simultaneous acceleration and steering.]**

> **[TODO: Measure current on both branches under representative load if this can be done safely with suitable measurement equipment.]**

---

## 12.6 Test Evidence Table

| Test | Problem / Observation | Change | Result |
|---|---|---|---|
| LiDAR mounting | Angled scan distorted geometry | Remounted level | More consistent planar scan |
| Camera view | Original FOV too narrow | Wider lens installed | Larger visible field |
| IMU integration | Congested earlier location | Moved under LiDAR | Cleaner final sensor layout |
| Pi static power path | Wiring causes small voltage loss | LM2596 set to 5.1 V | 5.0 V measured at Pi input |
| Motor/control static power path | Final XL4015 setting previously undocumented | Measured final robot | 11.1 V output, 11.0 V downstream |
| Battery state | Motor slows as battery discharges | Use practical voltage threshold | ~11.1 V used as performance threshold |
| V1 sensing | Limited isolated distance information | Replaced ultrasonic/light sensing with LiDAR | Richer field geometry |

---

# 13. Failure Modes, Noise and Risk Mitigation

The electrical system was designed with several possible failure modes in mind.

| Failure Mode | Effect | Mitigation / Design Response |
|---|---|---|
| Pi supply voltage drops | Reboot / instability | Dedicated LM2596 branch |
| Pi-branch wiring voltage drop | Reduced Pi input voltage | 5.1 V converter output measured as 5.0 V at Pi input |
| Motor/control wiring voltage drop | Reduced actuator-side voltage | 11.1 V XL4015 output measured as 11.0 V at branch input |
| Battery voltage decreases | Motor becomes slower | ~11.1 V practical performance threshold |
| Motor current spike | Voltage disturbance | Separate motor/control conversion branch |
| Servo current change | Supply disturbance | Keep actuator load away from Pi branch where practical |
| Loose USB connection | Sensor/controller loss | Secure cable routing and pre-run detection check |
| LiDAR tilted | Incorrect scan geometry | Rigid level mount |
| LiDAR unavailable | Localization unavailable | Software startup/error handling |
| Camera exposure changes | Unstable color detection | HSV tuning; future exposure locking |
| IMU magnetic disturbance | Heading bias | Relative heading reference + LiDAR geometry |
| Encoder signal error | Incorrect distance feedback | Verify direction/count behavior before run |
| Incorrect converter setting | Component damage / instability | Measure output before connecting electronics |
| Wiring short | High current / damage | Inspect and verify polarity before power-on |

---

## 13.1 Why Separate Power Branches?

The Raspberry Pi and the drivetrain have very different electrical behavior.

The Pi requires a relatively stable low-voltage supply, while the motor and servo can change current rapidly during acceleration, braking, and steering.

Using separate conversion branches reduces the direct interaction between these load types.

This does not mean that electrical disturbances are completely eliminated, but it gives the sensitive computing system a more controlled supply path.

---

## 13.2 Why Measure at the Load?

The converter output alone does not show the voltage actually received by the device.

For example, the completed robot measured:

```text
LM2596 output      = 5.1 V
Pi input           = 5.0 V

XL4015 output      = 11.1 V
Motor/control input = 11.0 V
```

Measuring both points reveals the approximately **0.1 V static drop** through each downstream wiring path.

This is more informative than documenting only the converter adjustment.

---

# 14. System-Level Engineering Decisions and Trade-offs

The electrical system contains several deliberate trade-offs.

| Decision | Alternative | Benefit | Trade-off |
|---|---|---|---|
| Separate power branches | One common regulated branch | Reduced interaction between actuator and Pi loads | More converters and wiring |
| LiDAR | Ultrasonic sensors | Rich 2D environmental data | More software complexity |
| Camera + LiDAR | LiDAR alone | Adds pillar color information | Lighting sensitivity |
| Relative IMU initialization | Full startup magnetometer calibration | Fast practical competition startup | Less reliance on absolute magnetic heading |
| DFR0566 | Direct Pi header wiring | Cleaner / repeatable I/O | Additional board and space |
| Encoder motor | Open-loop DC motor | Rotation feedback | More wiring / software |
| Adjustable camera mount | Fixed mount | Faster sensor tuning | Additional mechanical complexity |
| One main battery | Multiple independent batteries | Simpler energy source | Requires careful distribution and conversion |

---

## 14.1 Electrical → Mechanical

Electrical components require physical mounting space.

The Raspberry Pi and HAT require a dedicated structural layer, the converters require trays and plates, the LiDAR must remain level, the IMU position changed because of electronics congestion, and cable routing affects structural placement.

Therefore, electrical packaging influenced the final mechanical design.

---

## 14.2 Electrical → Software

The sensor architecture determines what information the software can use.

The change from:

```text
Ultrasonic + Light Sensor
```

to:

```text
LiDAR + Camera + IMU + Encoder
```

enabled a fundamentally different navigation strategy.

LiDAR provides the geometric information required for field localization, while the camera supplies traffic-pillar color information that LiDAR cannot measure. The IMU provides heading context, and the encoder provides information about drivetrain rotation.

The electrical sensor architecture therefore directly enabled the final localization and obstacle-handling software.

---

## 14.3 Mechanical → Electrical / Sensor Quality

Mechanical alignment also affects sensing.

```text
LiDAR angle
→ scan geometry

Camera position
→ visible field

IMU position
→ wiring congestion / magnetic environment

Motor alignment
→ mechanical load
→ electrical current
```

This is why sensor placement and electrical design cannot be evaluated independently from the chassis.

---

## 14.4 Most Important Electrical Decision

One of the most important electrical decisions was to use a separated power-distribution architecture rather than treating all loads as one shared low-voltage system.

The Raspberry Pi requires a stable power supply, while the motor and servo can create rapidly changing electrical loads. Therefore, the final architecture uses separate conversion paths for the computing and actuator systems while maintaining a shared ground reference.

---

# 15. Final Electrical Configuration

## Computing

The high-level computing system consists of the **Raspberry Pi 5** and **DFR0566 IO Expansion HAT**.

## Low-Level Control

Low-level actuator and encoder control is handled by the **Arduino UNO R4 Minima**.

## Environmental Sensors

The robot uses the **RPLiDAR C1** for environmental geometry and the **Raspberry Pi Night Vision Camera** for visual traffic-pillar information.

## Orientation

The **Gravity BNO055 IMU** provides the relative heading reference.

## Motion Feedback

The **CHP-20GP-180 dual-phase encoder** provides motor-rotation feedback.

## Actuators

The robot uses the **CHP-20GP-180 drive motor** and **GEEKSERVO steering servo**.

## Motor Control

Drive-motor control is provided by the **L298P Motor Shield**.

## Main Power

The main energy source is a **Helicox 3S 11.1 V 1100 mAh LiPo battery**. The battery measures approximately **12.6 V when fully charged**, while approximately **11.1 V is treated as the practical drivetrain-performance threshold**.

## Power Conversion

The Raspberry Pi branch uses an **LM2596 adjusted to 5.1 V**, with **5.0 V measured at the Raspberry Pi input** under static power-on.

The motor/control branch uses an **XL4015 adjusted to 11.1 V**, with **11.0 V measured at the downstream motor/control input** under static power-on.

The exact list of devices connected directly to the XL4015 output should still be confirmed from the final wiring.

## Power Distribution

Positive power distribution uses the **D1-2**, while the **PCT-21** provides common negative / ground distribution.

## Competition Interface

The robot uses an **SPST main power switch** and **ZX-Switch01 start button**.

---

## 15.1 Final Electrical Unknowns to Resolve

| Item | Required Verification |
|---|---|
| XL4015 loads | Confirm exactly what it powers |
| Camera FOV | 60° documentation vs 80° software calibration |
| Servo current | Conflicting reference values |
| Encoder 836-count unit | Verify PPR / gearbox interpretation |
| Pi loaded rail voltage | Measure during final software |
| Pi branch current | Measure under representative load |
| Motor/control loaded voltage | Measure during acceleration / steering |
| Motor/control branch current | Measure under representative load |

---

# 16. Electrical Reproducibility

The electrical system should be reproduced using this electrical architecture document together with the schematic diagram, physical wiring diagram, controller pin assignments, final measured converter settings, manufacturer documentation, source code, and complete build guide.

---

## 16.1 Minimum Reproduction Map

```text
MAIN POWER

3S LiPo
   |
Main Switch
   |
D1-2 Positive Distribution
   |
   +--> LM2596 --> 5.1 V OUT --> 5.0 V measured at Raspberry Pi input
   |
   +--> XL4015 --> 11.1 V OUT --> 11.0 V measured at Motor / Control input


GROUND

Battery -
LM2596 -
XL4015 -
Controllers / relevant devices
   |
   v
PCT-21 Common Ground
```

---

## 16.2 Controller / Sensor Connections

```text
Raspberry Pi 5
   |
   +--> DFR0566
   |       |
   |       +--> BNO055 via I2C
   |
   +--> Camera via CSI
   |
   +--> RPLiDAR C1 via USB
   |
   +--> Arduino UNO R4 via USB Serial
            |
            +--> Encoder D2 / D3
            |
            +--> Motor D11 / D13
            |
            +--> Servo D9
            |
            +--> Start Switch A0
```

---

## 16.3 Pre-Power Verification

Before connecting sensitive electronics, first complete the power wiring and keep the Raspberry Pi and Arduino disconnected. Connect the battery and measure the LM2596 output, confirming approximately **5.1 V**. Then verify approximately **5.0 V at the Raspberry Pi input path**.

Next, measure the XL4015 output and confirm approximately **11.1 V**, followed by approximately **11.0 V at the motor/control input path**. After the voltage settings and polarity are confirmed, power the system off before connecting the electronics.

The detailed physical build sequence is documented in:

[`../BUILD.md`](../BUILD.md)

---

## 16.4 Electrical Verification Checklist

Before the first autonomous run:

- [ ] Battery voltage is within expected operating range
- [ ] Main switch disconnects the robot correctly
- [ ] LM2596 output is approximately 5.1 V
- [ ] Raspberry Pi input is approximately 5.0 V
- [ ] XL4015 output is approximately 11.1 V
- [ ] Motor/control input is approximately 11.0 V
- [ ] Ground distribution is continuous
- [ ] Raspberry Pi boots without supply warnings
- [ ] Arduino is detected
- [ ] LiDAR is detected
- [ ] Camera opens correctly
- [ ] BNO055 returns changing heading values
- [ ] Encoder counts in both wheel directions
- [ ] Steering servo centers correctly
- [ ] Motor direction matches software command
- [ ] Start button produces the expected start event
- [ ] No connector becomes unusually hot during a short load test
- [ ] Pi rail remains stable while motor and steering are active

---

# 17. References

| Component / Topic | Reference |
|---|---|
| Raspberry Pi 5 | https://www.raspberrypi.com/products/raspberry-pi-5/ |
| Arduino UNO R4 Minima | https://docs.arduino.cc/hardware/uno-r4-minima |
| RPLiDAR C1 | https://www.slamtec.com/en/C1 |
| Gravity BNO055 + BMP280 | https://www.dfrobot.com/product-1793.html |
| DFR0566 IO Expansion HAT | https://wiki.dfrobot.com/dfr0566/docs/22892 |
| Camera | Final camera manufacturer / supplier documentation |
| CHP-20GP-180 | Motor manufacturer / supplier specification |
| LM2596 | https://www.ti.com/product/LM2596 |
| XL4015 | https://www.xlsemi.com/datasheet/XL4015-5A-36V-DC-DC-Converter.pdf |
| L298P Motor Shield | https://www.mouser.com/en/ProductDetail/STMicroelectronics/L298P |

---

# Final Electrical Summary

The final electrical and sensing architecture of YBR-SUNFLOWER developed through several major changes rather than remaining fixed from the first prototype.

```text
Initial Sensor Concept
Camera + Ultrasonic + Light Sensor + IMU
                    |
                    v
      Need richer spatial information
                    |
                    v
          LiDAR Architecture
Camera + LiDAR + IMU + Encoder
                    |
                    v
       Final Physical Rebuild
                    |
                    v
Separated Power + Final Sensor Placement
                    |
                    v
       Version 3 Competition Robot
```

The final design uses one main 3S LiPo battery, separate conversion paths for the computing and motor/control systems, and a common electrical ground. LiDAR provides environmental geometry, the camera provides visual color information, the BNO055 provides a relative heading reference, and the motor encoder provides drivetrain feedback. The DFR0566 organizes Pi-side peripheral wiring, while the LiDAR and other sensors are positioned according to their measurement requirements.

Static multimeter measurements on the completed Version 3 robot confirmed **5.1 V at the LM2596 output and 5.0 V at the Raspberry Pi input**, together with **11.1 V at the XL4015 output and 11.0 V at the motor/control input**.

The battery measures approximately **12.6 V when fully charged**. Based on actual robot testing, drivetrain performance begins to decrease below approximately **11.1 V**, so this value is treated as a practical performance threshold rather than an absolute battery minimum.

The electrical system was therefore designed not only to **power the robot**, but to provide the sensing information and electrical reliability required by the complete autonomous system.

The final engineering process follows:

> **Select → Integrate → Test → Identify Failure → Modify → Validate**

rather than documenting only the final wiring configuration.
