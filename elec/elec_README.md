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

The complete robot uses one **Helicox 3S 1100 mAh LiPo battery** as its primary power source. The original team purchasing documentation identifies the battery as an 11.1 V, 1100 mAh pack. A matching Helicox supplier listing specifies the same 3S / 11.1 V / 1100 mAh configuration with a **30C discharge rating**.

| Specification | Value | Evidence Type |
|---|---:|---|
| Battery chemistry | LiPo | Product specification |
| Configuration | 3S | Product specification |
| Nominal voltage | 11.1 V | Product specification |
| Fully charged voltage | ~12.6 V | Measured / standard 3S LiPo value |
| Capacity | 1100 mAh / 1.1 Ah | Product specification |
| Discharge rating | 30C | Matching Helicox product reference |
| Theoretical current from C-rating | 33 A (`1.1 Ah × 30C`) | Calculated reference only |
| Manufacturer / brand | Helicox | Final robot component |

The 30C figure is a battery product rating rather than a measured continuous-current capability of the complete robot. The robot's practical current is limited by the converters, motor driver, wiring and connectors long before the theoretical 33 A battery figure becomes relevant.

### 3.1.1 Measured Battery Operating Range

The battery voltage changes continuously with state of charge. A fully charged pack measures approximately **12.6 V**.

During robot testing, drivetrain performance was observed to decrease when the battery fell below approximately **11.1 V**, mainly because the drive motor became noticeably slower. Approximately **11.1 V is therefore used as the practical performance threshold of the robot**, not as an absolute LiPo discharge limit.

```text
Fully charged battery             ≈ 12.6 V
Nominal battery voltage           = 11.1 V
Practical robot threshold         ≈ 11.1 V
```

This threshold is an observed system-level behavior of the completed robot.

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

The Raspberry Pi 5 has a load-dependent current requirement. Raspberry Pi documentation lists approximately **800 mA typical bare-board active current** and a **5 A recommended PSU capacity** for full Pi 5 operation and downstream peripherals. The 5 A value is a supply-capability recommendation, not the measured current of this robot.

The completed robot was measured at **5.0 V at the Raspberry Pi input** under static power-on. For documentation of dynamic operation, the following values are engineering estimates based on the Pi 5 reference current together with the camera, RPLiDAR C1, BNO055 and interface loads used by YBR-SUNFLOWER.

| Condition | Pi Rail Voltage | Estimated Current | Estimated Power |
|---|---:|---:|---:|
| Static / low-processing power-on | 5.00 V | ~1.10 A | ~5.50 W |
| LiDAR + camera + IMU active | 4.98 V | ~1.45 A | ~7.22 W |
| Full navigation / vision software | 4.95 V | ~2.20 A | ~10.89 W |
| Short high-compute condition | 4.90 V | ~2.60 A | ~12.74 W |

---

## 3.4 Motor / Control Power Branch — XL4015

The XL4015 supplies the final **motor/control-side power branch**. The completed Version 3 robot was measured at **11.1 V at the XL4015 output** and approximately **11.0 V at the downstream motor/control input**.

```text
XL4015 output          = 11.1 V
Motor/control input    = 11.0 V
Static voltage drop    ≈ 0.1 V
```

**Evidence type: Measured**

The branch feeds the drive/control assembly shown in the wiring diagram. Device-specific voltage requirements are then handled by the motor shield, Arduino power circuitry and associated downstream control wiring rather than treating every device as a direct 11.1 V load.

### 3.4.1 XL4015 Reference Capability

The XL4015 regulator IC and common adjustable XL4015 modules are specified for approximately **5 A maximum output**, with a switching frequency around **180 kHz** and adjustable step-down output. A representative module reference lists 1.25–30 V adjustable output and 5 A maximum current.

These are IC/module ratings, not measured continuous-current values of the robot's exact assembled converter. Thermal conditions, PCB layout, wiring and cooling affect practical capability.

### 3.4.2 Regulation Headroom

Because the XL4015 is a buck converter configured for approximately **11.1 V output**, available regulation headroom decreases as battery voltage approaches 11.1 V. This is consistent with the team's observation that drivetrain speed decreases at low battery voltage.

The exact transition depends on load, battery internal resistance and module losses, so the practical 11.1 V threshold is documented as a robot-level observation rather than a converter cutoff.

### 3.4.3 Estimated Motor / Control Branch Load

| Operating Condition | Estimated Rail Voltage | Estimated Current | Estimated Power |
|---|---:|---:|---:|
| Powered, motor stopped | 11.0 V | ~0.15 A | ~1.65 W |
| Normal straight driving | 10.9 V | ~0.65 A | ~7.09 W |
| Steering while driving | 10.8 V | ~0.95 A | ~10.26 W |
| Acceleration | 10.7 V | ~1.50 A | ~16.05 W |
| Short high-load condition | 10.6 V | ~3.20 A | ~33.92 W |

---

## 3.5 Common Ground Architecture

Although the system uses separate power-conversion branches, the controllers and signal interfaces require a common electrical reference.

The **PCT-21 series connector** is used for the negative / ground distribution. Technical references for the PCT-21 series commonly specify **32 A at 250 V**, with approximately 0.08–4.0 mm² supported conductor range depending on conductor type. The positive side uses a **D1-2 quick wire connector**; the D1-2 technical reference lists **32 A at 250 V**, 0.2–4.0 mm² solid-wire capacity and 0.2–2.5 mm² flexible-wire capacity.

These connector ratings are far above the estimated current of the robot, but the practical installation is still limited by the actual wire gauge, termination quality and mechanical strain.

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

The common reference is particularly important when signals pass between subsystems supplied through different power branches.

---

# 4. Power Budget and Electrical Load Analysis

The electrical design includes relatively stable computing loads and rapidly changing actuator loads. Manufacturer and supplier ratings provide component-level limits, measured values describe the completed static power path, and engineering estimates describe representative autonomous operation where direct current logging was not performed.

## 4.1 Component Electrical Reference

| Component | Supply / Operating Voltage | Reference Current | Peak / Maximum Reference | Evidence Type |
|---|---:|---:|---:|---|
| Raspberry Pi 5 (8 GB) | 5 V class supply | ~800 mA typical bare-board active | 5 A recommended PSU capacity | Raspberry Pi official documentation |
| RPLiDAR C1 | 4.8–5.2 V, 5.0 V typical | 230 mA typical | 260 mA max operating / ~800 mA startup | SLAMTEC datasheet |
| Raspberry Pi OV5647 Camera | CSI camera interface | ~200–250 mA added Pi load | ~250 mA reference | Raspberry Pi camera documentation |
| Gravity BNO055 + BMP280 SEN0253 | 3.3–5 V | 5 mA | — | DFRobot specification |
| DFR0566 IO Expansion HAT | 5 V operating | Board current not separately specified | Peripheral dependent | DFRobot specification |
| Arduino UNO R4 Minima | 5 V logic; 6–24 V VIN | Board current load dependent | 8 mA max per GPIO pin | Arduino official specification |
| CHP-20GP-180, 12 V, 19:1 | 12 V | ≤0.28 A no-load / ≤0.55 A rated | ≤2.7 A stall | Motor supplier specification |
| GEEKSERVO 2KG 360° | 3.3–6 V; 4.8 V rated | 70 mA rated | 700 mA slipping / 900 mA blocked | GeekServo specification |
| L298P Motor Shield | 5–12 V shield supply | Load dependent | 2 A per channel / 4 A max shield reference | Arduino / DFRobot shield specification |
| LM2596 adjustable module | Step-down | Load dependent | 3 A maximum module/IC reference | TI + module reference |
| XL4015 adjustable module | Step-down | Load dependent | 5 A maximum module reference | XL4015 module reference |
| D1-2 quick connector | Distribution | — | 32 A / 250 V reference | D1-2 technical reference |
| PCT-21 series connector | Distribution / common ground | — | 32 A / 250 V reference | PCT-21 technical reference |

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

These values were measured with the robot powered but without an autonomous run.

---

## 4.3 Estimated Dynamic Power Budget

### Computing Branch

| Condition | Voltage | Current | Power |
|---|---:|---:|---:|
| Low-processing idle | 5.00 V | ~1.10 A | ~5.50 W |
| Sensors active | 4.98 V | ~1.45 A | ~7.22 W |
| Full autonomous software | 4.95 V | ~2.20 A | ~10.89 W |
| Short high-compute condition | 4.90 V | ~2.60 A | ~12.74 W |

### Motor / Control Branch

| Condition | Voltage | Current | Power |
|---|---:|---:|---:|
| Motor stopped | 11.0 V | ~0.15 A | ~1.65 W |
| Normal straight driving | 10.9 V | ~0.65 A | ~7.09 W |
| Steering while driving | 10.8 V | ~0.95 A | ~10.26 W |
| Acceleration | 10.7 V | ~1.50 A | ~16.05 W |
| Short high-load condition | 10.6 V | ~3.20 A | ~33.92 W |

The values in this subsection are **engineering estimates** based on published component currents and representative autonomous operation. They are intentionally not labeled as measured results.

---

## 4.4 Combined System Power

Currents from the 5 V and 11 V rails should not be added directly. Electrical power is compared using:

```text
P = V × I
```

A representative full autonomous condition is approximately:

```text
Computing branch      ≈ 10.9 W
Drive/control branch  ≈  7–11 W
--------------------------------
Typical system        ≈ 18–22 W
```

A conservative short design peak is approximately:

```text
Computing branch peak     ≈ 12.7 W
Motor/control peak        ≈ 33.9 W
----------------------------------
Combined short peak       ≈ 46.6 W
```

---

## 4.5 Estimated Battery-Side Current

Assuming approximately **90% overall conversion efficiency** during a short high-load condition:

```text
Battery power ≈ 46.6 W / 0.90
              ≈ 51.8 W
```

At a representative battery voltage of 11.7 V:

```text
I ≈ 51.8 W / 11.7 V
  ≈ 4.4 A
```

A reasonable short battery-side peak estimate is therefore approximately **4–4.5 A**, well below the theoretical 33 A obtained from the battery's 30C product rating.

---

## 4.6 Estimated Battery Runtime

The battery capacity is 1.1 Ah. Using a representative average autonomous current of approximately 2.2 A gives an idealized runtime of about 30 minutes. Allowing for voltage decline, conversion losses and the team's 11.1 V practical performance threshold gives a more realistic engineering estimate of approximately:

> **20–25 minutes of representative autonomous operation per fully charged battery**

This is a calculated engineering estimate rather than a timed endurance-test result.

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

For power-system planning, Raspberry Pi documentation lists approximately **800 mA typical bare-board active current** for Raspberry Pi 5 and recommends a **5 A-capable supply** for full Pi 5 power availability. These values are reference specifications rather than measurements of this robot's complete Pi branch.

---

## 5.2 DFR0566 IO Expansion HAT

The DFR0566 is used as an interface layer on the Raspberry Pi.

It provides organized access to GPIO, I²C and other interfaces, reducing direct loose wiring around the Pi header and making the final sensor connection easier to reproduce.

In the final robot, the BNO055 is connected through the I²C interface provided by this expansion board.

The DFR0566 specification lists **5 V operating voltage**, **3.3 V sensor-interface power**, and an optional **6–12 V external PWM power input**. A standalone current-consumption figure for the complete HAT is not published, so its contribution is treated as part of the measured Raspberry Pi-side branch instead of assigning an unsupported fixed current value.

---

## 5.3 Arduino UNO R4 Minima

The Arduino UNO R4 Minima handles low-level hardware control.

Its responsibilities include drive-motor PWM and direction, steering-servo commands, quadrature encoder input, start-switch detection, and communication with the Raspberry Pi.

This division allows the Arduino to handle hardware timing while the Raspberry Pi focuses on navigation and perception.

The UNO R4 Minima operates at **5 V**. Its official datasheet specifies a maximum of **8 mA per GPIO pin**, so actuators such as the steering servo must not be powered directly from a GPIO output. The total board current depends on the board state and attached hardware and therefore remains part of the system-level current measurement.

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

The RPLiDAR C1 is the primary environmental-geometry sensor. It provides a 360° two-dimensional scan around the robot and is connected to the Raspberry Pi through its USB/UART adapter.

The final software uses the scan to compare the observed environment with the known WRO field geometry for localization. The LiDAR replaced the earlier ultrasonic-sensor concept because a full scan provides substantially richer spatial information than several isolated distance measurements.

### 6.1.1 Electrical and Communication Specification

| Parameter | Specification |
|---|---:|
| Supply voltage | 4.8–5.2 V |
| Typical supply voltage | 5.0 V |
| Typical operating current | 230 mA |
| Maximum operating current | 260 mA |
| Startup current | ~800 mA |
| Typical operating power at 5 V | ~1.15 W calculated from 5 V × 0.23 A |
| Scan frequency | 8–12 Hz, 10 Hz typical |
| Sample rate | 5 kHz |
| Communication | TTL UART through USB adapter in our robot |
| Baud rate | 460800 |

The startup current is significantly higher than the normal operating current. Therefore, the power system must support the LiDAR startup transient, not only its approximately 230 mA typical operating consumption.

---

## 6.2 Raspberry Pi Night Vision Camera

The camera is used primarily for traffic-pillar detection. The original purchasing documentation identifies the module as a **Cytron Fish Eye Lens Raspberry Pi 5MP IR Camera**, product code **RPI-FEYE-5MIRCAM**. The product page specifies an **OV5647 5 MP sensor**, 1/4-inch format, CSI interface and **130° diagonal field of view** for the supplied fish-eye lens.

The team's final vision software uses an approximately **80° horizontal FOV calibration parameter**. These two values are not contradictory because one is a supplier **diagonal optical specification** for the purchased lens configuration and the other is an **effective horizontal software calibration parameter** used by the vision model. The physical camera/lens configuration changed during development, so the software calibration is treated as the operative value for navigation calculations.

| Parameter | Final Documentation Value |
|---|---|
| Sensor | OV5647 |
| Resolution | 5 MP |
| Interface | Raspberry Pi CSI |
| Purchased camera reference | Cytron RPI-FEYE-5MIRCAM |
| Supplier FOV for purchased fish-eye lens | 130° diagonal |
| Working software FOV parameter | ~80° horizontal |
| Camera power contribution | ~200–250 mA added Pi load |

The Raspberry Pi documentation states that a Camera Module adds approximately **200–250 mA** to Raspberry Pi power requirements, so **250 mA** is used as the camera allowance in the power budget.

---

## 6.3 Gravity BNO055 IMU

The final robot uses the **DFRobot Gravity 10 DOF IMU AHRS BNO055 + BMP280 (SEN0253)**. The module combines BNO055 inertial sensing and onboard sensor fusion with BMP280 barometric sensing.

| Parameter | Specification |
|---|---:|
| Operating voltage | 3.3–5 V DC |
| Operating current | 5 mA |
| Interface | Gravity-I²C |
| Operating temperature | -40°C to 80°C |
| BNO055 I²C address | 0x28 |
| BMP280 I²C address | 0x76 |

In the final robot, the module is used primarily as a **relative heading reference** rather than as a fully calibrated absolute magnetic compass. At startup, the software allows the fused orientation output to settle and then treats the initial heading as the local reference.

---

## 6.4 Motor Encoder

The final drivetrain uses the encoder integrated into the **NFP/CHP-20GP-180-EN 12 V planetary gearmotor**. The selected motor is the **19:1** version.

The detailed motor supplier page lists the 12 V / 19:1 configuration as:

| Parameter | Supplier Value |
|---|---:|
| No-load current | ≤0.28 A |
| No-load speed | 780 rpm |
| Rated current | ≤0.55 A |
| Rated speed | 680 rpm |
| Rated torque | 0.4 kg·cm |
| Stall current | ≤2.7 A |
| Stall torque | ≥2 kg·cm |
| Encoder type | AB dual-phase incremental magnetic Hall encoder |
| Base pulse count | 11 PPR |
| Encoder supply | 3.3 V / 5 V |
| Supplier encoder signal for 19:1 version | 211.03 pulses per geared-output revolution |

The supplier explains the encoder line speed as **11 PPR × actual gear-reduction ratio**. The table therefore gives **211.03 pulses per geared-output revolution** for the 19:1 model.

The Arduino firmware historically uses a nominal calculation:

```text
11 PPR × 19 nominal ratio × 4 quadrature edges
= 836 counts per geared-output revolution
```

Using the supplier's more specific 211.03 pulse figure with x4 edge counting would give approximately:

```text
211.03 × 4 ≈ 844 counts per geared-output revolution
```

Therefore, **836 is documented as the firmware calibration constant based on the nominal 19:1 ratio**, while approximately **844 counts/revolution is the supplier-derived theoretical x4 value**. This distinction removes the previous ambiguity without presenting the nominal-ratio calculation as an exact physical encoder constant.

The Raspberry Pi does not use this encoder as its primary field-localization source; LiDAR geometry and heading information remain the main high-level localization inputs.

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

The camera pipeline uses HSV color thresholds for red and green traffic-pillar classification. The purchased camera reference specifies a 130° diagonal fish-eye lens, while the current navigation implementation uses an effective **~80° horizontal FOV software parameter**.

The 80° value is retained as the working vision-model calibration rather than being described as the supplier's optical specification. Camera performance is also affected by ambient lighting, auto exposure, lens position, camera angle and image cropping.

HSV thresholds were therefore tuned using actual camera images rather than ideal RGB values.

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

During subsystem testing, forward motor motion was configured to produce the expected encoder sign and reverse motion the opposite sign. The firmware uses the nominal 836-count calibration constant described in Section 6.4.

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

The camera/lens configuration was changed to provide a wider view. The build documentation identifies a supplier fish-eye camera with a 130° diagonal FOV, while earlier team notes contain another lens value and the software currently uses an 80° horizontal calibration parameter.

Because these values describe different possible lens configurations or different FOV axes, the effective horizontal FOV of the final installed lens still needs to be measured or calibrated directly.

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

## 12.5 Estimated Dynamic Power Behavior

Static voltage measurements establish the actual converter settings. Dynamic current was not logged directly, so representative autonomous behavior is documented using engineering estimates derived from published component specifications.

### Computing Branch

| Condition | Estimated Voltage | Estimated Current | Estimated Power |
|---|---:|---:|---:|
| Sensors active | 4.98 V | ~1.45 A | ~7.22 W |
| Full autonomous software | 4.95 V | ~2.20 A | ~10.89 W |
| Short high-compute condition | 4.90 V | ~2.60 A | ~12.74 W |

### Motor / Control Branch

| Condition | Estimated Voltage | Estimated Current | Estimated Power |
|---|---:|---:|---:|
| Normal straight driving | 10.9 V | ~0.65 A | ~7.09 W |
| Steering while driving | 10.8 V | ~0.95 A | ~10.26 W |
| Acceleration | 10.7 V | ~1.50 A | ~16.05 W |
| Short high-load condition | 10.6 V | ~3.20 A | ~33.92 W |

---

## 12.6 Test Evidence Table

| Test | Problem / Observation | Change | Result |
|---|---|---|---|
| LiDAR mounting | Angled scan distorted geometry | Remounted level | More consistent planar scan |
| Camera view | Original view too narrow | Wider camera/lens configuration used | Larger visible field; final horizontal FOV still requires calibration |
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
| Incorrect converter setting | Component damage or instability | Measure output before connecting electronics |
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
LM2596 output       = 5.1 V
Pi input            = 5.0 V

XL4015 output       = 11.1 V
Motor/control input = 11.0 V
```

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

The high-level computing system consists of the **Raspberry Pi 5 (8 GB)** and **DFR0566 IO Expansion HAT**. The LM2596 output is measured at 5.1 V and the Raspberry Pi input at approximately 5.0 V.

Representative full-autonomous computing-branch load is estimated at approximately **2.2 A / 10.9 W**, with a short high-compute estimate of approximately **2.6 A / 12.7 W**.

## Low-Level Control

Low-level actuator and encoder control is handled by the **Arduino UNO R4 Minima**, communicating with the Raspberry Pi through USB serial at 115200 baud.

## Environmental Sensors

The robot uses the **RPLiDAR C1** for field geometry and the **OV5647 5 MP IR camera** for visual traffic-pillar information. The LiDAR is specified at 230 mA typical operating current, 260 mA maximum operating current and approximately 800 mA startup current. The camera power budget uses a 250 mA reference allowance.

## Orientation

The **DFRobot SEN0253 BNO055 + BMP280** module provides the relative heading reference and is specified at approximately 5 mA operating current.

## Motion Feedback

The **20GP-180 AB Hall encoder** provides motor-rotation feedback. The supplier lists 211.03 pulses per geared-output revolution for the 19:1 version; the firmware uses an 836-count x4 calibration based on the nominal 19:1 ratio.

## Actuation

The drive motor is the **12 V NFP/CHP-20GP-180-EN, 19:1** version, specified at ≤0.28 A no-load, ≤0.55 A rated and ≤2.7 A stall. Steering uses the **GEEKSERVO 2KG 360°**, specified at 70 mA rated, 700 mA slipping and 900 mA blocked-rotor current.

## Motor Control

Drive-motor control is provided by an **L298P-based Motor Shield**. The reference shield specification supports 5–12 V operation and up to 2 A per channel / 4 A total at the shield-design level.

## Main Power

The main energy source is a **Helicox 3S 11.1 V 1100 mAh 30C LiPo**. It measures approximately 12.6 V when fully charged and approximately 11.1 V is used as the practical drivetrain-performance threshold.

## Power Conversion

The Raspberry Pi branch uses an **LM2596 adjusted to 5.1 V**, with approximately 5.0 V measured at the Pi input. The motor/control branch uses an **XL4015 adjusted to 11.1 V**, with approximately 11.0 V measured at the downstream motor/control input.

## Power Distribution

Positive distribution uses the **D1-2** quick connector and common negative / ground uses the **PCT-21** connector. Both connector families have technical references around 32 A / 250 V, well above the estimated current of this robot.

## Competition Interface

The robot uses an **SPST main power switch** and the **INEX ZX-Switch01** as the competition start input.

---

## 15.1 Evidence Status

The final electrical documentation contains no unresolved placeholder values. Quantitative information is separated by evidence type:

| Evidence Type | Examples in This Document |
|---|---|
| Direct team measurement | 12.6 V full battery, 5.1 V LM2596 output, 5.0 V Pi input, 11.1 V XL4015 output, 11.0 V motor/control input |
| Manufacturer / supplier specification | Pi current reference, LiDAR current, BNO055 current, motor current/torque, servo current, connector ratings |
| Calculated value | 0.1 V static wiring drops, battery 33 A theoretical C-rating current, branch power values |
| Engineering estimate | Dynamic rail current/voltage, typical 18–22 W system power, ~46.6 W short design peak, 20–25 min runtime |

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

## 16.4 Expected Electrical Behavior

A correctly reproduced electrical system should show the following behavior:

| System | Expected Behavior |
|---|---|
| Main switch | Powers or disconnects the complete robot |
| Battery | ~12.6 V fully charged; performance begins to decline near the team's ~11.1 V threshold |
| LM2596 | ~5.1 V converter output |
| Raspberry Pi input | ~5.0 V static input |
| XL4015 | ~11.1 V converter output |
| Motor/control input | ~11.0 V static downstream input |
| Ground distribution | Common reference through PCT-21 |
| Raspberry Pi | Boots normally and initializes high-level software |
| Arduino | Appears through USB serial and accepts commands |
| RPLiDAR C1 | Starts and streams scan data at the configured serial rate |
| Camera | Provides CSI image frames for color processing |
| BNO055 | Provides changing fused heading values |
| Encoder | Changes count with motor rotation and direction |
| Steering servo | Responds to commanded steering position |
| Drive motor | Direction and PWM response match software command |
| ZX-Switch01 | Generates the competition start event |

---

# 17. References

| Component / Topic | Reference | Reference Role |
|---|---|---|
| Raspberry Pi 5 | https://www.raspberrypi.com/documentation/computers/raspberry-pi.html | Official power requirements and Pi 5 current reference |
| Raspberry Pi 5 (8 GB) team purchase model | https://gammaco.com/gammaco/Raspberry_Pi_GB_89RD014.html | Team purchase / model confirmation |
| Arduino UNO R4 Minima | https://docs.arduino.cc/hardware/uno-r4-minima | Official board specification |
| RPLiDAR C1 | https://www.dfrobot.com/product-2803.html | Exact purchased model page and LiDAR specifications |
| RPLiDAR C1 electrical datasheet | https://www.slamtec.com/en/C1 | Manufacturer product / datasheet source |
| Raspberry Pi 5 MP IR Camera | https://th.cytron.io/p-fish-eye-lense-raspberry-pi-5mp-ir-camera | Team purchase source; OV5647, 5 MP, 130° diagonal fish-eye reference |
| Raspberry Pi Camera power requirement | https://www.raspberrypi.com/documentation/computers/camera_software.html | Official ~200–250 mA camera power contribution |
| Gravity BNO055 + BMP280 SEN0253 | https://wiki.dfrobot.com/sen0253/ | Exact module voltage, current and interface specifications |
| DFR0566 IO Expansion HAT | https://wiki.dfrobot.com/dfr0566 | Exact HAT voltage, interfaces and dimensions |
| 20GP-180-EN Motor + Encoder | https://microdcmotors.com/product/20mm-6v-12v-dc-planetary-geared-motor-with-encoder-model-nfp-20gp-180-en | Detailed 12 V / 19:1 motor, torque, current and encoder specifications |
| GEEKSERVO 2KG 360° Servo | https://kittenbothk-eng.readthedocs.io/en/latest/motors/2kgServo.html | Servo voltage, current, torque and range specifications |
| L298P Motor Shield reference design | https://store-usa.arduino.cc/collections/maker-solutions/products/arduino-motor-shield-rev3 | Official L298P shield design specifications |
| L298P Motor Shield matching product | https://www.dfrobot.com/product-1395.html | Matching L298P dual-H-bridge shield specifications |
| Helicox 3S 11.1 V 1100 mAh 30C LiPo | https://www.udshobby.com/product/380/แบต-helicox-3s-11-1v-30c-มีให้เลือกหลายขนาด-900-1100-1500-2200-3000-3500mah | Matching exact battery capacity / voltage / C-rating reference |
| LM2596 regulator | https://www.ti.com/product/LM2596 | Official regulator IC specification |
| LM2596 adjustable module reference | https://www.sunrom.com/p/dc-dc-step-down-switching-regulator-based-on-lm2596 | Matching 3 A adjustable module characteristics |
| XL4015 adjustable module reference | https://www.phippselectronics.com/product/xl4015-5a-adjustable-cc-cv-step-down-dc-power-supply-module/ | Matching 5 A adjustable module characteristics |
| D1-2 quick wire connector | https://itead.cc/product/sonoff-quick-wire-connectors/ | D1-2 32 A / 250 V and conductor-size reference |
| PCT-21 series connector | https://www.cxdefa.com/user-manual/instructions-sublink/push-wire-conductor-2.html | PCT-21 series conductor-size / connection reference |
| PCT-21 electrical rating reference | https://telehan.en.made-in-china.com/product/KRtrayOlZfhZ/China-Pct-21-Series-Quick-Wiring-Connector-with-Block-Lever-Push-in-Conductor-Terminal-Block.html | 32 A / 250 V PCT-21 series reference |
| ZX-Switch01 | https://inex.co.th/home/product/zx-switch01/ | Official INEX start-switch module description |
| SPST Main Power Switch | Team wiring diagram and physical robot | Generic SPST component; no model-specific electrical specification is used in calculations |

---

# Final Electrical Summary

The final electrical and sensing architecture of YBR-SUNFLOWER developed through multiple iterations rather than remaining fixed from the first prototype.

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

The completed robot uses one Helicox 3S LiPo battery and separates the Raspberry Pi computing supply from the motor/control conversion path while maintaining a common signal ground.

Static multimeter testing measured:

```text
Battery fully charged     ≈ 12.6 V

LM2596 output             = 5.1 V
Raspberry Pi input        = 5.0 V

XL4015 output             = 11.1 V
Motor/control input       = 11.0 V
```

The battery reference is 11.1 V, 1100 mAh and 30C. The robot begins to show lower drivetrain performance near approximately 11.1 V, so that value is used as the practical performance threshold.

Published component specifications establish the important reference loads: Raspberry Pi 5 ~800 mA bare-board active, RPLiDAR C1 230 mA typical / ~800 mA startup, camera ~200–250 mA added load, SEN0253 ~5 mA, 20GP-180 ≤0.55 A rated / ≤2.7 A stall, and GeekServo 70 mA rated / 900 mA blocked-rotor.

Based on those specifications and representative autonomous operation, the complete robot is estimated to use approximately **18–22 W during typical autonomous driving**, with a conservative short design peak around **46.6 W** and approximately **4–4.5 A estimated battery-side peak current**.

The 1100 mAh battery therefore gives an estimated practical autonomous operating time of approximately **20–25 minutes**, depending on motor loading, steering activity, processing workload and battery condition.

The motor supplier reference also resolves the encoder terminology: the 12 V / 19:1 model lists **211.03 encoder pulses per geared-output revolution**. The firmware's **836-count** constant is the x4 quadrature value calculated from the nominal 11 PPR × 19:1 ratio, while the supplier-derived x4 theoretical figure is approximately **844 counts/revolution**.

All quantitative values in this document are identified as **measured, manufacturer/supplier reference, calculated, or engineering estimate**. No estimated value is presented as a direct experimental measurement.

The final engineering process follows:

> **Select → Integrate → Test → Identify Failure → Modify → Validate**

The electrical documentation therefore describes not only how the robot is wired, but also the engineering reasoning, component limits, power budget, failure considerations and reproducibility of the final electrical architecture.
