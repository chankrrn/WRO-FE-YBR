# WRO-FE-YBR-SUNFLOWER

## YBR-SUNFLOWER  — WRO 2026 Future Engineers

Welcome to **YBR-SUNFLOWER's documentation** for the **WRO 2026 Future Engineers** competition.

We are a three-member team from the **Science–Mathematics English Program at Yothinburana School, Thailand**. Our project is an autonomous self-driving vehicle designed to navigate the WRO Future Engineers competition field using a combination of computer vision, distance sensing, orientation feedback, motor control, and autonomous decision-making.

This repository contains our robot's mechanical design, electrical and sensing architecture, software, CAD files, wiring documentation, and development records.

---

# Contents

* [Our Team](#our-team)
* [Our Robot](#our-robot)
* [Mobility and Mechanical Design](#mobility-and-mechanical-design)
* [Power and Sensor Architecture](#power-and-sensor-architecture)
* [Software Architecture and Obstacle Management](#software-architecture-and-obstacle-management)
* [System Thinking and Engineering Decisions](#system-thinking-and-engineering-decisions)
* [Build / Compile / Upload](#build--compile--upload)
* [Repository Structure](#repository-structure)

---

# Our Team

We are **YBR-SUNFLOWER**, a team from the Science–Mathematics English Program at Yothinburana School, Thailand.

Our team consists of three students:

* **Peradon Nimsongprasert**
* **Chanakarn Yimsakul**
* **Thanphisit Sakulvitulthai**

Our mentor is **Punnapon Tanasnitikul**.

We were brought together by our interest in robotics and enjoy learning, solving problems, and turning our ideas into working systems through competition.

### Team Roles

> Roles are described according to each member's contribution to the development process.

| Member                    | Main Responsibility |
| ------------------------- | ------------------- |
| Peradon Nimsongprasert    | [ADD ROLE]          |
| Chanakarn Yimsakul        | [ADD ROLE]          |
| Thanphisit Sakulvitulthai | [ADD ROLE]          |

---

# Our Robot

<table>
  <tr>
    <td align="center"><strong>Top View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/top.jpg" width="300"></td>
    <td align="center"><strong>Front View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/front.jpg" width="300"></td>
    <td align="center"><strong>Left View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/left.jpg" width="300"></td>
  </tr>
  <tr>
    <td align="center"><strong>Bottom View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/bottom.jpg" width="300"></td>
    <td align="center"><strong>Back View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/back.jpg" width="300"></td>
    <td align="center"><strong>Right View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/right.jpg" width="300"></td>
  </tr>
</table>

<img src="https://github.com/chankrrn/WRO-FE-YBR/blob/b513354a630ec0d0f7143e4fd617df6cad310518/other/ComponentsImage1.png" width="700" height="400">

<img src="https://github.com/chankrrn/WRO-FE-YBR/blob/b513354a630ec0d0f7143e4fd617df6cad310518/other/ComponentsImage2.png" width="700" height="400">

<img src="https://github.com/chankrrn/WRO-FE-YBR/blob/b513354a630ec0d0f7143e4fd617df6cad310518/other/ComponentsImage3.png" width="700" height="400">

<img src="https://github.com/chankrrn/WRO-FE-YBR/blob/b513354a630ec0d0f7143e4fd617df6cad310518/other/ComponentsImage4.png" width="700" height="400">

<img src="https://github.com/chankrrn/WRO-FE-YBR/blob/6b0eed2ec69b003c0b4ab057b13127d00ccb34aa/other/ComponentsImage5.png" width="700" height="400">


## Overview

Our robot is a compact four-wheeled autonomous vehicle designed for the WRO Future Engineers challenge.

The robot combines:

* A Raspberry Pi 5 for high-level computation
* An Arduino UNO R4 Minima for low-level motor and steering control
* A DC geared motor with encoder for drivetrain control
* A servo motor for steering
* LiDAR for distance and environmental information
* A camera for visual detection
* An IMU for orientation and heading information
* A start button for autonomous competition operation

The final robot has an approximate dimension of:

**230 × 140 × 130 mm**

The complete mechanical and electrical design is documented in the `mech` and `elec` directories.

---

## Robot Photos

The `robot-photos` directory contains photographs of the robot from multiple directions, including top and bottom views.

[Add robot photos here.]

---

## Performance Videos

### Test Run

[Add test video link]

### Open Challenge

[Add Open Challenge video link]

### Obstacle Challenge

[Add Obstacle Challenge video link]

---

# Mobility and Mechanical Design

## Robot Design

Our vehicle uses a four-wheel automotive-style configuration with a dedicated drivetrain and steering mechanism.

The main mechanical design includes:

* **Drive Motor:** CHP-20GP-180 DC geared motor with dual-phase encoder
* **Steering:** GEEKSERVO 2 kg 360° servo
* **Wheels:** LEGO Tire 43.2 × 22 ZR and 30.4 mm reinforced-rim wheel
* **3D-Printed Structures:** Custom components designed and produced for our vehicle

Detailed CAD models and mechanical documentation can be found in the `mech` directory.

[Add robot overview image / 3D model image here.]

---

## Drivetrain Philosophy

One of our main mechanical design decisions was to prioritize **precision, stability, and controllability over maximum speed**.

We selected a higher gear ratio to provide more available torque. Although this reduces maximum speed, the additional torque allows the robot to move more reliably at low speed and improves control during precise maneuvers.

During prototype testing, we found that insufficient torque made low-speed movement and acceleration from a stop difficult to control. This led us to further optimize the drivetrain and gearbox.

---

## Motor Selection

<img width="400" alt="image" src="https://github.com/user-attachments/assets/a1ab7c62-66f1-469f-9876-43a2f6fce361" />

We selected the **CHP-20GP-180**, a brushed DC motor with a dual-phase quadrature encoder.

The encoder allows us to measure motor rotation and provides feedback for more precise motor control.

We considered two gearbox configurations:

| Gear Ratio | Maximum Speed |
| ---------- | ------------: |
| 1:5        |      1350 RPM |
| 1:19       |       390 RPM |

We selected the **1:19 configuration** because our design philosophy prioritizes torque, precision, and stability over maximum speed.

---

## Gearbox Development

<image src=https://github.com/chankrrn/WRO-FE-YBR/blob/main/mech/models/Powertrains.png width = "800">

During the prototype stage, we used a **28:28 gearbox**, giving a **1:1 ratio**.

Testing showed that the available torque was too low for our desired low-speed behavior. The robot struggled to accelerate smoothly from a stop and was difficult to control accurately at low speeds.

For the final design, we developed a custom **3D-printed 21-tooth gear** that could connect directly to the motor without requiring the additional gear adapter used in the prototype.

The final gearbox uses a **21:28 ratio**.

Although the change appears relatively small, testing showed a significant improvement in low-speed controllability and overall precision.

Detailed mechanical development and CAD files are available in:

[`mech/mech_README.md`](mech/mech_README.md)

---

# Power and Sensor Architecture

The robot uses a combination of distance sensing, computer vision, orientation sensing, encoder feedback, and a physical start button.

## Sensors

### RPLiDAR C1

The LiDAR provides 2D distance measurements around the robot and is used to obtain information about walls, traffic signs, and the parking area.

### Raspberry Pi Night Vision Camera

The camera is used for visual detection, including identification of colored traffic signs and parking-related visual information.

### Gravity BNO055 IMU

The BNO055 provides orientation and heading information and is used to support stable motion and heading control.

### Motor Encoder

The CHP-20GP-180 encoder provides feedback about motor rotation, speed, and direction.

### ZX-Switch01

The external switch is used as the robot's competition start button.

---

## Computing Architecture

### Raspberry Pi 5

The Raspberry Pi 5 is the main high-level computing platform.

It processes information from the LiDAR, camera, and IMU and determines the robot's navigation and driving commands.

### Arduino UNO R4 Minima

The Arduino UNO R4 Minima performs lower-level control of:

* Drive motor
* Steering servo
* Encoder feedback
* Start button

This separation allows high-level processing and low-level motor control to be handled independently.

---

## Power Architecture

Our electrical system is powered by a **Helix 1100 mAh 11.1 V 3S LiPo battery**.

The battery supply is divided into separate regulated power branches for the computing system and the motor/control system.

### Raspberry Pi Branch

The Raspberry Pi is supplied through an **LM2596 step-down converter**.

The output is tuned to approximately **5.1 V** to compensate for voltage losses through wiring and connectors.

### Motor / Control Branch

The motor-side system uses an **XL4015 step-down converter** and an **L298P Motor Shield** connected to the Arduino UNO R4 Minima.

The purpose of separating the power branches is to reduce the effect of motor current changes on the Raspberry Pi and improve system stability.

Detailed electrical architecture, wiring, power calculations, component information, and calibration methods are documented in:

[`elec/elec_README.md`](elec/elec_README.md)

---

# Software Architecture and Obstacle Management

The robot's software is divided between high-level processing on the Raspberry Pi and low-level control on the Arduino UNO R4 Minima.

The Raspberry Pi processes sensor information and determines the robot's navigation behavior.

The Arduino receives control commands and handles the drivetrain, steering, encoder feedback, and start sequence.

The final software architecture and obstacle-management strategy are documented in:

[`src/`](src/)

and will be described in greater detail in:

[`software/software_README.md`](software/software_README.md)

[Add software architecture diagram / state machine here.]

---

# System Thinking and Engineering Decisions

Our robot was developed as a complete system rather than as independent mechanical, electrical, and software components.

Mechanical, electrical, and software decisions affect one another.

For example:

* The drivetrain gear ratio affects acceleration, low-speed control, and the behavior required from the software.
* Sensor selection determines what information is available to the navigation system.
* Power distribution affects the reliability of the Raspberry Pi and motor system.
* Encoder feedback allows the control system to use motor movement information for more precise operation.
* The computing architecture determines how sensor data can be processed and converted into control commands.

One of our main design philosophies was to prioritize **precision, stability, and repeatability over maximum speed**.

This philosophy influenced the selection of the drivetrain gear ratio, motor configuration, gearbox design, sensor architecture, and control strategy.

The development process was iterative. We built prototypes, tested them, identified problems, and modified the design based on the observed behavior of the robot.

Detailed engineering decisions are documented throughout the `mech`, `elec`, and software documentation.

---

# Build / Compile / Upload

## Hardware

The robot requires the following main hardware:

* Raspberry Pi 5
* Arduino UNO R4 Minima
* DFRobot Pi OI Expansion HAT
* L298P Motor Shield
* CHP-20GP-180 DC geared motor with encoder
* GEEKSERVO steering servo
* RPLiDAR C1
* Raspberry Pi Camera
* BNO055 IMU
* ZX-Switch01
* 11.1 V 3S LiPo battery
* LM2596 step-down converter
* XL4015 step-down converter

## Source Code

The robot's source code is stored in:

[`src/`](src/)

## Mechanical Files

Mechanical design files and 3D-printable components are stored in:

[`mech/`](mech/)

## Electrical Files

Electrical diagrams and documentation are stored in:

[`elec/`](elec/)

and:

[`schemes/`](schemes/)

> Detailed build, configuration, and upload instructions will be added as the final development environment is documented.

---

# Repository Structure

```text
WRO-FE-YBR/
│
├── elec/
│   └── elec_README.md
│
├── mech/
│   ├── models/
│   └── mech_README.md
│
├── other/
│
├── robot-photos/
│
├── schemes/
│   ├── README.md
│   ├── Schematic Diagram.png
│   └── Wiring Diagram.png
│
├── src/
│
├── team-photos/
│
├── video/
│
├── LICENSE.md
└── README.md
```

---

# Conclusion

YBR's WRO Future Engineers robot was developed through an iterative engineering process combining mechanical design, electronics, sensing, and software.

Our main design philosophy is to prioritize **precision, stability, reliability, and repeatability** rather than maximizing a single performance factor such as top speed.

The documentation in this repository records the design and development of the robot and provides the technical resources required to understand and reproduce the system.

As development continues, additional testing results, software documentation, and final competition materials will be added to the repository.
****
