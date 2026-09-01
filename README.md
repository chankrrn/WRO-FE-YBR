# *YBR-SUNFLOWER 🌻*

<div align="center">

## WRO 2026 Future Engineers

### Official Engineering Documentation

**Yothinburana School — Science–Mathematics English Program**  
**Thailand**

<br>

<img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/8fcf201d1934eadb0b708aad52bb1ed08f38afb4/robot-photos/Landscape.JPG" width="500" height="500">

<br><br>


**Autonomous Self-Driving Vehicle · Mechanical Engineering · Electrical & Sensor Architecture · Localization · Computer Vision · Autonomous Navigation**

</div>

---

## Documentation Quick Links

| Documentation | Purpose |
|---|---|
| [Mechanical Design](mech/mech_README.md) | Chassis, drivetrain, steering, CAD, materials, mechanical calculations, trade-offs and iterations |
| [Electrical & Sensor System](elec/elec_README.md) | Power architecture, controllers, sensors, wiring, calibration, interfaces and reliability |
| [Software Architecture](software/software_README.md) | Localization, path planning, Pure Pursuit, computer vision, sensor fusion, obstacle strategy and parking |
| [Build & Reproduction Guide](BUILD.md) | Complete assembly, wiring, software setup, upload, verification and troubleshooting procedure |
| [Source Code](src/) | Raspberry Pi and Arduino competition software |
| [Schematics](schemes/) | Electrical schematic and physical wiring diagrams |
| [Mechanical Models](mech/models/) | CAD and 3D-printable mechanical parts |
| [Robot Photos](robot-photos/) | Final robot photographs |
| [Engineering Process](engineering-process/) | Development, assembly, soldering, coding, testing and prototype evidence |
| [Videos](video/) | Test and competition demonstration videos |
| [Version History](CHANGELOG.md) | Major engineering versions and repository release notes |

---

# Contents

1. [Our Team](#1-our-team)
2. [Project Overview](#2-project-overview)
3. [Final Robot](#3-final-robot)
4. [Engineering Development Process](#4-engineering-development-process)
5. [System Architecture](#5-system-architecture)
6. [Mobility and Mechanical Engineering](#6-mobility-and-mechanical-engineering)
7. [Power and Sensor Architecture](#7-power-and-sensor-architecture)
8. [Software Architecture and Autonomous Strategy](#8-software-architecture-and-autonomous-strategy)
9. [Systems Thinking and Major Engineering Decisions](#9-systems-thinking-and-major-engineering-decisions)
10. [Testing, Validation and Failure Analysis](#10-testing-validation-and-failure-analysis)
11. [Build and Reproducibility](#11-build-and-reproducibility)
12. [Repository Structure](#12-repository-structure)
13. [Version History and Development Milestones](#13-version-history-and-development-milestones)
14. [WRO Engineering Documentation Map](#14-wro-engineering-documentation-map)
15. [References and Related Documentation](#15-references-and-related-documentation)

---

# 1. Our Team

## 1.1 Team Members

<div align="center">
  
<img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/e39aad0b6cc114cde8bbbf52a57e2f17c71f2b99/team-photos/t_pic.jpg" width="700">

</div>

We are **YBR-SUNFLOWER**, representing Yothinburana School, Thailand.

| Member | Primary Responsibility | Supporting Responsibilities |
|---|---|---|
| **Peradon Nimsongprasert** | Robot construction and software development | Robot testing, debugging, and performance optimization |
| **Chanakarn Yimsakul** | Documentation and technical writing | Organizing project information, recording development progress, and supporting software development |
| **Thanphisit Sakulvitulthai** | Hardware development and electrical systems | Supporting documentation, hardware testing, and robot assembly |

**Mentor:** Mr. Punnapon Tanasnitikul

---

## 1.2 How We Work as a Team

Our development process integrates mechanical, electrical, and software work throughout the development. A mechanical change can affect steering behavior, while a sensor-placement change can affect localization. Similarly, a power-system change can affect computing reliability, and a software change can reveal mechanical limitations that were not clear during static testing. 

Therefore, subsystem decisions are developed and tested as part of the complete robot rather than independently until the final stage.

Our team divides the work based on each member’s main area of responsibility. Peradon focuses on building the robot and developing the code, Chanakarn is mainly responsible for documentation, and Thanphisit focuses on hardware development while also supporting the documentation. We regularly discuss changes and test the robot together to make sure that mechanical, hardware, and software improvements work well as a complete system.

---

# 2. Project Overview

## 2.1 About YBR-SUNFLOWER

**YBR-SUNFLOWER** is an autonomous self-driving vehicle developed for the **WRO 2026 Future Engineers** competition by a three-member student team from the Science–Mathematics English Program at Yothinburana School, Thailand.

The vehicle combines mechanical design, embedded control, electrical engineering, computer vision, LiDAR localization, orientation sensing and autonomous navigation into one integrated system.

The robot was not developed as a set of independent components. Mechanical, electrical and software decisions were repeatedly changed together as testing revealed limitations in earlier designs.

Our main engineering priorities are:

1. **Precision**
2. **Stability**
3. **Reliability**
4. **Repeatability**
5. **Reproducibility**

We deliberately prioritize predictable autonomous behavior over maximum theoretical speed.

---

## 2.2 Competition Challenge

The WRO Future Engineers competition requires the vehicle to operate fully autonomously on a field whose configuration is not completely known before the round begins.

The competition contains two major driving challenges.

### Open Challenge

The vehicle must autonomously complete three laps while responding to the randomized track configuration and driving direction.

**Open Challenge Performance Video:**  `[TODO: OPEN CHALLENGE VIDEO]`


### Obstacle Challenge

The vehicle must:

1. complete three autonomous laps,
2. detect red and green traffic pillars,
3. pass each traffic pillar on the correct side,
4. adapt to randomized obstacle positions,
5. identify the parking situation,
6. and perform the required parallel-parking maneuver.

The starting position, driving direction and field configuration may change between rounds.

For this reason, our final software does not depend on one fixed prerecorded path from one fixed starting point.

**Obstacle Challenge Performance Video:**  `[TODO: OPEN CHALLENGE VIDEO]`

> **All the videos including additional testing videos can be found in:** [`video/RunTest_video.md`](https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/e39aad0b6cc114cde8bbbf52a57e2f17c71f2b99/video/RunTest_video.md)


---

## 2.3 Competition Constraints That Influenced Our Design

| Constraint | Engineering Response |
|---|---|
| Fully autonomous operation | All sensing, localization, decision-making and actuator control run onboard |
| Randomized field configuration | LiDAR-based localization and field-coordinate navigation |
| Randomized driving direction | Path direction can be selected during initialization |
| Red / green traffic pillars | Camera-based color recognition and obstacle mapping |
| Parallel parking requirement | Distance-based movement and dedicated parking sequence |
| Maximum robot dimensions | Compact stacked mechanical and electrical layout |
| Three-minute run limit | Efficient initialization and autonomous execution |
| One main power switch and one competition start button | Separate SPST main switch and ZX-Switch01 start control |
| Limited onboard power | Single 3S LiPo with separated power branches |
| Limited physical space | Layered chassis and compact sensor placement |

---

## 2.4 Engineering Design Philosophy

A major engineering decision throughout this project was:

> **Precision, stability and repeatability are more important to our robot than maximum speed.**

This principle influenced the complete system. Mechanically, it affected motor selection, drivetrain reduction, Ackermann-style steering, bearing support and chassis rigidity. Electrically, it influenced sensor placement and the decision to separate the Raspberry Pi power path from actuator-related loads.

The same philosophy also influenced software development. Localization, path tracking, steering control and parameter tuning were designed around predictable behavior rather than aggressive speed. The final result is a vehicle intended to perform the same maneuver consistently instead of being optimized only for its fastest possible run.


---

# 3. Final Robot

## 3.1 Robot at a Glance

| Category | Final Configuration |
|---|---|
| Robot type | Four-wheel automotive-style autonomous vehicle |
| Main computer | Raspberry Pi 5 |
| Low-level controller | Arduino UNO R4 Minima |
| Drive motor | CHP-20GP-180 brushed DC geared motor with quadrature encoder |
| Internal motor gearbox | 19:1 reduction |
| External drivetrain | 16-tooth motor gear → 28-tooth differential gear |
| Steering | Servo-driven Ackermann-style steering |
| Steering servo | GEEKSERVO 2 kg 360° servo |
| Distance sensing | RPLiDAR C1 |
| Vision | Raspberry Pi Night Vision Camera |
| Orientation sensing | Gravity BNO055 IMU |
| Drive feedback | Dual-phase motor encoder |
| Pi I/O interface | DFR0566 IO Expansion HAT |
| Motor driver | L298P Motor Shield |
| Main battery | 11.1 V 3S LiPo, 1100 mAh |
| Power architecture | Separate computing and motor/control branches |
| Final dimensions | Approximately **230 × 140 × 130 mm** |
| Final weight | **[TODO: Measure final competition weight]** |
| Final physical version | Version 3 |

---

## 3.2 Final Robot — Six-View Documentation

The following photographs document the final competition robot from all required directions.

| Front View | Right View | Rear View |
|---|---|---|
| <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Front.png" width="250"> | <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Right.png" width="250"> | <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Rear.png" width="250"> |


| Left View | Top View | Bottom View |
|---|---|---|
| <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Left.png" width="250"> | <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Top.png" width="250"> | <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/b80bfdc62db4c16ad0f3b1d6b3734f01582681df/robot-photos/Bottom.png" width="250"> |

| Right-Front View | Right-Rear View | Left-Front View | Left-Rear View |
|---|---|---|---|
| <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Right%20to%20Front.png" width="187.5"> | <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Right%20to%20Rear.png" width="187.5"> | <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Left%20to%20Front.png" width="187.5"> | <img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/4d5d33f8a46e3297b6dbb8371d7695a194f6d349/robot-photos/Left%20to%20Rear.png" width="187.5">



## 3.3 Annotated Final Robot Layout

This section should will allow you to understand the physical system without first reading every subsystem document.

### Overall Component Layout

<img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/8fcf201d1934eadb0b708aad52bb1ed08f38afb4/robot-photos/Layout1.png">

<img src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/8fcf201d1934eadb0b708aad52bb1ed08f38afb4/robot-photos/Layout2.png">

### Mechanical Layout

> **[TODO: Add annotated mechanical layout / drivetrain image]**

Recommended labels:

- front steering assembly,
- Ackermann linkage,
- servo,
- front axle,
- drive motor,
- 16T drive gear,
- differential,
- rear axle,
- bearings,
- wheelbase,
- track width.

### Internal Electronics Layout

> **[TODO: Add photograph of the robot with covers / upper structure removed so that wiring and controllers are visible.]**

Detailed mechanical and electrical layout documentation is available in:

- [`mech/mech_README.md`](mech/mech_README.md)
- [`elec/elec_README.md`](elec/elec_README.md)

---

# 4. Engineering Development Process

The final robot is the result of multiple design iterations.

Our development followed the repeated engineering cycle:

```text
PLAN
  ↓
BUILD
  ↓
TEST
  ↓
IDENTIFY PROBLEM
  ↓
MODIFY
  ↓
TEST AGAIN
```

We track and document major physical architecture changes using version numbers across each development stage of development.

---

## 4.1 Development Timeline

| Version | Mechanical Development | Electrical / Sensor Development | Software Development | Main Outcome |
|---|---|---|---|---|
| **V1 — Initial Prototype** | Initial printed chassis and 1:1 external drivetrain | Ultrasonic + light-sensor sensing concept | Arduino-side control; complete Raspberry Pi autonomous driving not yet implemented | Initial hardware concept and drivetrain testing |
| **V2 — LiDAR Prototype** | 21T drive gear, Ackermann steering, bearings and expanded structure | LiDAR introduced; ultrasonic and light sensor removed; IMU relocated | Navigation development and tuning became the main focus | More complete autonomous sensing architecture |
| **V3 — Final Robot** | Robot rebuilt, 16T drive gear, stronger rear structure and final component layout | Final sensor placement, final wiring and separated power architecture | Localization, racing line, Pure Pursuit, obstacle mapping and competition software | Current competition platform |

> V1 did not have a complete Raspberry Pi autonomous competition run. We therefore do not claim autonomous performance results for that version.

---

## 4.2 Prototype Comparison

### Version 1

<img width="250" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/5e710e7e09354c6d071cfc761c1bd883adb1747e/robot-photos/ver-1/v1-Front.webp">

**Main purpose:** establish the initial drivetrain, steering and sensing concept.

### Version 2

<img width="250" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/5e710e7e09354c6d071cfc761c1bd883adb1747e/robot-photos/ver-2/v2-Front.webp">

**Main purpose:** integrate LiDAR-based sensing and begin complete autonomous navigation development.

### Version 3

<img width="250" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/620ed8f2468a557cd779440bb241dcd40da863fb/robot-photos/Front.png">

**Main purpose:** final mechanical rebuild and integrated competition architecture.

---

## 4.3 Team Engineering Process

### Mechanical Assembly

> **[TODO: Add photograph of the team assembling the chassis / drivetrain / steering system]**

Recommended evidence:

- drivetrain assembly,
- bearing installation,
- steering assembly,
- 3D-printed part fitting,
- mechanical revision.

### Electronics Assembly and Soldering

<img width="400" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/d517b8e67f0eb4ee75be5f6ee4206786ad332d23/team-photos/elec_solder.png">

<img width="400" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/d517b8e67f0eb4ee75be5f6ee4206786ad332d23/team-photos/elec_moresolder.png">

### Software Development

<img width="400" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/db587b3524622d05e9146c5895c82e2466aeb306/team-photos/software_process.png">

<img width="400" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/bd6e92daaf74e9fd8a30c49821987b749f3e22e1/team-photos/software_moreprocess.png">

### Robot Testing

[`video/RunTest_video.md`](https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/e39aad0b6cc114cde8bbbf52a57e2f17c71f2b99/video/RunTest_video.md)

> **[TODO: Add photograph of the team performing track testing]**

Recommended evidence:

- Open Challenge testing,
- obstacle testing,
- sensor calibration,
- steering testing,
- parking testing,
- debugging after a failed run.

Additional development evidence is stored in:

[`engineering-process/`](engineering-process/)

---

# 5. System Architecture

## 5.1 High-Level Robot Architecture

The robot is divided into five interacting engineering subsystems:

1. **Mechanical platform**
2. **Power system**
3. **Sensing**
4. **Computing and control**
5. **Autonomous software**

> **[TODO: Add NEW high-level system architecture block diagram]**

Recommended diagram structure:

```text
                           SENSORS
              ┌──────────────┼───────────────┐
              │              │               │
            LiDAR          Camera           IMU
              │              │               │
              └──────────────┼───────────────┘
                             │
                             v
                      Raspberry Pi 5
                             │
              ┌──────────────┼───────────────┐
              │              │               │
          Localization   Vision System   Path Planning
              │              │               │
              └──────────────┼───────────────┘
                             │
                       Pure Pursuit
                             │
                        USB Serial
                             │
                             v
                    Arduino UNO R4
                       │          │
                       v          v
                  Drive Motor   Steering
                       │
                    Encoder
```

The exact interfaces and software modules are documented in the electrical and software documentation.

---

## 5.2 Control Responsibility

### Raspberry Pi 5 — High-Level Control

The Raspberry Pi is responsible for:

- LiDAR processing,
- camera processing,
- IMU reading,
- localization,
- field mapping,
- obstacle detection,
- racing-line generation,
- path tracking,
- autonomous decisions,
- and generation of steering / motor commands.

### Arduino UNO R4 Minima — Low-Level Control

The Arduino is responsible for:

- drive-motor actuation,
- steering-servo actuation,
- quadrature encoder reading,
- start-button input,
- and execution of commands received from the Raspberry Pi.

The two controllers communicate over USB Serial at **115200 baud**.

---

## 5.3 Sensor Roles

| Sensor | Primary Information | Used By |
|---|---|---|
| RPLiDAR C1 | Environment geometry and distance | Localization, clearance and navigation |
| BNO055 IMU | Relative orientation / heading reference | Heading resolution and navigation |
| Camera | Red / green pillar recognition | Obstacle mapping |
| Motor encoder | Drive-motor rotation | Low-level distance feedback |
| Start switch | Competition start command | Run initialization |

No single sensor is expected to solve every perception problem. The camera, LiDAR, IMU and encoder provide different types of information, and the software assigns each of them a specific role according to what that sensor can measure reliably.

---

# 6. Mobility and Mechanical Engineering

The complete mechanical documentation is available in:

## [`mech/mech_README.md`](mech/mech_README.md)

The mechanical system was developed around the same design philosophy used throughout the project:

> **Predictable controllability is more valuable than maximum speed.**

---

## 6.1 Drivetrain

The final drivetrain combines:

- CHP-20GP-180 motor,
- internal 19:1 gearbox,
- custom 16-tooth motor drive gear,
- 28-tooth LEGO differential gear,
- rear differential,
- bearing-supported axle,
- and LEGO-compatible wheels.

The final drivetrain uses a CHP-20GP-180 motor with a 19:1 internal gearbox to power a rear differential through a custom 16-tooth motor drive gear meshed with a 28-tooth LEGO differential gear. Ball bearings support the rear axle and reduce direct shaft contact with the printed structure, driving the LEGO-compatible wheels efficiently.
The external drivetrain was developed through several iterations:

```text
V1: 28T → 28T
        1:1

V2: 21T → 28T
     Increased reduction

V3: 16T → 28T
     Final reduction
```

Each step increased the external reduction. This reduced maximum drivetrain speed but increased available torque and improved low-speed controllability, which better matched our autonomous driving and parking requirements.

Detailed torque calculations, motor selection, drivetrain iterations and CAD files are documented in the mechanical README.

---

## 6.2 Steering

The front steering system uses an **Ackermann-style geometry** driven by a GEEKSERVO steering servo.

The final steering assembly includes:

- steering axle,
- left and right steering arms,
- linkage,
- servo bracket,
- upper and lower structural mounts.

The purpose of the geometry is to allow the inner and outer front wheels to follow different turning radii during a corner. This better matches automotive-style motion and reduces unnecessary tire scrub compared with forcing both wheels to use the same steering angle.

---

## 6.3 Mechanical Manufacturing

Most of the custom structural components are manufactured by FDM 3D printing. The team uses both **ABS** and **ABS-GF**, selecting the material according to the mechanical function of each part.

ABS provides greater impact toughness and is suitable for many general structural components, while ABS-GF provides greater stiffness and dimensional stability for parts where deformation is more critical. Detailed material data, part selection and manufacturing information are documented in the mechanical README.

Detailed material selection and 3D models files are documented in the mechanical README.

---

# 7. Power and Sensor Architecture

The complete electrical documentation is available in:

## [`elec/elec_README.md`](elec/elec_README.md)

---

## 7.1 Power Architecture

The robot uses one **11.1 V 3S LiPo battery** as its main power source.

The electrical architecture separates the system into dedicated power branches rather than directly supplying every subsystem from one shared rail.

### Computing Branch

The Raspberry Pi is supplied through an LM2596 step-down converter adjusted to approximately **5.1 V**. This branch is dedicated to the high-level computing system and its connected Pi-side electronics.

### Motor / Control Branch

The actuator and control side uses a separate XL4015-based power path together with the motor-control electronics. The separation is intended to reduce the effect that rapidly changing motor and servo current can have on the Raspberry Pi supply.

> **[TODO: In the final electrical rewrite, confirm and document the exact XL4015 output voltage measured on the completed robot.]**

---

## 7.2 Final Sensor Architecture

The original sensing concept changed significantly during development.

### Initial Concept

```text
Camera
+
Ultrasonic Sensors
+
Light Sensor
+
IMU
```

### Final Concept

```text
Camera
+
RPLiDAR C1
+
BNO055 IMU
+
Motor Encoder
```

The ultrasonic and light-sensor approach was removed because the final localization strategy required richer two-dimensional environmental information. LiDAR provided field geometry over many directions and therefore matched the navigation architecture more effectively.

The camera remains important because LiDAR cannot determine the red or green color of a traffic pillar, while the BNO055 provides orientation information and the motor encoder provides drivetrain feedback.

---

## 7.3 Sensor Placement Iterations

Two important physical sensing problems were identified during development.

### Camera Field of View

**Problem:** the original camera lens provided a limited field of view.

**Change:** the lens was replaced with a wider-angle configuration.

**Result:** approximately 60° field of view.

### LiDAR Orientation

**Problem:** an excessive LiDAR mounting angle distorted the 2D representation of the environment.

**Change:** the LiDAR mounting orientation was corrected closer to parallel with the field.

**Result:** improved 2D scan geometry.

The complete sensor-placement reasoning is documented in the electrical README.

---

# 8. Software Architecture and Autonomous Strategy

The complete software documentation is available in:

## [`software/software_README.md`](software/software_README.md)

Our final software architecture is based on a different question from our original camera-reactive concept.

Instead of asking:

> **"What should I steer away from right now?"**

the final system asks:

> **"Where am I on the field, where should I be, and how should I reach that point?"**

This change led to the final localization-based architecture.

---

## 8.1 Autonomous Navigation Pipeline

The main software pipeline is:

```text
LiDAR + IMU
     │
     v
 Localization
(x, y, heading)
     │
     v
 Racing Line
     │
     v
Target Lookahead Point
     │
     v
 Pure Pursuit
     │
     v
Steering + Speed Command
     │
     v
Arduino
     │
     ├── Drive Motor
     └── Steering Servo
```

---

## 8.2 Localization

The robot uses a **particle filter** to estimate its position on the known WRO field.

The localizer uses:

- LiDAR scan geometry,
- estimated robot motion,
- IMU heading information,
- and the known field map.

The estimated pose contains:

```text
X position
Y position
Heading
Confidence
```

This allows the rest of the navigation system to reason in field coordinates rather than only reacting to the immediate camera image.

---

## 8.3 Racing Line and Pure Pursuit

The robot generates a smooth closed path around the WRO field.

Instead of using a steering PID to continuously react to wall offset, the final navigation system uses **Pure Pursuit**.

Pure Pursuit selects a point ahead of the robot on the target path and calculates the steering required to approach that point.

The principal trade-off is the lookahead distance:

| Lookahead | Advantage | Disadvantage |
|---|---|---|
| Short | Accurate local tracking | Can oscillate at higher speed |
| Long | Smoother and more stable | Can cut corners |

The software therefore adjusts lookahead according to the driving condition.

---

## 8.4 Obstacle Strategy

The camera detects the red and green traffic pillars.

The robot does not switch into a completely separate obstacle-avoidance controller.

Instead:

1. the camera identifies the pillar,
2. estimates its bearing and distance,
3. transforms the detection into field coordinates,
4. stores it in a persistent block map,
5. offsets the racing line around the pillar,
6. and allows the same Pure Pursuit controller to follow the modified path.

This keeps normal path tracking and obstacle handling inside one unified navigation system.

---

## 8.5 Sensor Fusion

Different sensors are trusted for different information.

| Sensor | Trusted For | Not Primarily Used For |
|---|---|---|
| LiDAR | Position, scan geometry, clearance | Pillar color |
| IMU | Relative orientation / heading quadrant | Long-term absolute position |
| Camera | Pillar color, bearing and apparent distance | Wall localization |
| Encoder / motor feedback | Drive movement feedback | Absolute field position |

When sensor information conflicts, the software applies explicit rules rather than assuming every sensor is equally reliable.

Detailed arbitration and failure-handling behavior are documented in the software README.

---

## 8.6 Parking

The final-round software includes a dedicated parking maneuver.

Current high-level parking sequence:

1. position the robot relative to the parking structure,
2. reverse while turning into the parking space,
3. reverse in a straighter trajectory,
4. apply the final steering correction to become parallel with the outer wall,
5. stop.

> **[TODO: Add parking state-machine diagram after the final parking implementation is locked.]**

> **[TODO: Add parking success-rate / repeatability result from real track testing.]**

---

# 9. Systems Thinking and Major Engineering Decisions

The robot was developed as one complete system.

The following table summarizes some of the most important decisions.

| Engineering Decision | Alternative / Earlier Design | Reason for Final Choice | Trade-off |
|---|---|---|---|
| 19:1 motor gearbox | Faster 1:5 option | More torque and better low-speed control | Lower maximum speed |
| 16:28 external drivetrain | 28:28 and 21:28 prototypes | Improved low-speed controllability | Additional speed reduction |
| Ackermann-style steering | Simpler steering geometry | Better automotive turning behavior | Increased mechanical complexity |
| LiDAR-based sensing | Ultrasonic + light sensor | Richer environmental information for localization | Higher software complexity |
| Wider-angle camera | Original narrow lens | Larger visual field | Increased image distortion at edges |
| Pi + Arduino architecture | One controller for everything | Separate high-level computing from actuator control | More communication interfaces |
| Separate power branches | One shared low-voltage rail | Reduce motor-related disturbance to Pi supply | Additional converter and wiring |
| Relative IMU reference | Full startup magnetometer calibration | Faster and more practical competition startup | Reduced dependence on absolute magnetic heading |
| Particle-filter localization | Immediate camera-only reaction | Robot can reason about its position on the known field | More computation |
| Pure Pursuit | Reactive steering / competing correction loops | Simple geometric path tracking with clear tuning behavior | Requires a reliable pose |
| Adjustable camera mount | Fixed camera position | Allows angle tuning without redesigning the chassis | More printed parts / joints |
| ABS + ABS-GF | One material for every part | Material selected according to stiffness / toughness requirement | More manufacturing complexity |

Detailed evidence for these decisions is distributed across the mechanical, electrical and software documentation.

---

## 9.1 Subsystem Interaction

### Mechanical → Software

Changing drivetrain reduction changes:

- acceleration,
- maximum speed,
- low-speed controllability,
- steering correction timing,
- and the software speed profile.

### Mechanical → Electrical

Component placement determines:

- cable routing,
- converter position,
- sensor mounting,
- electronics accessibility,
- and weight distribution.

### Electrical → Software

Sensor architecture determines what information the software can use.

Moving from ultrasonic + light sensing to LiDAR changed the software from local reactive sensing toward field localization.

### Software → Mechanical

Software testing revealed real steering behavior that differed from assumed steering geometry.

This required the control model to represent the actual physical steering response rather than only the CAD geometry.

---

# 10. Testing, Validation and Failure Analysis

Testing is used to change the robot design, not only to confirm that the final system works.

---

## 10.1 Software Testing Without the Robot

The software repository includes simulation and test tools that can run important parts of the real navigation software without requiring the complete robot.

Examples include:

```text
test_navigation.py
test_driving.py
test_steering.py
test_color_picker.py
```

These tools allow:

- localization testing,
- random-start trials,
- steering-parameter sweeps,
- steering calibration,
- color-threshold selection,
- and rapid iteration before using limited physical track time.

---

## 10.2 Measured Steering Behavior

Testing identified that the real steering geometry differed from the original assumed value.

The steering calibration process measured approximately:

> **21% more turning than the original assumed full-lock steering value.**

Steering-response testing also measured approximately:

> **0.35 s to reach 63% of the steering response.**

These measurements are important because the real mechanical response affects Pure Pursuit tracking and high-speed stability.

---

## 10.3 Development Problems and Responses

| Problem | Investigation / Observation | Engineering Change | Result |
|---|---|---|---|
| Low drivetrain torque | Robot difficult to control smoothly at low speed | 28:28 → 21:28 → 16:28 external drivetrain | Improved low-speed controllability |
| Narrow camera view | Insufficient visible area | Wider-angle camera lens | Approximately 60° FOV |
| Distorted LiDAR map | LiDAR mounting angle not sufficiently level | Repositioned LiDAR | Improved 2D environment representation |
| Crowded electronics | IMU beside Pi I/O HAT | Relocated IMU under LiDAR | Cleaner physical layout |
| Limited ultrasonic architecture | Needed richer environmental information | Replaced ultrasonic/light-sensor concept with LiDAR | Enabled localization-based navigation |
| Motor-related power changes | Computing and actuator loads interact electrically | Separate power branches | Reduced dependence of Pi supply on actuator branch |
| Steering-model error | Real vehicle turned differently from assumed geometry | Developed steering calibration tools | More accurate control model |
| Sensor disagreement | Sensors provide different types of information | Explicit sensor arbitration | More predictable failure behavior |
| Software exception | Program can terminate unexpectedly | Guaranteed cleanup / motor stop in finalization logic | Robot does not intentionally continue driving after Python failure |

---

## 10.4 Final Physical Test Results

This section should contain only results that were actually measured by the team.

| Test | Trials | Success / Measurement | Evidence |
|---|---:|---|---|
| Open Challenge complete run | **[TODO]** | **[TODO]** | **[TODO: video / log]** |
| Obstacle pillar passing | **[TODO]** | **[TODO]** | **[TODO: video / log]** |
| Parallel parking | **[TODO]** | **[TODO]** | **[TODO: video / log]** |
| Wall-contact test | **[TODO]** | **[TODO]** | **[TODO]** |
| Localization convergence | **[TODO]** | **[TODO]** | Software debug logs |
| Steering calibration | **[TODO]** | ~21% difference from original assumption | Steering test |
| Steering response | **[TODO]** | ~0.35 s to 63% response | Steering-lag test |
| Power stability under driving load | **[TODO]** | **[TODO: measured voltages]** | **[TODO]** |

> **Do not replace TODO values with estimated numbers. Only include measurements that were actually recorded.**

---

## 10.5 Engineering Testing Evidence

> **[TODO: Add testing photo 1 — steering / drivetrain]**

> **[TODO: Add testing photo 2 — LiDAR / localization debug]**

> **[TODO: Add testing photo 3 — obstacle detection]**

> **[TODO: Add testing photo 4 — parking / final track run]**

Additional evidence:

[`engineering-process/testing/`](engineering-process/testing/)


---

# 11. Build and Reproducibility

The complete reproduction procedure is documented in:

# [`BUILD.md`](BUILD.md)

`BUILD.md` is intended to allow others to reproduce the robot using the design files and source code in this repository.

It contains:

1. bill of materials,
2. mechanical manufacturing,
3. mechanical assembly,
4. electrical wiring,
5. Raspberry Pi setup,
6. Arduino setup,
7. compile and upload instructions,
8. first-power verification,
9. sensor verification,
10. actuator verification,
11. first autonomous run,
12. troubleshooting.

---

## 11.1 Reproduction Resources

| Resource | Location |
|---|---|
| Mechanical design reasoning | [`mech/mech_README.md`](mech/mech_README.md) |
| Electrical design reasoning | [`elec/elec_README.md`](elec/elec_README.md) |
| Software design reasoning | [`software/software_README.md`](software/software_README.md) |
| Build procedure | [`BUILD.md`](BUILD.md) |
| Source code | [`src/`](src/) |
| CAD / 3D files | [`mech/models/`](mech/models/) |
| Electrical schematic | [`schemes/Schematic Diagram.png`](schemes/Schematic%20Diagram.png) |
| Wiring diagram | [`schemes/Wiring Diagram.png`](schemes/Wiring%20Diagram.png) |
| Final robot photographs | [`robot-photos/`](robot-photos/) |
| Development evidence | [`engineering-process/`](engineering-process/) |

---

## 11.2 First-Run Verification

Before autonomous testing, the build guide verifies:

- power-converter outputs,
- Raspberry Pi startup,
- Arduino connection,
- motor direction,
- steering center,
- encoder feedback,
- LiDAR communication,
- camera operation,
- IMU response,
- start-button behavior,
- and basic straight / turn movement.

This staged verification reduces the chance that several untested subsystems fail simultaneously during the first autonomous run.

---

# 12. Repository Structure

```text
WRO-FE-YBR-SUNFLOWER/
│
├── README.md
├── BUILD.md
├── LICENSE.md
├── .gitignore
│
├── mech/
│   ├── mech_README.md
│   └── models/
│       ├── STL files
│       └── PNG files
│
├── elec/
│   └── elec_README.md
│
├── software/
│   └── software_README.md
│
├── src/
│   ├── Arduino/
│   │   ├── Main.ino
│   │   └── libraries/
│   │
│   └── Raspberrypi/
│       ├── main.py
│       ├── tasks/
│       ├── classes/
│       ├── utils/
│       ├── liraries
│       ├── test_*.py
│       ├── pyproject.toml
│       └── uv.lock
│
├── schemes/
│   ├── README.md
│   ├── Schematic Diagram.png
│   └── Wiring Diagram.png
│
├── robot-photos/
│   ├── front.jpg
│   ├── rear.jpg
│   ├── left.jpg
│   ├── right.jpg
│   ├── top.jpg
|   ├── bottom.jpg
|   ├── ver-0/
|   ├── ver-1/
│   └── ver-2/
│
├── team-photos/
|   ├── t-pic.jpg
│   └── PNG files
│
├── video/
|   ├── RunTest_video.md
│   └── final_video.md
│
├── other/
|   ├── PNG files
│   └── JPG files
│
└── docs/
    └── [TODO: engineering report / supporting documentation]
```

---

# 13. Version History and Development Milestones

The Git history is part of the engineering documentation because it shows that the robot was developed iteratively rather than uploaded only as a final code dump.

## 13.1 Physical Versions

| Version | Major Change | Status |
|---|---|---|
| V1 | Initial chassis, 1:1 external drivetrain and original sensing concept | Archived prototype |
| V2 | Ackermann steering, bearings, 21T drivetrain gear and LiDAR architecture | Functional prototype |
| V3 | New chassis, 16T drivetrain gear, final electrical layout and competition architecture | Current robot |

---

## 13.2 Repository Milestones

| Milestone | Date | Commit / Release | Description |
|---|---|---|---|
| Initial development | **[TODO]** | **[TODO: commit link]** | Early mechanical / software development |
| Prototype integration | **[TODO]** | **[TODO: commit link]** | Major subsystem integration |
| LiDAR architecture | **[TODO]** | **[TODO: commit link]** | Navigation architecture change |
| Final mechanical rebuild | **[TODO]** | **[TODO: commit link]** | Version 3 robot |
| Competition software | **[TODO]** | **[TODO: commit link]** | Final qualification / obstacle architecture |
| Documentation release | **[TODO]** | **[TODO: release link]** | Competition documentation version |

Detailed release information:

[`CHANGELOG.md`](CHANGELOG.md)

> **[TODO: Create GitHub Release for the submitted competition version and tag the exact commit used for judging.]**

---

# 14. WRO Engineering Documentation Map

This repository separates detailed engineering information by subsystem so that each part of the robot can be evaluated and reproduced clearly.

| WRO Engineering Area | Primary Evidence | Additional Evidence |
|---|---|---|
| **Mobility and Mechanical Design** | [`mech/mech_README.md`](mech/mech_README.md) | CAD, drivetrain iterations, material selection, robot photos |
| **Power and Sensor Architecture** | [`elec/elec_README.md`](elec/elec_README.md) | Schematics, wiring, power budget, sensor placement and calibration |
| **Software Architecture and Obstacle Strategy** | [`software/software_README.md`](software/software_README.md) | Source code, localization, Pure Pursuit, object detection, parking and tests |
| **Systems Thinking and Engineering Decisions** | [Section 9](#9-systems-thinking-and-major-engineering-decisions) | Prototype history, trade-offs, failure analysis and subsystem interactions |
| **Reproducibility and GitHub Quality** | [`BUILD.md`](BUILD.md) | CAD, code, wiring, repository structure, test workflow, Git history and release notes |

---

## 14.1 Evidence Philosophy

We distinguish between:

- **manufacturer specifications,**
- **calculated values,**
- **software configuration values,**
- **observed behavior,**
- and **team-measured test results.**

Quantitative performance values are only reported as measurements when the team actually recorded them.

This prevents unmeasured estimates from being presented as experimental results.

---

# 15. References and Related Documentation

## Internal Documentation

- [`mech/mech_README.md`](mech/mech_README.md) — Mechanical engineering
- [`elec/elec_README.md`](elec/elec_README.md) — Electrical and sensing
- [`software/software_README.md`](software/software_README.md) — Software and autonomy
- [`BUILD.md`](BUILD.md) — Complete reproduction guide
- [`CHANGELOG.md`](CHANGELOG.md) — Version and release history
- [`schemes/`](schemes/) — Electrical diagrams
- [`src/`](src/) — Competition source code
- [`engineering-process/`](engineering-process/) — Development evidence

## External References

> **[TODO: Add official WRO 2026 Future Engineers rules link]**

Component datasheets and manufacturer references are maintained in the subsystem documentation where they are directly relevant to the engineering decisions.

---

# Final Summary

YBR-SUNFLOWER is the result of an iterative engineering process that connects mechanical design, electrical architecture, sensing and autonomous software.

The project evolved from an initial prototype using a simpler drivetrain and sensing concept into a localization-based autonomous vehicle using:

- a torque-focused drivetrain,
- Ackermann-style steering,
- LiDAR localization,
- relative IMU heading,
- camera-based traffic-pillar detection,
- Pure Pursuit path tracking,
- separate high-level and low-level controllers,
- and structured power distribution.

The repository documents not only the final robot, but also the decisions, problems, trade-offs, testing and iterations that produced it.

Our goal is that others should be able to understand **what we built, why we built it this way, how it works, how it was tested, and how to reproduce it** from the documentation contained here.

---

<div align="center">

### YBR-SUNFLOWER

**WRO 2026 Future Engineers**

Yothinburana School · Thailand

**Mechanical · Electrical · Software · Autonomous Systems**

</div>
