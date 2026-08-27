# Mechanical Design & Mobility System

This document describes the complete mechanical development of the **YBR-SUNFLOWER WRO 2026 Future Engineers robot**, including the drivetrain, steering system, chassis, structural components, mechanical calculations, material selection, manufacturing process, prototype iterations, testing, trade-offs, and final engineering decisions.

The purpose of this document is not only to show the final mechanical design, but also to explain:

- why each major mechanical component was selected,
- how torque and speed influenced the drivetrain design,
- why the steering and drivetrain use their current architecture,
- how the robot evolved from its reference blueprint to the final Version 3 platform,
- which mechanical problems were discovered during development,
- how testing changed the design,
- what trade-offs were accepted,
- and how the final mechanical system can be reproduced.

---

# Contents

1. [Mechanical Engineering Overview](#1-mechanical-engineering-overview)
2. [Mechanical Design Requirements and Philosophy](#2-mechanical-design-requirements-and-philosophy)
3. [Mechanical Development Process — V0 to V3](#3-mechanical-development-process--v0-to-v3)
4. [Final Mechanical Architecture](#4-final-mechanical-architecture)
5. [Drivetrain Engineering](#5-drivetrain-engineering)
6. [Steering System](#6-steering-system)
7. [Structural Design and Final 3D Components](#7-structural-design-and-final-3d-components)
8. [Mechanical Sensor and Electronics Integration](#8-mechanical-sensor-and-electronics-integration)
9. [Manufacturing and Material Selection](#9-manufacturing-and-material-selection)
10. [Mechanical Testing and Iteration](#10-mechanical-testing-and-iteration)
11. [Mechanical Trade-offs and Engineering Decisions](#11-mechanical-trade-offs-and-engineering-decisions)
12. [Mechanical Risks and Failure Modes](#12-mechanical-risks-and-failure-modes)
13. [CAD Files and Reproducibility](#13-cad-files-and-reproducibility)
14. [Final Mechanical Configuration](#14-final-mechanical-configuration)

---

# 1. Mechanical Engineering Overview

The YBR-SUNFLOWER robot uses a four-wheel automotive-style mechanical architecture.

The final mobility system consists of:

- one rear drive motor,
- a rear differential,
- a custom external gear reduction,
- two rear driven wheels,
- two front steering wheels,
- servo-driven Ackermann-style steering,
- bearing-supported drivetrain shafts,
- a multi-layer 3D-printed chassis,
- adjustable camera mounting,
- a dedicated LiDAR / IMU structure,
- and modular electronics mounting plates.

Our mechanical system was developed around one central principle:

> **Precision, controllability, stability, and repeatability are more important to our application than maximum theoretical speed.**

This decision affected the motor, gearbox, wheel size, steering geometry, chassis structure, bearing system and material selection.

---

## 1.1 Mechanical System at a Glance

| Category | Final Configuration |
|---|---|
| Vehicle layout | Four-wheel automotive-style vehicle |
| Drive architecture | Rear-wheel drive |
| Drive motor | CHP-20GP-180 DC geared motor |
| Internal gearbox | 19:1 reduction |
| External drive gear | Custom 16-tooth 3D-printed gear |
| Differential gear | LEGO Technic 28-tooth differential |
| External reduction | 28 / 16 = 1.75:1 |
| Steering | Ackermann-style front steering |
| Steering actuator | GEEKSERVO 2 kg 360° servo |
| Wheels | LEGO Tire 43.2 × 22 ZR |
| Rear axle support | Ball bearings + custom axle sleeves |
| Main structural materials | ABS and ABS-GF |
| Manufacturing method | FDM 3D printing |
| Main printer | Bambu Lab H2D |

---

## 1.2 Mechanical Animation

This section provides a quick visual explanation of the final mechanical system.

> **[TODO: Add 360° rotating CAD animation / GIF of the complete Version 3 robot]**
>
> Recommended file:
>
> `models/animations/final_robot_360.gif`

Optional additional animations:

> **[TODO: Add drivetrain animation showing motor → 16T gear → differential → wheels]**

> **[TODO: Add Ackermann steering animation showing left/right wheel motion]**

These animations should help demonstrate how the mechanical subsystems interact before the detailed design sections below.

---

## 1.3 Criterion 1 Evidence Map

| Level 6 Requirement | Evidence in This Document |
|---|---|
| Torque and speed reasoning | Section 5 |
| Drivetrain selection | Sections 5.1–5.7 |
| Steering mechanism | Section 6 |
| Component-selection reasoning | Sections 5, 6 and 9 |
| Structural stability | Section 7 |
| Design trade-offs | Section 11 |
| Testing and iteration | Sections 3 and 10 |
| Mechanical failure analysis | Section 12 |
| Reproducible mechanical design | Section 13 |

---

# 2. Mechanical Design Requirements and Philosophy

## 2.1 Mechanical Requirements

The mechanical system must allow the robot to:

- accelerate smoothly from a stop,
- maintain controllable speed,
- make repeated steering corrections,
- negotiate the WRO track corners,
- pass traffic pillars without excessive wheel slip,
- perform low-speed parking maneuvers,
- support the complete electrical and sensing system,
- maintain stable sensor placement,
- and remain compact enough for the competition field.

---

## 2.2 Main Mechanical Constraints

| Constraint | Effect on Mechanical Design |
|---|---|
| Limited robot footprint | Compact layered chassis |
| Parking requires low-speed precision | Torque prioritized over maximum speed |
| Repeated cornering | Ackermann-style steering and differential |
| Heavy onboard electronics | Rigid multi-layer structure |
| LiDAR requires stable mounting | Dedicated upper sensor support |
| Camera requires tuning | Adjustable camera mechanism |
| 3D-printed drivetrain parts | Material and bearing selection become important |
| Need for rapid redesign | Modular printed components |

---

## 2.3 Design Philosophy

Our initial mechanical development showed that a fast drivetrain is not automatically a better drivetrain.

For WRO Future Engineers, the robot must repeatedly:

- enter corners,
- make steering corrections,
- pass close to obstacles,
- and perform precise low-speed motion.

A drivetrain that produces high speed but cannot be controlled consistently therefore creates problems for the autonomous software.

This led to our main mechanical philosophy:

> **We deliberately accept lower maximum speed when the change increases torque, low-speed controllability, stability, and repeatability.**

This philosophy can be seen directly in the progression of our drivetrain:

```text
28:28
  ↓
21:28
  ↓
16:28
```

Each change increased the external reduction and therefore increased available output torque while reducing maximum wheel speed.

---

# 3. Mechanical Development Process — V0 to V3

The final robot was not produced in one design step.

The mechanical architecture developed through a sequence of reference study, prototyping, testing and redesign.

```text
V0 — Reference Blueprint
        ↓
V1 — First Team Prototype
        ↓
V2 — Functional Prototype
        ↓
V3 — Final Competition Robot
```

---

# 3.1 V0 — Reference Blueprint / Previous-Generation Robot

> **Important:** V0 was not built as part of the current YBR-SUNFLOWER 2026 development cycle.

V0 refers to the **white robot developed by our senior team in the previous season**.

We studied this robot as a mechanical reference before beginning our own design.

Rather than treating the previous robot as a final design to copy, we used it as a **starting blueprint for identifying mechanical requirements and limitations**.

> **[TODO: Add photograph of the white previous-generation robot]**

Recommended caption:

> **Figure — V0 reference robot from the previous season. This vehicle was studied as a mechanical reference before the current team began developing V1.**

---

## 3.1.1 Lessons Taken from V0

One important lesson from the previous-generation design was the relationship between **speed and controllability**.

The previous approach focused more heavily on speed and used conventional DC motors without the encoder configuration selected for our current robot.

The resulting system demonstrated that:

- high speed alone does not guarantee consistent autonomous performance,
- insufficient usable torque makes very low-speed movement difficult,
- accurate parking becomes more difficult without motor feedback,
- and mechanical design must support the requirements of the autonomous control system.

These observations influenced our decision to investigate a geared encoder motor for the new robot.

---

## 3.1.2 How V0 Influenced V1

The purpose of V0 was therefore not to define the final 2026 robot.

Instead, it gave us an initial set of engineering questions:

1. How can we obtain more controllable low-speed movement?
2. How should the drivetrain balance speed and torque?
3. How can we design a steering mechanism specifically for our own chassis?
4. How can mechanical components be made easier to modify during development?
5. How can the chassis support the new sensors and electronics required by our navigation strategy?

These questions became the starting point for V1.

---

# 3.2 V1 — Initial Mechanical Prototype

<table align="center">
  <tr>
    <td align="center">
      <img src="models/Ver1.png" width="500">
    </td>
  </tr>
</table>

V1 was the first mechanical prototype developed for the current robot.

Its main purpose was not to create a complete competition vehicle immediately.

Instead, V1 was used to test the two most fundamental mechanical systems:

- drivetrain,
- and steering.

The prototype used a 3D-printed base and a **28:28 external gear configuration**, creating a **1:1 external ratio**.

```text
Motor Gear: 28T
      ↓
Differential Gear: 28T

External Ratio = 1:1
```

---

## 3.2.1 Goal of V1

The goal was to answer:

> Can the basic motor, drivetrain, chassis and steering concept physically move the vehicle as expected?

At this stage, simplicity was more important than optimization.

---

## 3.2.2 Problem Identified

Testing showed that the 1:1 external drivetrain did not provide the amount of torque and low-speed control that we wanted.

The robot could move, but:

- acceleration from a stop was less controllable,
- low-speed movement was difficult,
- steering corrections were harder to perform consistently,
- and the drivetrain behavior did not match our precision-focused design philosophy.

This became one of the main reasons for changing the drivetrain in V2.

---

# 3.3 V2 — Functional Mechanical Prototype

<table align="center">
  <tr>
    <td align="center">
      <img src="models/Ver2.png" width="500">
    </td>
  </tr>
</table>

V2 expanded V1 from a basic motion prototype into a more complete robot platform.

Major mechanical changes included:

- changing the motor drive gear from 28 teeth to **21 teeth**,
- developing an **Ackermann-style steering mechanism**,
- adding the rear bearing system,
- installing more of the final electronics,
- and creating a more complete mechanical structure.

At this stage, the robot was mechanically usable, although several areas could still be improved.

---

## 3.3.1 Drivetrain Change

The external drivetrain changed from:

```text
V1
28T → 28T
1:1
```

to:

```text
V2
21T → 28T
28 / 21 ≈ 1.33:1 reduction
```

The smaller driver gear increased the external reduction.

This produced:

- greater theoretical output torque,
- lower output speed,
- improved low-speed control,
- and better steering precision.

---

## 3.3.2 Steering Development

V2 introduced the Ackermann-style steering architecture used as the basis of the final system.

Instead of treating both front wheels as if they followed the same turning radius, the steering geometry was designed so that the inner and outer wheels could follow different paths during a turn.

This reduced unnecessary tire scrub and made the vehicle geometry more suitable for automotive-style cornering.

---

## 3.3.3 Bearing Development

V2 also introduced bearings to support the rear drivetrain shaft.

This prevented the axle from rotating directly against 3D-printed structural surfaces.

The bearing system improved:

- alignment,
- rotational smoothness,
- drivetrain rigidity,
- and long-term mechanical reliability.

---

# 3.4 V3 — Final Competition Robot

<table align="center">
  <tr>
    <td align="center">
      <img src="models/Ver Final.png" width="500">
    </td>
  </tr>
</table>

Version 3 is the current final mechanical platform.

Unlike the transition from V1 to V2, V3 involved a more complete redesign and rebuild of the vehicle.

Major changes included:

- reducing the motor drive gear again from 21 teeth to **16 teeth**,
- strengthening the rear structure,
- adding additional structural pillars,
- creating the final LiDAR / IMU mounting system,
- integrating the additional XL4015 tray,
- revising component placement,
- and adding the rear wing / handling structure.

---

## 3.4.1 Final Drivetrain Change

The final external drivetrain is:

```text
16T Driver
     ↓
28T Differential

External Reduction
= 28 / 16
= 1.75:1
```

Compared with V2, this provides additional torque multiplication at the cost of additional speed reduction.

After testing the different mechanical versions, this configuration provided the best balance for our robot between:

- acceleration,
- torque,
- low-speed control,
- steering correction,
- and usable speed.

---

## 3.4.2 Structural Improvements

The rear section was reinforced with additional vertical supports.

The goal was to increase rigidity around the stacked electronics and sensor structures.

The final version also integrates the camera structure, LiDAR support and rear handling wing into the main mechanical platform.

---

# 3.5 Mechanical Evolution Summary

| Version | Driver Gear | External Ratio | Major Mechanical Change | Main Reason |
|---|---:|---:|---|---|
| V0 | Previous-generation configuration | — | Reference vehicle | Learn from previous design |
| V1 | 28T | 1.00:1 | Initial drivetrain / steering prototype | Test basic concept |
| V2 | 21T | 1.33:1 | Ackermann steering + bearings | Increase torque and precision |
| V3 | 16T | 1.75:1 | Final rebuilt platform | Further improve controllability |

---

## 3.6 Iteration Logic

The drivetrain progression demonstrates one of the clearest engineering trade-offs in our robot.

```text
Higher Speed
    ↑
    │
28:28
    │
21:28
    │
16:28
    ↓
Higher Torque / Control
```

We did not select the final ratio only because it produced more torque.

We selected it because the increased reduction produced behavior that better matched the requirements of autonomous navigation and parking.

---

# 4. Final Mechanical Architecture

> **[TODO: Add NEW annotated final mechanical layout image]**

Recommended labels:

- Main Base
- Front Steering Assembly
- Steering Servo
- Camera Mechanism
- Motor
- 16T Drive Gear
- Differential
- Rear Axle
- Bearing Mounts
- Electrical Base
- UNO Base
- Raspberry Pi Base
- LiDAR / IMU Mount
- Rear Wing
- Step-down Tray

---

## 4.1 Mechanical Layout

The final robot uses a layered mechanical architecture.

```text
               Camera
                  │
            Camera Mount
                  │
              Pi Layer
                  │
         Electronics Layer
                  │
       ┌──────────┴──────────┐
       │                     │
Front Steering          Rear Drivetrain
       │                     │
 Ackermann             Motor + Gear
       │                     │
Front Wheels            Differential
                             │
                        Rear Wheels
```

This arrangement keeps the main drivetrain and steering mechanisms close to the lower structural layer while allowing electronics and sensors to be mounted above them.

---

# 5. Drivetrain Engineering

# 5.1 Drivetrain Architecture

The final power path is:

```text
CHP-20GP-180 Motor
        │
        │ Internal Gearbox
        │ 19:1
        v
Motor Output Shaft
        │
        │ 16-Tooth Custom Gear
        v
28-Tooth Differential Gear
        │
        v
LEGO Differential
     ┌──┴──┐
     │     │
     v     v
 Left    Right
 Wheel   Wheel
```

This arrangement combines the motor's internal reduction with an additional external reduction before the wheels.

---

# 5.2 Motor Selection

## CHP-20GP-180 DC 12 V Geared Motor

<img src="../other/DrivingMotor1.png" width="400">
<img src="../other/DrivingMotor2.jpg" width="500">

We selected the **CHP-20GP-180 DC geared motor** with a **19:1 internal gearbox** and dual-phase quadrature encoder.

The main reasons for selecting this motor were:

- increased torque compared with higher-speed variants,
- integrated encoder feedback,
- suitable physical size,
- reliable operation during prototype testing,
- and better suitability for precise autonomous movement.

Our project places greater value on controlled low-speed movement than on the highest possible no-load motor speed.

---

## 5.2.1 Why We Prioritized the 19:1 Configuration

A faster motor configuration can increase maximum vehicle speed.

However, increased maximum speed is less useful if the robot becomes more difficult to:

- accelerate smoothly,
- control through corners,
- position accurately,
- or park.

We therefore selected the higher-reduction configuration because the mechanical characteristics better matched the requirements of the WRO tasks.

---

## 5.2.2 Motor Encoder

The integrated dual-phase encoder provides feedback about motor rotation.

This is especially valuable for movements that require a defined amount of travel instead of relying only on open-loop motor power.

<table align="center">
  <tr>
    <td align="center">
      <img src="../other/DrivingMotor3.jpg" width="600">
    </td>
  </tr>
</table>

### Encoder Wiring

- **Red** — Motor power positive
- **Black** — Hall-effect sensor GND
- **Yellow** — Encoder Signal B
- **Green** — Encoder Signal A
- **Blue** — Hall-effect sensor 5 V
- **White** — Motor power negative

---

## 5.2.3 Electrical Specifications

| Specification | Value |
|---|---:|
| Voltage | 12 V DC |
| No-Load Current | ≤ 280 mA |
| Rated Current | ≤ 550 mA |
| Stall Current | ≤ 2.7 A |

---

## 5.2.4 Mechanical Specifications

| Specification | Value |
|---|---:|
| Internal Gear Ratio | 19:1 |
| No-Load Speed | ~780 RPM |
| Rated Speed | ~680 RPM |
| Rated Torque | 0.40 kg·cm / 0.039 N·m |
| Stall Torque | ≥ 2.0 kg·cm / 0.196 N·m |
| Gearbox Length | 21.5 mm |

> **[TODO: Before final submission, verify that the RPM values above are the motor output values for the exact purchased 19:1 variant. Previous documentation used an additional ~390 RPM figure that corresponds closely to the final drivetrain output after the 16:28 reduction.]**

---

## 5.2.5 Encoder Specifications

| Specification | Value |
|---|---|
| Type | AB Dual-Phase Hall |
| Resolution | ~211 PPR |
| Supply Voltage | 3.3 V / 5.0 V DC |
| Output | Square Wave |

---

# 5.3 Custom Motor Drive Gear

## 16-Tooth Motor Gear

<table align="center">
  <tr>
    <td align="center">
      <img src="models/GearAdapter.PNG" width="400">
      <p><a href="models/GearAdapter.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The final motor drive gear is a custom **16-tooth 3D-printed gear**.

It connects directly to the CHP-20GP-180 motor shaft and meshes with the 28-tooth differential gear.

The center bore is designed to fit the motor shaft tightly to reduce slipping.

The custom design also removes the need for an additional adapter between the motor and the drivetrain.

---

# 5.4 Gearbox Development

## V1 — 28:28

```text
28T Driver
   ↓
28T Driven

Ratio = 1:1
```

The initial objective was to maximize speed while keeping the transmission mechanically simple.

Testing showed that this configuration did not provide enough usable low-speed torque for our application.

---

## V2 — 21:28

```text
21T Driver
   ↓
28T Driven

Reduction = 28 / 21
          ≈ 1.33:1
```

This increased torque and improved low-speed behavior.

The custom 21-tooth gear also connected directly to the motor shaft, making the drivetrain more compact and rigid.

---

## V3 — 16:28

```text
16T Driver
   ↓
28T Driven

Reduction = 28 / 16
          = 1.75:1
```

This further increased torque multiplication.

The additional reduction decreased theoretical maximum speed but improved:

- acceleration,
- low-speed stability,
- control authority,
- and precision.

This became our final configuration.

---

# 5.5 Differential

## LEGO Technic Differential Gear — 28 Teeth

<img src="../other/DifferentialGear1.png" width="300">
<img src="../other/DifferentialGear2.jpg" width="400">

The LEGO Technic differential transfers power to both rear wheels while allowing them to rotate at different speeds during a turn.

This is important because the inside and outside wheels travel different distances when cornering.

Without a differential, a rigidly connected rear axle would force both wheels to rotate at the same speed, increasing tire scrub and resistance.

The differential therefore provides:

- smoother cornering,
- reduced wheel scrub,
- improved maneuverability,
- and better compatibility with the automotive-style steering system.

---

# 5.6 Final Drivetrain Calculation

<img src="models/Powertrains.png" width="800">

The final drivetrain uses:

```text
Motor Output Gear = 16 teeth
Differential Gear = 28 teeth
```

---

## Step 1 — External Gear Reduction

```text
External Reduction
= Driven Gear / Driver Gear

= 28 / 16

= 1.75 : 1
```

---

## Step 2 — Rated Output RPM

Using the documented motor rated speed of approximately **680 RPM**:

```text
Output RPM
= Motor Rated RPM / External Reduction

= 680 / 1.75

≈ 389 RPM
```

Using the documented no-load speed of approximately **780 RPM**:

```text
780 / 1.75 ≈ 446 RPM
```

---

## Step 3 — Output Torque

Using the documented rated motor torque:

```text
Motor Rated Torque = 0.0392 N·m
```

The ideal theoretical output after the external reduction is:

```text
Output Torque
= Motor Torque × External Reduction

= 0.0392 × 1.75

≈ 0.069 N·m
```

Ignoring drivetrain losses, the differential then distributes torque between the two rear outputs.

Approximate theoretical torque per wheel:

```text
0.069 / 2

≈ 0.035 N·m
```

---

## 5.6.1 Calculated Final Results

| Parameter | Calculated Value |
|---|---:|
| External gear reduction | 1.75:1 |
| Differential input speed — rated | ~389 RPM |
| Differential input speed — no load | ~446 RPM |
| Ideal differential input torque | ~0.069 N·m |
| Ideal torque per rear output | ~0.035 N·m |

> These calculations are ideal theoretical values and do not include mechanical losses from gear friction, bearings, differential friction, tire deformation or drivetrain alignment.

This distinction is important because calculated drivetrain performance and measured vehicle performance are not identical.

---

# 5.7 Wheel Selection

## LEGO Tire 43.2 × 22 ZR  
## Wheel 30.4 mm D × 20 mm Reinforced Rim

<img src="../other/wheel1.png" width="200">
<img src="../other/wheel2.png" width="200">

The wheel was selected because its size provides a practical balance between:

- speed,
- acceleration,
- torque demand,
- ground clearance,
- and controllability.

A smaller wheel travels less distance per revolution and can reduce vehicle speed.

A larger wheel increases distance traveled per revolution but also increases the torque required at the axle for the same force at the ground.

Our selected wheel size works well with the motor and drivetrain reduction while maintaining predictable handling.

---

# 6. Steering System

## 6.1 Steering Architecture

The final robot uses a servo-driven **Ackermann-style front steering system**.

<table align="center">
  <tr>
    <td align="center">
      <img src="models/SteeringSystem.PNG" width="500">
      <p><a href="models/SteeringSystem.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

---

## 6.2 Why Ackermann-Style Steering?

During a turn, the inner front wheel follows a smaller-radius path than the outer wheel.

The steering geometry is therefore designed so that the two wheels do not need identical steering angles.

Conceptually:

```text
                Turn Centre
                     ●
                    /|
                   / |
          Inner   /  |  Outer
          Wheel  /   |  Wheel
```

The inward steering-arm geometry approximates Ackermann behavior.

The design aims to:

- reduce tire scrub,
- reduce resistance during turns,
- improve cornering stability,
- and create more predictable automotive-style motion.

---

## 6.3 Steering Components

<table align="center">
  <tr>
    <td align="center">
      <img src="models/SteeringAxle.PNG" width="300">
      <p><a href="models/SteeringAxle.stl">Steering Axle</a></p>
    </td>
    <td align="center">
      <img src="models/Top_SteeringMount.PNG" width="300">
      <p><a href="models/Top_SteeringMount.stl">Top Steering Mount</a></p>
    </td>
    <td align="center">
      <img src="models/Bottom_SteeringMount.PNG" width="300">
      <p><a href="models/Bottom_SteeringMount.stl">Bottom Steering Mount</a></p>
    </td>
  </tr>
</table>

<table align="center">
  <tr>
    <td align="center">
      <img src="models/L_SteeringArm.PNG" width="280">
      <p><a href="models/L_SteeringArm.stl">Left Steering Arm</a></p>
    </td>
    <td align="center">
      <img src="models/R_SteeringArm.PNG" width="280">
      <p><a href="models/R_SteeringArm.stl">Right Steering Arm</a></p>
    </td>
    <td align="center">
      <img src="models/Top_SteeringCap.PNG" width="280">
      <p><a href="models/Top_SteeringCap.stl">Top Steering Cap</a></p>
    </td>
    <td align="center">
      <img src="models/SteeringLinkageArm.PNG" width="280">
      <p><a href="models/SteeringLinkageArm.stl">Steering Linkage Arm</a></p>
    </td>
  </tr>
</table>

### Steering Axle

The steering axle acts as the primary pivot for the steering arms.

It maintains wheel alignment while allowing the steering arms to rotate with minimal unwanted lateral movement.

### Steering Linkage Arm

The linkage connects the servo output to the steering arms.

It converts servo rotation into a push-pull motion that changes the angle of both front wheels.

### Top Steering Mount / Cap

The upper structure stabilizes the steering pivots and works together with the lower mount to keep the steering assembly rigid.

### Bottom Steering Mount

The lower mount supports and aligns the steering pivots.

The original design also included a mounting location for the light sensor used in the earlier sensing architecture.

Although the light sensor was later removed from the final sensing system, the mechanical history of this feature is retained because it shows how the steering structure evolved together with the sensor architecture.

### Left and Right Steering Arms

The steering arms connect directly to the front wheels.

Their inward geometry creates the Ackermann-style relationship between the inside and outside wheels during cornering.

---

## 6.4 Steering Geometry Measurements

> **[TODO: Add final measured wheelbase]**

> **[TODO: Add final measured front track width]**

> **[TODO: Add maximum inner wheel steering angle]**

> **[TODO: Add maximum outer wheel steering angle]**

> **[TODO: Add measured minimum turning radius if available]**

These measurements should be taken from the completed Version 3 robot rather than only from the CAD model.

---

# 6.5 Steering Servo

## GEEKSERVO 2 kg 360° Servo

<img src="../other/servo.png" width="400">

The GEEKSERVO was selected because it provides sufficient speed and torque for the steering system while also being mechanically compatible with LEGO-style mounting.

This simplifies its integration with the hybrid custom / LEGO mechanical structure.

The servo has also been used reliably during earlier development without significant mechanical failures.

The gear mechanism can slip under excessive blocking load instead of remaining completely locked, which can reduce the chance of damage under severe mechanical overload.

### Servo Wiring

- **Red** — Positive
- **Brown** — Ground
- **Yellow** — Signal

### Electrical Specifications

| Specification | Value |
|---|---:|
| Working Voltage | 3.3–6 V |
| Rated Voltage | 4.8 V |
| Rated Current | 200 mA |
| Stall Current | 700 mA |
| Sliding Current | 450 mA |

---

## 6.6 Servo Bracket

<table align="center">
  <tr>
    <td align="center">
      <img src="models/ServoBracket.PNG" width="400">
      <p><a href="models/ServoBracket.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The servo bracket securely holds the LEGO-compatible steering servo.

Built-in cylindrical mounting features allow the servo to integrate directly with the surrounding structure while maintaining the required height and alignment.

The bracket was designed to be:

- compact,
- lightweight,
- easy to print,
- and rigid enough to keep the steering actuator aligned.

---

# 7. Structural Design and Final 3D Components

The final chassis is divided into modular structural layers.

This makes it easier to:

- manufacture individual parts,
- replace damaged parts,
- modify one subsystem without redesigning the complete robot,
- and organize the electronics vertically.

---

# 7.1 Main Base

<table align="center">
  <tr>
    <td align="center">
      <img src="models/MainBase.PNG" width="400">
      <p><a href="models/MainBase.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The Main Base is the robot's primary structural layer.

It supports:

- the battery,
- Arduino UNO R4,
- steering servo,
- and drivetrain components.

Placing heavier mechanical and power components low in the robot helps keep the center of mass closer to the ground.

The mounting points also provide a rigid interface for the upper structural layers.

---

# 7.2 Supporting Base 1 — Electrical Base

<table align="center">
  <tr>
    <td align="center">
      <img src="models/ElecPlate.PNG" width="400">
      <p><a href="models/ElecPlate.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

This plate supports several electrical-system components, including:

- LM2596,
- PCT-21 connector,
- D1-2 connector,
- main power switch,
- competition start switch.

It also acts as part of the support structure for the Raspberry Pi layer.

---

# 7.3 Supporting Base 2 — Arduino Base

<table align="center">
  <tr>
    <td align="center">
      <img src="models/UnoPlate.PNG" width="400">
      <p><a href="models/UnoPlate.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The Arduino base supports the Arduino UNO R4 Minima and its motor-control hardware.

The XL4015 assembly is also integrated around this section of the robot.

---

# 7.4 Supporting Base 3 — Raspberry Pi Base

<table align="center">
  <tr>
    <td align="center">
      <img src="models/PiPlate.PNG" width="400">
      <p><a href="models/PiPlate.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

This layer supports:

- Raspberry Pi 5,
- Raspberry Pi I/O Expansion HAT,
- camera structure,
- and rear structural supports.

The stacked layout allows the high-level computing system to occupy otherwise unused vertical space without interfering directly with the drivetrain.

---

# 7.5 Motor Bracket

<table align="center">
  <tr>
    <td align="center">
      <img src="models/MotorBracket.PNG" width="400">
      <p><a href="models/MotorBracket.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The motor bracket is a custom 3D-printed structural component designed to secure the drive motor rigidly.

The mount includes threaded mounting features so that screws can be fastened without separate nuts.

This provides:

- compact assembly,
- easier access,
- and rigid motor alignment.

Maintaining motor alignment is particularly important because misalignment between the 16T motor gear and 28T differential gear can increase friction or cause poor gear meshing.

---

# 7.6 Rear Bearing Mount System

<table align="center">
  <tr>
    <td align="center">
      <img src="models/BearingSystem.PNG" width="400">
      <p><a href="models/BearingSystem.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

<table align="center">
  <tr>
    <td align="center">
      <img src="models/BearingMount.PNG" width="300">
      <p><a href="models/BearingMount.stl">Bearing Mount</a></p>
    </td>
    <td align="center">
      <img src="models/L%26R_AxleSleeve.PNG" width="300">
      <p><a href="models/L%26R_AxleSleeve.stl">Left / Right Axle Sleeve</a></p>
    </td>
    <td align="center">
      <img src="models/Mid_AxleSleeve.PNG" width="300">
      <p><a href="models/Mid_AxleSleeve.stl">Middle Axle Sleeve</a></p>
    </td>
  </tr>
</table>

The rear-wheel bearing system uses ball bearings to support the differential output shaft.

The inner bearing race interfaces with the shaft through custom sleeves while the outer race remains fixed in the mount.

This prevents the axle from rotating directly against printed plastic.

The bearing system therefore improves:

- rotational efficiency,
- axle alignment,
- structural rigidity,
- durability,
- and drivetrain consistency.

---

# 7.7 Rear Wing / Handling Structure

<table align="center">
  <tr>
    <td align="center">
      <img src="models/RearWing.PNG" width="400">
      <p><a href="models/RearWing.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The rear wing is based on an **S1223 airfoil profile**, but aerodynamic performance is not its primary purpose on this robot.

At the low operating speed of the vehicle, and with the camera support disturbing airflow, meaningful aerodynamic downforce is expected to be very small.

Its main practical function is therefore as:

- a structural rear element,
- additional support for the camera structure,
- and a convenient handling point for carrying the robot.

This distinction prevents us from claiming an aerodynamic benefit that is not significant at the robot's operating speed.

---

# 7.8 Step-Down Tray

<table align="center">
  <tr>
    <td align="center">
      <img src="models/StepdownTray.PNG" width="400">
      <p><a href="models/StepdownTray.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The step-down tray holds the XL4015 converter above the Arduino / motor-shield section.

The tray allows the converter to be mechanically secured instead of relying only on wiring or loose mounting.

---

# 8. Mechanical Sensor and Electronics Integration

Mechanical design and sensor performance are directly connected.

The mechanical structure must hold each sensor:

- securely,
- at the correct orientation,
- with a clear field of view,
- and without excessive vibration or movement.

---

# 8.1 Camera Positioning Mechanism

<table align="center">
  <tr>
    <td align="center">
      <img src="models/CamMount.PNG" width="400">
      <p><a href="models/CamMount.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The camera positioning mechanism allows the camera height and angle to be adjusted.

This was selected instead of a completely fixed mount because competition and testing environments can differ.

A fixed mount would require:

```text
Change Camera Position
        ↓
Modify CAD
        ↓
Reprint Part
        ↓
Reassemble Robot
```

The adjustable mechanism instead allows:

```text
Change Camera Position
        ↓
Adjust Existing Mechanism
        ↓
Continue Testing
```

This reduces mechanical iteration time.

---

## 8.1.1 Camera Components

<table align="center">
  <tr>
    <td align="center">
      <img src="models/CamPlate.PNG" width="300">
      <p><a href="models/CamPlate.stl">Camera Plate</a></p>
    </td>
    <td align="center">
      <img src="models/CamArm.PNG" width="300">
      <p><a href="models/CamArm.stl">Camera Arm</a></p>
    </td>
    <td align="center">
      <img src="models/CamArmConnector.PNG" width="300">
      <p><a href="models/CamArmConnector.stl">Camera Arm Connector</a></p>
    </td>
  </tr>
</table>

### Camera Plate

The camera plate holds the camera rigidly and reduces unwanted movement during autonomous operation.

### Camera Arm

The camera arm positions the camera at the required height and angle while maintaining a clear view of the field.

### Camera Arm Connector

The connector links the camera support to the Raspberry Pi / rear structure.

This distributes the mechanical load and increases rigidity during acceleration, braking and steering.

---

# 8.2 LiDAR and IMU Mount

<table align="center">
  <tr>
    <td align="center">
      <img src="models/LiDARMount.PNG" width="400">
      <p><a href="models/LiDARMount.stl">View 3D model</a></p>
    </td>
  </tr>
</table>

The LiDAR mount is designed to hold the RPLiDAR securely while maintaining a clear scanning area.

The recessed center allows the IMU to be positioned underneath the LiDAR.

The final arrangement:

- uses vertical space efficiently,
- reduces crowding around the Raspberry Pi,
- keeps the LiDAR mechanically stable,
- and moves the IMU away from some of the denser electronics and wiring around the Pi area.

The IMU placement was also chosen to reduce exposure to nearby sources of magnetic interference that could influence magnetometer-based heading information.

---

# 9. Manufacturing and Material Selection

Most custom mechanical parts were manufactured using FDM 3D printing.

This allowed us to rapidly modify parts between prototypes rather than depending on fixed commercial chassis components.

---

# 9.1 3D Printer

## Bambu Lab H2D

<img src="../other/X2D.png" width="400">

The **Bambu Lab H2D** was used to manufacture the custom robot parts.

It was suitable for our development process because it provides:

- high printing speed,
- dimensional accuracy,
- large build volume,
- a heated chamber,
- high-temperature printing capability,
- and compatibility with engineering filaments.

Rapid printing was especially useful during prototype development because mechanical changes could be manufactured and tested quickly.

---

## 9.1.1 General Specifications

| Specification | Value |
|---|---|
| Build Volume | 325 × 320 × 325 mm³ — Single Nozzle |
| Dual-Nozzle Build Volume | 300 × 320 × 325 mm³ |
| Hotend | All-Metal |
| Included Nozzle | 0.4 mm Hardened Steel |
| Maximum Nozzle Temperature | 350°C |
| Filament Diameter | 1.75 mm |
| Maximum Toolhead Speed | 1000 mm/s |
| Maximum Acceleration | 20,000 mm/s² |
| Maximum Chamber Temperature | 65°C |
| Maximum Heatbed Temperature | 120°C |
| Supported Nozzle Sizes | 0.2 / 0.4 / 0.6 / 0.8 mm |

---

## 9.1.2 Physical Dimensions

| Specification | Value |
|---|---|
| Printer Dimensions | 492 × 514 × 626 mm³ |
| Net Weight | 31 kg |

---

## 9.1.3 Build Plate

| Specification | Value |
|---|---|
| Build Plate | Flexible Steel Plate |
| Included Plate | Textured PEI |
| Supported Plates | Textured PEI / Smooth PEI |
| Maximum Heatbed Temperature | 120°C |
| Heated Chamber | Up to 65°C |

---

# 9.2 Filament Selection

## Bambu Lab ABS and ABS-GF

<img src="../other/ABS-GF.jpg" width="350">
<img src="../other/ABS Olive.jpg" width="350">

We use two main materials:

- **ABS**
- **ABS-GF — glass-fiber-reinforced ABS**

The materials are not used interchangeably.

They are selected according to the mechanical requirements of each part.

---

## ABS

ABS provides a useful balance of:

- impact toughness,
- strength,
- smoother printed surface,
- and some additional flexibility compared with ABS-GF.

It is useful for general structural components and parts where extreme stiffness is not the primary requirement.

---

## ABS-GF

ABS-GF contains glass-fiber reinforcement.

This increases:

- stiffness,
- dimensional stability,
- and heat resistance.

It is used for components that require greater rigidity and resistance to deformation.

---

## 9.2.1 Material Trade-off

| Property | ABS | ABS-GF |
|---|---|---|
| Impact toughness | Higher | Lower |
| Stiffness | Lower | Higher |
| Dimensional stability | Good | Higher |
| Surface texture | Smoother | Rougher |
| Best use | General / impact-tolerant parts | Rigid structural parts |

---

## 9.2.2 Material Specifications

| Specification | Bambu Lab ABS | Bambu Lab ABS-GF |
|---|---:|---:|
| Material | ABS | Glass-Fiber Reinforced ABS |
| XY Impact Strength | 39.3 kJ/m² | 14.5 kJ/m² |
| XY Bending Strength | 62 MPa | 68 MPa |
| XY Bending Modulus | 1880 MPa | 2860 MPa |
| Z Impact Strength | 7.4 kJ/m² | 5.3 kJ/m² |
| HDT @ 0.45 MPa | 87°C | 99°C |
| Filament Diameter | 1.75 mm | 1.75 mm |

The material data shows the design trade-off clearly:

> **ABS provides greater impact toughness, while ABS-GF provides greater stiffness and heat resistance.**

We therefore select the material according to the function of each component rather than printing the complete robot from one material.

> **[TODO: Add manufacturer datasheet links for the material-property values above.]**

---

# 10. Mechanical Testing and Iteration

Mechanical testing is used to modify the design rather than only verify that the final vehicle moves.

---

## 10.1 Drivetrain Testing

The clearest mechanical iteration was the drivetrain.

| Version | Configuration | Observation | Decision |
|---|---|---|---|
| V1 | 28:28 | Insufficient low-speed torque / control | Increase reduction |
| V2 | 21:28 | Improved torque and controllability | Test further reduction |
| V3 | 16:28 | Best balance found for current robot | Selected as final |

---

## 10.2 Testing Logic

```text
V1 — 28:28
      │
      │ Low-speed control insufficient
      v
V2 — 21:28
      │
      │ Improved, but further torque desired
      v
V3 — 16:28
      │
      v
Final Configuration
```

This is a direct example of testing affecting the final mechanical design.

---

## 10.3 Steering Testing

The steering mechanism was tested together with the software rather than evaluated only from CAD geometry.

Software steering-calibration tools later showed that the real robot's steering response differed from the original assumed geometry.

The measured vehicle turned approximately **21% more than the original assumed full-lock value**.

This demonstrates why the physical mechanical system must be measured after assembly.

> **[TODO: Add link to steering test evidence / log.]**

---

## 10.4 Additional Mechanical Measurements

To strengthen the final competition documentation, record the following using the completed V3 robot:

| Measurement | Final Value |
|---|---|
| Robot length | **[TODO]** |
| Robot width | **[TODO]** |
| Robot height | **[TODO]** |
| Robot mass | **[TODO]** |
| Wheelbase | **[TODO]** |
| Front track width | **[TODO]** |
| Rear track width | **[TODO]** |
| Minimum turning radius | **[TODO]** |
| Maximum steering angle | **[TODO]** |

---

## 10.5 Optional Mechanical Repeatability Test

> **[TODO: If enough testing data exists, add a short repeatability table.]**

Example structure:

| Test | Trials | Result |
|---|---:|---|
| Smooth start from rest | [TODO] | [TODO] |
| Full steering left / right | [TODO] | [TODO] |
| Corner completion | [TODO] | [TODO] |
| Parking approach | [TODO] | [TODO] |
| Mechanical failure | [TODO] | [TODO] |

Only actual recorded test results should be entered.

---

# 11. Mechanical Trade-offs and Engineering Decisions

The final mechanical design contains several intentional trade-offs.

| Decision | Benefit | Cost / Trade-off | Final Reason |
|---|---|---|---|
| 19:1 motor gearbox | Higher torque | Lower motor speed | Better low-speed behavior |
| 16:28 external reduction | Additional torque multiplication | Reduced wheel speed | Precision prioritized |
| Differential | Reduced wheel scrub during turns | More mechanical parts | Better cornering behavior |
| Ackermann-style steering | More appropriate inner/outer wheel geometry | Increased design complexity | Better automotive motion |
| Bearings | Lower shaft friction and better alignment | Additional components | Improved reliability |
| Adjustable camera mount | Rapid physical tuning | More parts / joints | Faster iteration |
| Layered chassis | Efficient use of volume | Taller structure | Better electronics organization |
| ABS-GF | Higher stiffness | Lower impact toughness | Used where rigidity is critical |
| ABS | Higher toughness | Lower stiffness | Used where impact tolerance is useful |
| Rear wing as handle | Strong handling point | Additional mass | Practical handling / support |
| Custom printed drivetrain parts | Rapid modification | Potential wear / tolerances | Enables fast prototype iteration |

---

## 11.1 Most Important Mechanical Decision

The most important mechanical trade-off was:

> **Maximum speed vs. torque and controllability.**

Our prototype development showed that maximizing drivetrain speed did not produce the most useful autonomous vehicle.

The final design therefore uses both:

- a 19:1 internal motor reduction,
- and a 1.75:1 external reduction.

The final drivetrain sacrifices maximum theoretical speed in exchange for greater torque multiplication and more controllable low-speed behavior.

---

# 12. Mechanical Risks and Failure Modes

| Mechanical Risk / Failure Mode | Possible Effect | Design Response |
|---|---|---|
| Insufficient drivetrain torque | Poor acceleration / inconsistent low-speed movement | Increase external reduction |
| Excessive drivetrain speed | Harder steering and parking control | Select higher reduction |
| Gear misalignment | Friction, wear, poor power transfer | Rigid motor bracket |
| Rear axle bending / friction | Inconsistent drivetrain behavior | Bearing-supported axle |
| Steering play | Inconsistent wheel angle | Rigid steering mounts and pivots |
| Printed-part deformation | Misalignment | Use ABS-GF where additional stiffness is needed |
| Camera vibration | Unstable image | Rigid camera plate and support |
| LiDAR mounting error | Incorrect scan geometry | Dedicated rigid LiDAR mount |
| Electronics structure flex | Sensor / wiring movement | Reinforced rear structure |
| Mechanical obstruction | Limited sensor view | Elevated / dedicated sensor mounts |

---

# 13. CAD Files and Reproducibility

The complete mechanical system is designed so that another team can reproduce the physical structure using the provided CAD / STL files.

Mechanical assembly instructions are provided separately in:

[`../BUILD.md`](../BUILD.md)

---

## 13.1 Final Mechanical Files

| Component | Model / STL |
|---|---|
| Main Base | [`MainBase.stl`](models/MainBase.stl) |
| Electrical Base | [`ElecPlate.stl`](models/ElecPlate.stl) |
| Arduino Base | [`UnoPlate.stl`](models/UnoPlate.stl) |
| Raspberry Pi Base | [`PiPlate.stl`](models/PiPlate.stl) |
| 16T Driver Gear | [`GearAdapter.stl`](models/GearAdapter.stl) |
| Motor Bracket | [`MotorBracket.stl`](models/MotorBracket.stl) |
| Bearing System | [`BearingSystem.stl`](models/BearingSystem.stl) |
| Bearing Mount | [`BearingMount.stl`](models/BearingMount.stl) |
| Left / Right Axle Sleeve | [`L&R_AxleSleeve.stl`](models/L%26R_AxleSleeve.stl) |
| Middle Axle Sleeve | [`Mid_AxleSleeve.stl`](models/Mid_AxleSleeve.stl) |
| Steering System | [`SteeringSystem.stl`](models/SteeringSystem.stl) |
| Steering Axle | [`SteeringAxle.stl`](models/SteeringAxle.stl) |
| Top Steering Mount | [`Top_SteeringMount.stl`](models/Top_SteeringMount.stl) |
| Bottom Steering Mount | [`Bottom_SteeringMount.stl`](models/Bottom_SteeringMount.stl) |
| Left Steering Arm | [`L_SteeringArm.stl`](models/L_SteeringArm.stl) |
| Right Steering Arm | [`R_SteeringArm.stl`](models/R_SteeringArm.stl) |
| Top Steering Cap | [`Top_SteeringCap.stl`](models/Top_SteeringCap.stl) |
| Steering Linkage | [`SteeringLinkageArm.stl`](models/SteeringLinkageArm.stl) |
| Servo Bracket | [`ServoBracket.stl`](models/ServoBracket.stl) |
| Camera Mount | [`CamMount.stl`](models/CamMount.stl) |
| Camera Plate | [`CamPlate.stl`](models/CamPlate.stl) |
| Camera Arm | [`CamArm.stl`](models/CamArm.stl) |
| Camera Connector | [`CamArmConnector.stl`](models/CamArmConnector.stl) |
| LiDAR / IMU Mount | [`LiDARMount.stl`](models/LiDARMount.stl) |
| Rear Wing | [`RearWing.stl`](models/RearWing.stl) |
| Step-Down Tray | [`StepdownTray.stl`](models/StepdownTray.stl) |

---

## 13.2 Manufacturing Files

> **[TODO: Add `.3mf` / Bambu Studio project files if they are still available.]**

Recommended structure:

```text
mech/
├── mech_README.md
│
├── models/
│   ├── CAD/
│   ├── STL/
│   ├── renders/
│   └── animations/
│
└── slicer/
    ├── ABS/
    └── ABS-GF/
```

Including slicer files would improve reproducibility because another team could reproduce not only the geometry but also the intended printing orientation and manufacturing parameters.

---

# 14. Final Mechanical Configuration

The final Version 3 mechanical system consists of:

## Drivetrain

- CHP-20GP-180 DC geared motor
- 19:1 internal gearbox
- dual-phase encoder
- custom 16-tooth drive gear
- LEGO Technic 28-tooth differential
- 1.75:1 external reduction
- bearing-supported rear axle
- LEGO rear wheels

## Steering

- GEEKSERVO steering servo
- custom servo bracket
- Ackermann-style steering geometry
- custom left / right steering arms
- steering linkage
- upper and lower steering mounts

## Structure

- Main Base
- Electrical Base
- Arduino Base
- Raspberry Pi Base
- reinforced rear structure
- rear handling wing

## Sensor Mounting

- adjustable camera mechanism
- dedicated LiDAR mount
- IMU mounting below LiDAR

## Manufacturing

- Bambu Lab H2D
- ABS
- ABS-GF
- modular 3D-printed components

---

# Final Mechanical Summary

The mechanical development of YBR-SUNFLOWER progressed from a previous-generation reference vehicle through three new mechanical stages.

The most important development path was:

```text
Previous-Generation Reference
            ↓
Basic Mechanical Prototype
            ↓
Torque + Steering Improvement
            ↓
Complete Mechanical Rebuild
            ↓
Final Competition Platform
```

The final design reflects several deliberate engineering decisions:

- torque was prioritized over maximum speed,
- the drivetrain reduction was increased through testing,
- a differential was used to support automotive-style cornering,
- Ackermann-style steering was adopted,
- bearings were added to improve drivetrain alignment and efficiency,
- material selection was based on stiffness and toughness requirements,
- and modular sensor / electronics mounts were designed to support continued iteration.

The final mechanical design is therefore not only the result of component selection.

It is the result of repeated:

> **Design → Build → Test → Identify → Modify → Validate**

cycles throughout the development of the robot.
