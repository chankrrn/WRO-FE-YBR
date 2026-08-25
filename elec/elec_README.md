# Electrical & Sensor System

This document describes the electrical architecture, power system, sensors, wiring, calibration, testing, and engineering decisions of our WRO Future Engineers 2026 robot.

The goal of this documentation is not only to describe what components we use, but also to explain **why we selected them, how they are integrated, how they were tested, and how the design evolved.**

---

# 1. Electrical Hardware Overview

## 1.1 Annotated Hardware Layout

![Annotated Electrical Hardware Layout](./images/electrical_layout.png)

The annotated view shows the main electrical and sensing components
installed on our final robot, including the Raspberry Pi 5, Arduino UNO
R4 Minima, RPLiDAR C1, BNO055 IMU, camera module, battery, motor driver,
and voltage converters.

[เขียนสั้น ๆ อีก 1 ย่อหน้าเรื่องการแบ่งหน้าที่ของ hardware]

## 1.2 Hardware Roles
[ตารางสั้น ๆ]

| Component | Role |
|---|---|
| Raspberry Pi 5 | High-level processing |
| Arduino UNO R4 | Low-level control |
| LiDAR | Distance sensing |
| Camera | Visual sensing |
| BNO055 | Heading / orientation |
| Encoder | Motor feedback |
| Battery | Main power source |
| ... | ... |

---

# 2. Power System

## 2.1 Battery and Power Distribution

![Power Architecture](./images/power_architecture.png)

### Battery

**Battery:** [MODEL]

**Voltage:** [VALUE]

**Capacity:** [VALUE]

[WRITE]

<!-- Explain why this battery was selected. -->

---

## 2.2 Power Distribution

The robot uses a single battery pack, with power distributed into separate regulated branches through **D1-2 (positive)** and **PCT-21 (negative)**.

[WRITE]

<!-- Explain:
- D1-2 = positive distribution
- PCT-21 = negative / ground distribution
- where each branch goes
- why the branches are separated
-->

---

## 2.3 Voltage Conversion

| Component | Input | Output | Supplies |
| --------- | ----: | -----: | -------- |
| LM2596    |   [ ] |    [ ] | [ ]      |
| XL4015    |   [ ] |    [ ] | [ ]      |

[WRITE]

<!-- Explain why these converters were used and why their output voltages were selected. -->

---

## 2.4 Power Budget

| Component    | Voltage | Typical Current | Peak / Stall Current |
| ------------ | ------: | --------------: | -------------------: |
| Raspberry Pi |     [ ] |             [ ] |                  [ ] |
| Camera       |     [ ] |             [ ] |                  [ ] |
| LiDAR        |     [ ] |             [ ] |                  [ ] |
| IMU          |     [ ] |             [ ] |                  [ ] |
| Motor        |     [ ] |             [ ] |                  [ ] |
| Servo        |     [ ] |             [ ] |                  [ ] |

### Power Margin

[WRITE]

<!-- Explain:
- estimated total load
- converter capacity
- battery capability
- any important margin or limitation
-->

---

## 2.5 Power Reliability

[WRITE]

<!-- LEVEL 6:
Describe actual or anticipated problems such as:
- voltage drops
- motor current changes
- electrical noise
- overheating
- unstable power

Then explain how your design reduces these risks.
Only include problems your team actually observed or seriously considered.
-->

---

# 3. Controllers and Communication

## 3.1 Raspberry Pi 5

![Raspberry Pi 5](./images/raspberry_pi.png)

**Purpose:** [WRITE]

**Why we selected it:** [WRITE]

**Main responsibilities:**

* [ ]
* [ ]
* [ ]

---

## 3.2 Arduino UNO R4 Minima

![Arduino UNO R4 Minima](./images/arduino.png)

**Purpose:** [WRITE]

**Why we selected it:** [WRITE]

**Main responsibilities:**

* [ ]
* [ ]
* [ ]

---

## 3.3 Communication

![Controller Communication](./images/controller_communication.png)

[WRITE]

<!-- Explain:
- How Raspberry Pi communicates with Arduino
- Protocol/interface
- What information is exchanged
- Why this method was selected
-->

---

# 4. Sensors and Hardware Placement

> This section combines **sensor selection + placement + purpose** instead of making three separate long sections.

## 4.1 Camera

![Camera Placement](./images/camera_placement.png)

**Purpose:** [WRITE]

**Why we chose it:** [WRITE]

**Placement:** [WRITE]

**Why this position:** [WRITE]

**Problems / limitations:** [WRITE]

**Final solution:** [WRITE]

---

## 4.2 RPLiDAR C1

![LiDAR Placement](./images/lidar_placement.png)

**Purpose:** [WRITE]

**Why we chose it:** [WRITE]

**Placement:** [WRITE]

**Why this position:** [WRITE]

**Problems / limitations:** [WRITE]

**Final solution:** [WRITE]

---

## 4.3 BNO055 IMU

![IMU Placement](./images/imu_placement.png)

**Purpose:** [WRITE]

**Why we chose it:** [WRITE]

**Placement:** [WRITE]

**Why this position:** [WRITE]

**Problems / limitations:** [WRITE]

**Final solution:** [WRITE]

---

## 4.4 Encoder

![Motor and Encoder](./images/motor_encoder.png)

**Purpose:** [WRITE]

**Why encoder feedback is important:** [WRITE]

**How the robot uses the feedback:** [WRITE]

---

## 4.5 Start / Touch Sensor

![Start Button](./images/start_button.png)

**Purpose:** [WRITE]

**Placement:** [WRITE]

**Why this position:** [WRITE]

---

# 5. Sensor Selection and Trade-offs

<!-- KEEP THIS SHORT.
Do NOT repeat all descriptions from Section 4.
This section exists specifically to show engineering comparison. -->

## 5.1 Selection Criteria

[WRITE]

<!-- Examples:
accuracy, range, response time, field of view, power consumption,
weight, integration difficulty, reliability
-->

---

## 5.2 Important Sensor Decisions

### Camera vs. [Alternative]

[WRITE]

> **We chose [X] instead of [Y] because [reason].**

---

### LiDAR vs. [Alternative]

[WRITE]

> **We chose [X] instead of [Y] because [reason].**

---

### IMU / Heading Sensor

[WRITE]

> **We chose [X] instead of [Y] because [reason].**

---

# 6. Wiring Architecture

## 6.1 Schematic Diagram

![Schematic Diagram](../schemes/Schematic%20Diagram.png)

[WRITE A SHORT EXPLANATION]

---

## 6.2 Wiring Diagram

![Wiring Diagram](../schemes/Wiring%20Diagram.png)

[WRITE A SHORT EXPLANATION]

---

## 6.3 Wiring Reference

| Device       | Interface | Controller | Pin / Port | Power |
| ------------ | --------- | ---------- | ---------- | ----- |
| Camera       | [ ]       | [ ]        | [ ]        | [ ]   |
| LiDAR        | [ ]       | [ ]        | [ ]        | [ ]   |
| IMU          | [ ]       | [ ]        | [ ]        | [ ]   |
| Encoder      | [ ]       | [ ]        | [ ]        | [ ]   |
| Motor Driver | [ ]       | [ ]        | [ ]        | [ ]   |
| Servo        | [ ]       | [ ]        | [ ]        | [ ]   |
| Start Button | [ ]       | [ ]        | [ ]        | [ ]   |

---

# 7. Calibration

## 7.1 Calibration Overview

[WRITE]

<!-- Explain why calibration is necessary for your system. -->

---

## 7.2 Camera Calibration

**Problem:** [WRITE]

**Method:** [WRITE]

**Important parameters:** [WRITE]

**Result:** [WRITE]

---

## 7.3 LiDAR Calibration

**Problem:** [WRITE]

**Method:** [WRITE]

**Important parameters:** [WRITE]

**Result:** [WRITE]

---

## 7.4 IMU Calibration

**Problem:** [WRITE]

**Method:** [WRITE]

**Important parameters / offsets:** [WRITE]

**Result:** [WRITE]

---

# 8. Testing and Iteration

> **This is one of the most important sections for Level 6.**

## 8.1 Testing Method

[WRITE]

<!-- Explain:
- what was tested
- where it was tested
- what counted as success
- what metric was observed
-->

---

## 8.2 Electrical Iteration Log

| Version | Problem | Change | Result |
| ------- | ------- | ------ | ------ |
| V1      | [ ]     | [ ]    | [ ]    |
| V2      | [ ]     | [ ]    | [ ]    |
| V3      | [ ]     | [ ]    | [ ]    |
| Final   | [ ]     | [ ]    | [ ]    |

<!-- Add a photo or measurement next to important iterations if available. -->

---

## 8.3 Example of an Important Iteration

### [Problem / Component]

**Problem:** [WRITE]

**Cause / Hypothesis:** [WRITE]

**Change:** [WRITE]

**Test:** [WRITE]

**Result:** [WRITE]

**Decision:** [WRITE]

<!-- This can be very short.
One or two strong real examples are better than ten vague examples.
-->

---

# 9. Failure Modes and Reliability

| Failure Mode | Possible Cause | Effect | Mitigation |
| ------------ | -------------- | ------ | ---------- |
| [ ]          | [ ]            | [ ]    | [ ]        |
| [ ]          | [ ]            | [ ]    | [ ]        |
| [ ]          | [ ]            | [ ]    | [ ]        |

<!-- Examples to consider:
- power instability
- sensor noise
- loose connector
- damaged wire
- sensor obstruction
- overheating
- communication failure

Only include issues relevant to your actual robot.
-->

---

# 10. System-Level Engineering Decisions

<!-- This section exists mainly to support Criterion 4.
Keep it short and focus on the decisions that connect Electrical,
Mechanical and Software. -->

## 10.1 Electrical → Mechanical

[WRITE]

<!-- Example:
How motor power / encoder / sensor placement affected the mechanical design.
-->

---

## 10.2 Electrical → Software

[WRITE]

<!-- Example:
How sensor selection affects the navigation algorithm or control system.
-->

---

## 10.3 Major System Trade-off

> **We chose [X] instead of [Y] because [reason].**

[WRITE]

<!-- Include the constraint:
power / weight / processing / space / reliability / speed / precision
-->

---

# 11. Final Electrical Configuration

![Final Electrical Layout](./images/final_electrical_layout.png)

## Final Components

| Component            | Model | Purpose |
| -------------------- | ----- | ------- |
| Main Controller      | [ ]   | [ ]     |
| Secondary Controller | [ ]   | [ ]     |
| Battery              | [ ]   | [ ]     |
| Motor Driver         | [ ]   | [ ]     |
| Camera               | [ ]   | [ ]     |
| LiDAR                | [ ]   | [ ]     |
| IMU                  | [ ]   | [ ]     |
| Encoder              | [ ]   | [ ]     |
| Servo                | [ ]   | [ ]     |
| Power Converter      | [ ]   | [ ]     |

### Final Design Summary

[WRITE 1–2 PARAGRAPHS]

<!-- Answer:
Why is this the final configuration?
What did the team prioritize?
What important changes happened during development?
-->

---

# 12. Reproducibility

## Required Hardware

[LINK TO COMPONENT LIST / DATASHEETS]

## Wiring Files

* [Schematic Diagram]
* [Wiring Diagram]

## Calibration

[LINK / VALUES]

## Technical Files

[LINK TO RELEVANT FILES]

### Rebuilding the Electrical System

[WRITE A SHORT STEP-BY-STEP DESCRIPTION]

<!-- Another team should understand:
1. what components are required
2. how power is distributed
3. how sensors are connected
4. how controllers communicate
5. what must be calibrated
-->

---

# 13. References

| Component / Topic     | Reference |
| --------------------- | --------- |
| Raspberry Pi 5        | [ ]       |
| Arduino UNO R4 Minima | [ ]       |
| RPLiDAR C1            | [ ]       |
| BNO055                | [ ]       |
| Camera                | [ ]       |
| Motor / Encoder       | [ ]       |
| Power Converters      | [ ]       |
| Motor Driver          | [ ]       |

---

# Level 6 Final Check

Before submitting, verify that this document contains:

* [ ] Power architecture
* [ ] Power budget
* [ ] Sensor selection reasons
* [ ] Sensor placement reasons
* [ ] Calibration
* [ ] Wiring diagram
* [ ] At least one meaningful trade-off
* [ ] Testing and iteration
* [ ] Failure modes / mitigation
* [ ] System-level decisions
* [ ] Final configuration
* [ ] Enough information for another team to understand the electrical system
