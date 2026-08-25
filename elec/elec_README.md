# Electrical & Sensor System

This document describes the electrical architecture, power system, sensors, wiring, calibration, testing, and engineering decisions of our WRO Future Engineers 2026 robot.

The goal of this documentation is not only to describe what components we use, but also to explain **why we selected them, how they are integrated, how problems were identified, and how the design evolved during development.**

---

# 1. Electrical Hardware Overview

## 1.1 Annotated Hardware Layout

<img width="700" height="600" alt="Base Plate View" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/1cafb8eafe769b20afa6bdf1097000732c3158ca/other/ComponentsImage1.png" />

<img width="700" height="600" alt="Right View" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/1cafb8eafe769b20afa6bdf1097000732c3158ca/other/ComponentsImage2.png" />

<img width="700" height="600" alt="Left View" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/1cafb8eafe769b20afa6bdf1097000732c3158ca/other/ComponentsImage3.png" />

<img width="700" height="600" alt="Front View1" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/1cafb8eafe769b20afa6bdf1097000732c3158ca/other/ComponentsImage4.png" />

<img width="700" height="600" alt="Front View2" src="https://github.com/chankrrn/WRO-FE-YBR-SUNFLOWER/blob/00793b0ff7648bf21d1988b03f3d5daa953632f8/other/ComponentsImage5.png" />

The annotated views show the main electrical and sensing components installed on our final robot, including the Raspberry Pi 5, Arduino UNO R4 Minima, RPLiDAR C1, BNO055 IMU, camera module, battery, motor driver, power converters, connectors, and start switch.

The electrical system is divided into high-level computing, low-level control, sensing, and power-distribution components. The Raspberry Pi 5 processes information from the camera, LiDAR, and IMU for navigation, while the Arduino UNO R4 Minima handles the drive motor, steering servo, encoder feedback, and start button.

## 1.2 Hardware Roles

| Component                        | Role                                                         |
| -------------------------------- | ------------------------------------------------------------ |
| Raspberry Pi 5                   | High-level processing and navigation                         |
| Arduino UNO R4 Minima            | Low-level motor, steering, encoder, and start-button control |
| RPLiDAR C1                       | 2D distance sensing                                          |
| Raspberry Pi Night Vision Camera | Visual sensing                                               |
| Gravity BNO055 IMU               | Orientation and heading information                          |
| CHP-20GP-180 Encoder             | Motor movement feedback                                      |
| L298P Motor Shield               | Motor control                                                |
| GEEKSERVO                        | Steering control                                             |
| Helix 1100 mAh 11.1 V 3S LiPo    | Main power source                                            |
| LM2596                           | Raspberry Pi power conversion                                |
| XL4015                           | Motor/control power conversion                               |
| D1-2                             | Positive power distribution                                  |
| PCT-21                           | Negative / ground distribution                               |
| SPST ON/OFF Switch               | Main power control                                           |
| ZX-Switch01                      | Competition start button                                     |

---

# 2. Power System

## 2.1 Battery and Power Distribution

### Battery

| Specification         | Value               |
| --------------------- | ------------------- |
| Model                 | Helix 1100 mAh LiPo |
| Configuration         | 3S                  |
| Nominal Voltage       | 11.1 V              |
| Fully Charged Voltage | 12.6 V              |
| Capacity              | 1100 mAh            |

The robot is powered by a single 3S LiPo battery with a nominal voltage of 11.1 V. The 11.1 V value is the nominal voltage of the battery rather than a constant output voltage. A fully charged 3S LiPo battery reaches approximately 12.6 V, and its voltage gradually decreases during operation.

The battery serves as the main power source for the entire robot. Since the Raspberry Pi, sensors, Arduino, motor driver, and servo operate at different voltage requirements, the battery voltage is regulated before being supplied to the individual subsystems.

The battery capacity is 1100 mAh. In practice, the available operating time depends on the current drawn by the robot and varies significantly with motor load, steering activity, sensor usage, and Raspberry Pi workload.

---

## 2.2 Power Distribution

The robot uses a single battery pack, with the battery output distributed into separate regulated power branches.

The power distribution system is divided into two main branches:

1. **Computing branch** — supplies the Raspberry Pi and its associated electronics through the LM2596 step-down converter.
2. **Motor and control branch** — supplies the motor/control system through the XL4015 step-down converter and motor driver.

This separation was chosen to reduce the effect of rapid current changes in the motor system on the Raspberry Pi and other sensitive computing electronics.

The battery connections are distributed through the electrical connectors shown in the schematic diagram. D1–D2 are used on the positive side of the power distribution, while PCT-21 connectors are used to distribute the negative/common-ground connections.

All regulated subsystems share a common electrical reference through the ground connection.

The overall power flow can be summarized as:

```text
11.1 V 3S LiPo Battery
          │
          ├─────────────── Computing Branch
          │                    │
          │                 LM2596
          │                    │
          │                 ~5.1 V
          │                    │
          │                Raspberry Pi
          │
          └─────────────── Motor / Control Branch
                               │
                            XL4015
                               │
                              ~5 V
                               │
                    Arduino / Motor Driver / Servo
```

The exact physical wiring and component connections are shown in the schematic and wiring diagrams in the `schemes/` directory.

---

## 2.3 Voltage Conversion

| Component                  |                                                       Input |              Output | Supplies               |
| -------------------------- | ----------------------------------------------------------: | ------------------: | ---------------------- |
| LM2596 step-down converter | Battery voltage, approximately 9.6–12.6 V during normal use | Approximately 5.1 V | Raspberry Pi branch    |
| XL4015 step-down converter | Battery voltage, approximately 9.6–12.6 V during normal use |   Approximately 5 V | Motor / control branch |

The LM2596 converter is used to reduce the LiPo battery voltage to approximately 5.1 V for the Raspberry Pi. The output was adjusted slightly above 5.0 V to compensate for voltage losses in wiring and connectors.

The XL4015 converter is used for the motor and control branch. Its output is regulated to the voltage required by the motor-control electronics shown in the schematic.

Both converters are step-down (buck) converters. Their purpose is to convert the higher and variable battery voltage into a lower and more stable voltage suitable for the connected electronics.

The converters provide an important separation between the battery voltage and the low-voltage electronics. This allows the same battery to power the entire robot without directly exposing 5 V electronics to the LiPo battery voltage.

---

## 2.4 Power Budget

Power consumption is divided into load-independent and load-dependent components.

For components whose current depends strongly on the operating condition, the team prefers measured values rather than using a single theoretical value.

| Component             |                             Voltage |    Typical / Reference Current |                                   Peak / Stall Current |
| --------------------- | ----------------------------------: | -----------------------------: | -----------------------------------------------------: |
| Raspberry Pi 5        |                                 5 V |                 Load dependent | Power system designed with up to 5 A supply capability |
| Camera                |                                 5 V | Approximately 250 mA reference |                                         Load dependent |
| RPLiDAR C1            |                                 5 V | Approximately 290 mA reference |                                         Load dependent |
| BNO055 IMU            | 3.3–5 V depending on breakout board |             Low-current sensor |                                          Not specified |
| Arduino UNO R4 Minima |                                 5 V |                 Load dependent |                                          Not specified |
| Drive Motor           |      Depends on motor configuration |         Measure experimentally |                                 Measure experimentally |
| Steering Servo        |                 Approximately 4.8 V |            Approximately 70 mA |                       Approximately 0.8–0.9 A at stall |

The values above should be treated as reference values rather than exact measurements of the final robot. Actual current depends on the specific hardware, workload, motor load, cable losses, and converter efficiency.

The drive motor is especially load-dependent. Its current increases significantly during acceleration, high mechanical load, or near-stall conditions. Therefore, the final motor current budget should be obtained through direct measurement on the completed robot.

### Power Margin

The power system was designed with additional current margin rather than operating each converter continuously at its maximum theoretical output.

The Raspberry Pi branch is given particular attention because the Raspberry Pi 5 can require substantially more current when processing camera data and communicating with external peripherals.

The motor/control branch is also designed to tolerate short-duration increases in current caused by motor acceleration and steering movement.

The final power margin should be verified experimentally by measuring the voltage at the output of each converter while the robot is operating under its highest expected load.

---

## 2.5 Power Reliability

Power reliability is important because the robot contains both high-current actuators and sensitive computing electronics.

The main reliability decisions are:

* A single LiPo battery is used as the main energy source.
* Step-down converters regulate the battery voltage before it reaches low-voltage electronics.
* The computing branch is separated from the motor/control branch.
* The Raspberry Pi supply is adjusted to approximately 5.1 V to compensate for voltage losses in the wiring.
* Power and ground connections are distributed through dedicated connectors and wiring rather than relying on the signal connections between boards.

Separating the power branches reduces the possibility that rapid changes in motor current will directly disturb the Raspberry Pi supply. This is particularly important during acceleration, steering movement, and other situations in which actuator current can change quickly.

During final testing, the team should verify that the voltage at each regulated power rail remains within the acceptable operating range while the robot is running continuously and while the motor and servo are under their highest expected loads.

---


---

# 3. Controllers and Communication

## 3.1 Raspberry Pi 5

**Purpose:** High-level computing and navigation.

The Raspberry Pi 5 processes information from the camera, LiDAR, and IMU and uses this information for the robot's navigation and driving decisions.

**Main responsibilities:**

* Process sensor information
* Perform high-level navigation and decision-making
* Send driving commands to the Arduino UNO R4 Minima

---

## 3.2 Arduino UNO R4 Minima

**Purpose:** Low-level control.

The Arduino UNO R4 Minima handles the drive motor, steering servo, encoder feedback, and physical start button.

**Main responsibilities:**

* Control the drive motor
* Control the steering servo
* Process encoder feedback
* Handle the start button

---

## 3.3 Communication

The Raspberry Pi 5 communicates with the Arduino UNO R4 Minima through a **USB-to-Serial connection**. The actual communication method is Serial communication, with the USB-to-Serial device providing the physical connection between the two controllers.

The Raspberry Pi sends control information to the Arduino, including:

1. **Steering angle** – the angle at which the steering servo should move.
2. **Motor speed** – the target speed of the drive motor.
3. **Motor angle** – the target motor position or rotation angle.

We chose Serial communication because it provides a simple and reliable wired connection between the two controllers. It also allows the Raspberry Pi to send control commands directly to the Arduino without using wireless communication.

---

# 4. Sensors and Hardware Placement

## 4.1 Camera - Raspberry Pi Night Vision Camera Module

<img width="213" height="213" alt="Camera" src="https://github.com/user-attachments/assets/4569d131-1d92-42f9-aca9-de952a0eb49e" />

**Purpose:** Visual detection.

The Raspberry Pi Night Vision Camera is used to provide visual information for the robot's detection and navigation system.

**Why we chose it:** The camera provides visual information that cannot be obtained from distance sensing alone.

**Placement:** The camera is mounted on the upper section of the robot.

**Why this position:** The upper position provides a wider view of the field for visual processing.

### Problem

During the early development of the robot, we found that the original camera lens had a relatively narrow field of view (FOV). This limited the area that the camera could see.

### Final Solution

We replaced the original lens with a wider-angle lens. The final lens provides an approximately **60° field of view**, giving the camera a wider view of the field and more visual information for the navigation system.

---

## 4.2 RPLiDAR C1

<img width="275" height="312" alt="LiDAR" src="https://github.com/user-attachments/assets/c808007d-88ed-45fe-9764-a8ee5f307576" />

**Purpose:** Distance and environmental sensing.

The RPLiDAR C1 provides 2D distance measurements around the robot. The information is used for detecting and navigating around walls, traffic signs, and the parking area.

**Why we chose it:** The robot requires distance information about its surrounding environment in addition to visual information.

**Placement:** The LiDAR is mounted at the front section of the robot.

### Problem

During testing with the previous version of the robot, the LiDAR was mounted at an excessive angle relative to the ground. Because the LiDAR produces a 2D scan, this caused the surrounding field to be represented incorrectly and distorted the resulting 2D map.

### Final Solution

When we built the new version of the robot, we changed the LiDAR mounting position so that it was more parallel to the ground. This produced a more accurate 2D representation of the surrounding environment and removed the major distortion observed in the previous robot.

---

## 4.3 BNO055 IMU

<img width="282" height="239" alt="IMU" src="https://github.com/user-attachments/assets/15e91a09-cf60-4824-845d-28d4c9bf8de7" />

**Purpose:** Orientation and heading feedback.

The Gravity BNO055 provides orientation and heading information for the control system.

**Placement:** The IMU is mounted **underneath the LiDAR** in the current robot.

### Design Evolution

In the previous robot version, the IMU was mounted beside the Raspberry Pi I/O Expansion HAT. This position occupied useful space around the electronics and made the arrangement more crowded.

When building the new robot, we relocated the IMU underneath the LiDAR to make better use of the available space and reduce obstruction around the Raspberry Pi I/O Expansion HAT.

### Problems / Limitations

No significant functional problem was observed with the BNO055 itself during development.

---

## 4.4 Encoder

<img width="329" height="116" alt="motor_encoder" src="https://github.com/user-attachments/assets/9a413340-51db-4588-b1f5-72959419bfdf" />

**Purpose:** Motor movement feedback.

The encoder integrated with the CHP-20GP-180 provides motor movement information that can be used for more precise control.

**Why encoder feedback is important:** The encoder allows the control system to receive feedback about the motor's actual movement instead of relying only on the commanded motor output.

**How the robot uses the feedback:** The Arduino UNO R4 Minima processes the encoder feedback as part of the low-level motor control system.

---

## 4.5 Start / Touch Sensor

<img width="196" height="217" alt="Touch Sensor" src="https://github.com/user-attachments/assets/18c3326b-c0d9-40d7-81cc-c5605b7d4bd8" />

**Purpose:** Competition start control.

The ZX-Switch01 is used as the physical competition start button.

**Placement:** The switch is mounted externally on the robot so that it can be accessed during the competition start procedure.

---

# 5. Sensor Selection and Trade-offs

## 5.1 Selection Criteria

Our sensor system was designed to provide several different types of information:

* Visual information
* Distance information
* Orientation information
* Motor movement feedback

The combination of these sensors allows different parts of the robot's environment and movement to be monitored.

---

## 5.2 Important Sensor Decisions

### Camera

We selected a camera because visual information is required for detecting field features that cannot be identified from distance measurements alone.

The main camera-related design change was the replacement of the original narrow-FOV lens with an approximately 60° lens.

---

### LiDAR vs. Ultrasonic Sensors

Our initial concept used ultrasonic sensors as part of the front sensing system. During development, we changed to RPLiDAR C1 and removed the ultrasonic sensors.

The LiDAR provided 2D environmental information that was more suitable for the navigation approach we adopted. This allowed us to remove the ultrasonic sensors and simplify the front sensing system.

---

### Light Sensor

The first prototype also included a light sensor mounted underneath the front steering structure. The original idea was to use it to detect the blue and orange field markings and help determine when the robot should turn.

After changing to the LiDAR-based sensing approach, the light sensor was removed because it was no longer needed by our navigation system.

---

### IMU

The BNO055 was retained because orientation and heading information remained useful to the control system.

---

# 6. Wiring Architecture

## 6.1 Schematic Diagram

![Schematic Diagram](../schemes/Schematic%20Diagram.png)

The schematic diagram shows the electrical connections between the battery, power-distribution connectors, voltage converters, controllers, sensors, motor-control system, and other electrical components.

The final power distribution uses **D1-2 for positive power** and **PCT-21 for negative / ground**.

---

## 6.2 Wiring Diagram

![Wiring Diagram](../schemes/Wiring%20Diagram.png)

The wiring diagram shows the physical connections used in the final robot and provides a reference for reproducing the electrical system.

---

# 7. Calibration and Sensor Setup

## 7.1 Camera

The main camera adjustment during development was the field of view.

**Problem:** The original lens had a narrow field of view.

**Adjustment:** The original lens was replaced with a wider-angle lens.

**Final setting:** Approximately 60° FOV.

**Result:** A wider area of the field became visible to the camera.

---

## 7.2 LiDAR

The main LiDAR adjustment was its physical mounting angle.

**Problem:** The previous robot had the LiDAR mounted at an excessive angle, which distorted the 2D map.

**Adjustment:** The LiDAR was repositioned to be more parallel to the ground.

**Result:** The 2D scan represented the environment more accurately.

---

## 7.3 IMU

No major functional problem was observed with the IMU during development.

[ADD ACTUAL IMU CALIBRATION PROCEDURE OR VALUES ONLY IF AVAILABLE]

---

# 8. Development History and Iteration

> This section documents the development of the robot itself. V1 did not have a full autonomous test run, so it is documented as an initial concept and design stage rather than as a measured test result.

## 8.1 Version 1 — Initial Prototype

<img width="500" height="550" alt="IMG_2874" src="https://github.com/user-attachments/assets/2cd51408-8423-43bb-8a1c-9e98e6d49be4" />

### Initial Design

Version 1 was our first physical robot concept. The overall shape was similar to the later versions, but its sensing system was significantly different.

The initial design used:

* Ultrasonic sensors at the front
* A light sensor mounted under the front steering structure
* A camera-based sensing concept
* An IMU mounted beside the Raspberry Pi I/O Expansion HAT

The original idea was to use the camera to help detect obstacles, while the ultrasonic sensors provided front distance information. The light sensor was intended to detect the blue and orange field markings and help determine when the robot should turn.

At this stage, the Arduino-side control code had been developed, but the Raspberry Pi autonomous driving software had not yet been implemented.

### Design Decision

After considering the limitations of this sensing approach, we decided to change the sensing architecture and move to a LiDAR-based system instead of continuing to develop the ultrasonic and light-sensor approach.

> **No autonomous test-run results are claimed for V1 because the full Raspberry Pi driving software had not yet been implemented.**

---

## 8.2 Version 2 — LiDAR-Based Prototype

<img width="355" height="355" alt="V2_front" src="https://github.com/user-attachments/assets/565e1a1a-5c90-46f8-8a8d-0b25f9c8d971" />
<img width="355" height="355" alt="V2_back" src="https://github.com/user-attachments/assets/888e36f8-cadd-46e1-8e95-7e89deb52e78" />
<img width="355" height="355" alt="V2_left" src="https://github.com/user-attachments/assets/455b8f9b-da00-4c4a-b316-7e742cbc7fa5" />
<img width="355" height="355" alt="V2_right" src="https://github.com/user-attachments/assets/cefe285f-9064-48ea-a7d9-442a564c19d1" />
<img width="355" height="355" alt="V2_top" src="https://github.com/user-attachments/assets/b6866442-817b-4d32-8025-cc98f1e219f3" />
<img width="355" height="355" alt="V2_buttom" src="https://github.com/user-attachments/assets/657bd3e9-f3c4-46d8-922b-e58752a9c771" />

Version 2 used the same physical robot platform as V1, but the sensing architecture was redesigned.

### Main Changes

* Added RPLiDAR C1
* Removed ultrasonic sensors
* Removed light sensor
* Relocated the BNO055 IMU underneath the LiDAR
* Simplified the front sensing system

The LiDAR-based approach provided the environmental distance information required by our navigation strategy, so the ultrasonic and light sensors were no longer necessary.

After this change, the robot was able to operate with the new sensing architecture, and development continued mainly through software tuning.

---

## 8.3 Version 3 — Final Robot

Version 3 is the current robot shown in the final hardware layout.

Unlike V1 and V2, Version 3 was **completely disassembled and rebuilt as a new physical robot**.

The main changes included:

* A redesigned physical structure
* Improved motor power and speed
* LiDAR retained as the main distance sensor
* IMU retained underneath the LiDAR
* Final electrical layout and component placement

The basic navigation concept remained similar to Version 2, but the drivetrain of Version 3 provides more power and higher speed.

---

# 9. Testing and Software Tuning

## 9.1 Software Iteration

Although the main software development is documented separately, software tuning strongly affected the overall robot development.

During testing, the team repeatedly adjusted control parameters to improve the robot's movement. Examples included parameters such as `wall_margin`.

Some observed problems included:

* The robot sometimes collided with the wall.
* The robot sometimes turned in the wrong direction.
* The robot could become misaligned during a turn.
* In some cases, the robot could rotate continuously in a circle instead of recovering its direction.

These problems were addressed through repeated adjustment and testing of the control parameters.

The software development process is documented in greater detail in the software documentation.

---

## 9.2 Electrical / Sensor Iteration

The main electrical and sensing iterations documented in this project were:

1. Replacing the narrow-FOV camera lens with an approximately 60° lens.
2. Changing the LiDAR mounting angle to improve the accuracy of the 2D scan.
3. Relocating the IMU from beside the Raspberry Pi I/O Expansion HAT to underneath the LiDAR.
4. Replacing the initial ultrasonic/light-sensor concept with LiDAR-based environmental sensing.

---

# 10. Failure Modes and Reliability

| Failure Mode                           | Cause                                           | Effect                                     | Mitigation                                         |
| -------------------------------------- | ----------------------------------------------- | ------------------------------------------ | -------------------------------------------------- |
| Narrow camera FOV                      | Original lens                                   | Limited visible field                      | Replace lens with wider approximately 60° FOV lens |
| Incorrect LiDAR angle                  | Sensor mounted too far from parallel to ground  | Distorted 2D map                           | Reposition LiDAR to be more parallel to the ground |
| Crowded IMU placement                  | IMU mounted beside Raspberry Pi I/O HAT         | Reduced available space around electronics | Relocate IMU underneath LiDAR                      |
| Motor power affecting computing system | Shared electrical power demand                  | Possible instability of computing system   | Use separate regulated power branches              |
| Wall collision                         | Software control parameters not yet fully tuned | Robot may hit wall                         | Continue tuning navigation parameters              |
| Incorrect turning                      | Software control parameters                     | Robot may turn incorrectly                 | Continue software tuning                           |
| Continuous rotation                    | Incorrect control behavior / tuning             | Robot may rotate without recovering        | Continue software tuning and parameter adjustment  |

---

# 11. System-Level Engineering Decisions

## 11.1 Electrical → Mechanical

The placement of electrical and sensing components influenced the physical design of the robot.

For example, the IMU was initially placed beside the Raspberry Pi I/O Expansion HAT. This arrangement occupied useful space around the electronics, so when the robot was rebuilt, the IMU was moved underneath the LiDAR.

The LiDAR also required a mechanically stable and level mounting position because its physical angle directly affected the quality of the 2D scan.

---

## 11.2 Electrical → Software

The navigation software depends on the information provided by the electrical and sensing system.

Changing from ultrasonic sensors and a light sensor to LiDAR changed the type of environmental information available to the navigation algorithm. The camera, LiDAR, IMU, and encoder therefore directly affect how the software controls the robot.

Software parameters such as wall margins and turning behavior were repeatedly adjusted based on the actual sensor behavior and robot movement.

---

## 11.3 Major System Trade-off

> **We chose LiDAR instead of the original ultrasonic and light-sensor approach because LiDAR provided environmental distance information that better matched our navigation strategy.**

This decision simplified the front sensing system by removing sensors that were no longer required and allowed the robot to rely on one main distance-sensing system together with the camera and IMU.

---

# 12. Final Electrical Configuration

The final electrical and sensor layout is shown in the annotated hardware drawings at the beginning of this document.

The final configuration uses:

* Raspberry Pi 5 for high-level processing
* Arduino UNO R4 Minima for low-level control
* RPLiDAR C1 for distance sensing
* Raspberry Pi Night Vision Camera for visual sensing
* BNO055 IMU for orientation and heading
* CHP-20GP-180 encoder for motor feedback
* L298P Motor Shield for motor control
* GEEKSERVO for steering
* Helix 11.1 V 3S LiPo battery
* LM2596 and XL4015 for regulated power distribution

The final robot shown in Version 3 represents the completed physical redesign and is the configuration used for the current development stage.

---

# 13. Reproducibility

The main electrical and sensing components are listed in the final configuration above.

The repository also contains:

* Schematic diagram
* Wiring diagram
* Component layout drawings
* Source code
* Mechanical design files

A team reproducing our electrical system should follow the final wiring diagram and component layout, connect the power distribution according to the documented D1-2 positive and PCT-21 negative connections, and configure the sensors according to their documented positions.

---

# 14. References

| Component / Topic     | Reference |
| --------------------- | --------- |
| Raspberry Pi 5        | https://www.raspberrypi.com/products/raspberry-pi-5/       |
| Arduino UNO R4 Minima | https://docs.arduino.cc/hardware/uno-r4-minima       |
| RPLiDAR C1            | https://www.slamtec.com/en/C1       |
| BNO055                | https://www.dfrobot.com/product-2258.html       |
| Camera                | https://www.waveshare.com/wiki/RPi_Camera_(H)       |
| CHP-20GP-180          | https://www.airsoftmotor.com/micro-dc-reduction-motor/planetary-gear-motor/chp-20gp-180-dc-planetary-gear-motor.html       |
| LM2596                | https://www.ti.com/product/LM2596       |
| XL4015                | https://www.xlsemi.com/datasheet/XL4015-5A-36V-DC-DC-Converter.pdf       |
| L298P                 | https://wiki.dfrobot.com/dri0017/       |

---
