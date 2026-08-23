# Electrical Design Approach

This document describes the electrical and sensing architecture of our WRO Future Engineers 2026 robot. It explains the main controllers, sensors, power distribution, wiring, calibration methods, and the reasoning behind our electrical design choices.

Our electrical design follows the same overall philosophy as the rest of the robot: **prioritize stability, reliability, and predictable control over unnecessary complexity**.

## Content

- [Electrical Planning](#electrical-planning)
- [Power Budget](#power-budget)
- [Electrical Components](#electrical-components)
  - [Controller](#controller)
  - [Sensors](#sensors)
  - [Power Management and Distribution](#power-management-and-distribution)
- [Wiring Reference Table](#wiring-reference-table)
- [Calibration Methods](#calibration-methods)
- [Testing & Iteration](#testing--iteration)
- [References / Datasheets](#references--datasheets)

---

## Electrical Planning

Our robot uses a single **11.1 V 3S LiPo battery** as its main power source. The electrical system is then divided into separate regulated branches for the computing system and the motor/control system.

### High-Level Electrical Architecture

```text
[11.1 V 3S LiPo Battery]
           │
      [Main Switch]
           │
     [Power Distribution]
        ┌──┴───────────────┐
        │                  │
 [LM2596 → 5 V]     [XL4015 → Motor-side supply]
        │                  │
 [Raspberry Pi 5]   [L298P Motor Shield]
        │                  │
   ┌────┼────┐       ┌─────┴─────┐
   │    │    │       │           │
 Camera LiDAR IMU   DC Motor   Steering Servo
   │    │    │
  CSI  UART I²C
        
 Arduino UNO R4 Minima
        │
        └── Low-level motor, steering, and start-button control
```

The system is divided into two main power branches. The Raspberry Pi branch uses the **LM2596** to provide a dedicated 5 V rail for the Raspberry Pi and its peripherals. The motor/control branch uses the **XL4015** for the L298P motor-control system and Arduino.

We selected this arrangement because motor operation can produce larger current changes than the computing and sensing system. Keeping the computing supply on its own regulated branch reduces the likelihood that motor-related voltage fluctuations will affect the Raspberry Pi and helps the two parts of the system operate more reliably.

The detailed schematic and wiring diagrams are stored in the [`schemes`](../schemes/) directory.

---

## Power Budget

The following table summarizes the electrical loads identified during our design process. Values are based on the component information used by the team.

| Component | Voltage | Current (typ./peak) | Power Source |
|---|---:|---:|---|
| Raspberry Pi 5 (8GB) | 5 V | 5 A recommended | LM2596 |
| IO Expansion HAT (DFR0566) | 5 V | Not specified | Pi header |
| Camera (OV5647 Night Vision) | 5 V | 200–250 mA | From Pi |
| RPLiDAR C1 | 5 V | 230 mA | From Pi / HAT |
| IMU (BNO055 + BMP280) | 3.3–5 V | 5 mA | IO Expansion HAT (I²C) |
| Arduino UNO R4 Minima | 5 V | Not specified | L298P Shield |
| L298P Motor Shield | 5 V logic / 12 V motor | Up to 2 A/channel | XL4015 |
| DC Gear Motor (CHP-20GP-180) | 12 V | 550 mA rated / 2.7 A stall | Through L298P |
| Steering Servo | 4.8 V | 70 mA rated / 900 mA stall | Through L298P |
| Touch Sensor (ZX-Switch01) | 5 V | ~10 mA | L298P Shield |

### Power Supply Evaluation

For the motor/control branch, the listed stall values of the drive motor and steering servo give a combined worst-case figure of approximately **3.6 A**. The XL4015 is rated for up to 5 A output, so the selected converter provides current capacity above this listed figure. Actual temperature, wiring losses, and operating conditions should still be considered during testing.

For the Raspberry Pi branch, the known external peripherals listed above require approximately **0.495 A**, excluding the Raspberry Pi itself and the small load of the IO Expansion HAT. The LM2596 is specified for up to 2 A without a heatsink and up to 3 A with a heatsink. Because the Raspberry Pi can require substantially more power under heavy workloads, the team treats the Pi's actual supply voltage as an important reliability consideration.

The power budget therefore influenced our architecture: the motor and computing systems were intentionally supplied through separate regulated branches rather than treating all loads as one undivided supply.

---

## Electrical Components

### Controller

#### Raspberry Pi 5 (8GB)

The **Raspberry Pi 5 (8GB)** serves as the main high-level processing unit of the robot. It is responsible for computationally demanding tasks such as camera processing, LiDAR data processing, navigation logic, and real-time decision-making.

We selected the Raspberry Pi 5 because it provides substantially higher processing capability in a compact form factor, while also providing the interfaces required by our sensors and camera system.

| Specification | Value |
|---|---|
| Main SoC | BCM2712 |
| Processor | Quad-core 64-bit ARM Cortex-A76, 2.4 GHz |
| Memory | 8 GB LPDDR4X-4267 SDRAM |
| Wireless hardware | Dual-band Wi-Fi, Bluetooth 5.0 / BLE |
| USB | 2 × USB 3.0, 2 × USB 2.0 |
| GPIO | 40-pin header |
| Camera Interface | 2 × 4-lane MIPI CSI/DSI |
| Power Input | 5 V DC via USB-C; 5 V/5 A recommended for high-power peripherals |
| Operating Temperature | 0–50 °C |

Wireless hardware is not used as part of the robot's competition control system.

#### Arduino UNO R4 Minima

The **Arduino UNO R4 Minima** is used as the low-level controller for the motor-control system. It receives control commands and handles the drive motor, steering, and start-button functions.

Using a separate low-level controller allows the time-sensitive motor and steering functions to remain separate from the higher-level processing performed on the Raspberry Pi.

| Specification | Value |
|---|---|
| Main MCU | Renesas RA4M1 |
| Processor | 32-bit Arm Cortex-M4, 48 MHz |
| Operating Voltage | 5 V |
| Digital I/O | 14 |
| Analog Inputs | 6 |
| Flash Memory | 256 kB |
| SRAM | 32 kB |
| EEPROM | 8 kB |
| DAC | 12-bit |
| USB | USB-C |
| VIN Input | 6–24 V |
| Maximum GPIO Current | 8 mA per pin |

Higher-current devices such as the motor and steering servo are therefore controlled through the motor driver / appropriate power path rather than directly from an Arduino GPIO pin.

#### IO Expansion HAT for Raspberry Pi (DFR0566)

The **DFR0566 IO Expansion HAT** provides an organized interface between the Raspberry Pi and the connected sensors. In our final system, the IMU is connected through the HAT using I²C, while the camera and LiDAR are connected directly to the Raspberry Pi.

This arrangement keeps the sensor wiring organized while allowing the Raspberry Pi to access the different interfaces required by the sensing system.

| Specification | Value |
|---|---|
| Supported Platforms | Pi 2B/3B/3B+/4B/Zero/Zero W/Pi 5 |
| I/O Ports | 24 (Gravity-compatible) |
| Analog Inputs | 6 (via ADS7830 ADC) |
| Communication to Pi | I²C |
| Output Voltage | 5 V regulated |
| Port Type | Gravity 3-pin (VCC–GND–Signal) |
| Dimensions | 65 × 56 mm |

Electrically, the HAT acts as an interface/breakout between the Pi and the connected sensors; it does not perform the main sensor-processing task itself.

#### Motor Driver — L298P

We selected the **L298P Motor Shield** as the motor-control interface between the Arduino and the drive/steering actuators. It provides PWM-based motor control and supports up to 2 A per channel according to the component information used in our design.

The Raspberry Pi does not directly drive the motor. Instead, the Raspberry Pi handles higher-level processing while the Arduino and L298P handle the lower-level actuator control.

---

## Sensors

### Camera — Raspberry Pi Night Vision Camera Module

The **Raspberry Pi Night Vision Camera Module** is the main vision sensor of the robot. It connects directly to the Raspberry Pi through the MIPI-CSI interface for image acquisition and processing.

The camera was selected because its compact form factor, fixed-focus lens, and infrared capability make it suitable for mounting on our adjustable camera mechanism and for obtaining visual information during autonomous operation.

| Specification | Value |
|---|---|
| Resolution | 5 MP |
| Sensor Format | 1/4 inch |
| Lens Type | Fixed focus |
| Infrared Capability | Yes |
| Interface | MIPI CSI |
| Field of View | ~60° |

### Placement and Justification

The camera is mounted on the robot's adjustable camera mechanism. The mechanism allows its position to be adjusted during development so that the field features required by the software can be captured within the camera's field of view.

The final competition configuration is shown in the robot photographs and wiring documentation in this repository.

### LiDAR — RPLiDAR C1

The **RPLiDAR C1** was selected as the primary distance-measurement sensor because it provides **360° scanning** in a compact package. This gives the robot access to distance information from many directions while navigating the field.

The LiDAR is connected to the Raspberry Pi, allowing its scan data to be processed together with camera and IMU information.

| Specification | Value |
|---|---|
| Distance Range | White: 0.05–12 m / Black: 0.05–6 m |
| Sample Rate | 5 kHz |
| Scanning Frequency | 8–12 Hz (10 Hz typical) |
| Angular Resolution | 0.72° |
| Communication | TTL UART, 460800 bps |
| Accuracy | ±30 mm |
| Weight | 110 g |

The LiDAR is mounted at the front of the robot so that it can observe walls and traffic-sign locations ahead of the vehicle while still providing a wide angular view around the robot.

The RPLiDAR C1 connects to the Raspberry Pi 5 through its official USB-to-Serial adapter board. Power is also supplied through this USB connection.

### Gyro/Compass — Gravity: 10 DOF IMU AHRS (BNO055 + BMP280)

The **Gravity: 10 DOF IMU AHRS** uses the BNO055 as the main inertial/orientation sensor and also includes a BMP280 pressure sensor.

This module was selected after our earlier experience with the **ZX-IMU**, which suffered from drift and instability. The BNO055 provides onboard sensor fusion and orientation information, reducing the amount of external processing required to obtain heading information.

| Specification | Value |
|---|---|
| Operating Voltage | 3.3–5 V DC |
| Operating Current | 5 mA |
| Interface | Gravity-I²C |
| Gyroscope Range | ±125°/s to ±2000°/s |
| Accelerometer Range | ±2 g to ±16 g |
| Operating Temperature | −40 °C to 80 °C |

The IMU is connected to the Raspberry Pi through the IO Expansion HAT using I²C.

### Motor Encoder

The **CHP-20GP-180** drive motor includes a dual-phase encoder. Encoder feedback is used to obtain information about motor rotation and to support more precise closed-loop control of the drivetrain.

The encoder is therefore part of the electrical sensing and control system as well as the mechanical drivetrain.

### Touch Sensor — ZX-Switch01 (Start Button)

The **ZX-Switch01** is used as the robot's dedicated Start button. It is mounted externally on the robot using a bolt and is connected to the Arduino UNO R4 Minima through the L298P Shield.

This keeps the competition Start function separate from the main power switch: the main switch powers the robot, while the Start button begins the programmed autonomous action.

---

## Power Management and Distribution

### On/Off Switch — SPST ON/OFF Switch

The SPST switch is used as the robot's main power switch. It disconnects the battery supply from the rest of the electrical system so that the vehicle can be placed on the field in a fully powered-off state before the start procedure.

The switch and power-distribution connectors form the first stage of the electrical system before the supply is divided into the regulated branches.

### Step-down Converter — LM2596 (5 V rail for Raspberry Pi)

The **LM2596** supplies the Raspberry Pi branch with a dedicated regulated voltage.

We use this converter so that the computing system does not rely directly on the changing battery voltage. The output is adjusted to approximately **5.1 V** to account for voltage losses through cables and connectors and to maintain a stable supply for the Pi during operation.

| Specification | Value |
|---|---|
| Input Voltage | DC 4.5–40 V |
| Output Voltage | DC 1.25–37 V, adjustable |
| Output Current | Up to 2 A without heatsink; 3 A with heatsink |
| Conversion Efficiency | Up to 92% |
| Module Dimension | ~43 × 21 × 14 mm |

### Step-down Converter — XL4015 (motor-side rail)

The **XL4015** is used for the motor/control branch. It provides the regulated supply used by the motor-control system and separates the higher-current drivetrain path from the Raspberry Pi power branch.

The output is adjusted for the drivetrain supply used by our robot.

| Specification | Value |
|---|---|
| Input Voltage | DC 4.0–38 V |
| Output Voltage | DC 1.25–36 V, adjustable |
| Output Current | Max. 5 A |
| Conversion Efficiency | Up to about 96% |

### Quick Wire Connectors — PCT-21 & D1-2

The robot uses quick-wire connectors as part of the battery power-distribution system. They allow the single battery supply to be distributed to the main regulated branches while keeping the wiring compact and easier to maintain.

The exact connector-to-wire assignments are documented in the wiring diagram in [`schemes`](../schemes/).

### Battery — Helix 1100 mAh 11.1 V 3-Cell LiPo

The robot uses a **Helix 1100 mAh 11.1 V 3-cell LiPo battery**.

| Specification | Value |
|---|---|
| Voltage | 11.1 V (3S) |
| Capacity | 1100 mAh |
| Discharge Rate | 30C |
| Charging Current | Up to 5C |
| Connector | Dean-type |

We selected this battery because its 11.1 V nominal output is appropriate for our overall electrical architecture, while its compact size, available capacity, and availability made it practical for our robot.

---

## Wiring Reference Table

The wiring below summarizes the principal electrical connections in the final architecture. The detailed physical routing should be read together with the wiring and schematic diagrams in the `schemes` directory.

| From | To | Pin / Port | Function |
|---|---|---|---|
| LiPo Battery | Power distribution | — | Main battery supply |
| Power distribution | SPST Switch | — | Main power ON/OFF |
| SPST Switch | LM2596 IN+ | — | Input to Pi power branch |
| SPST Switch | XL4015 IN+ | — | Input to motor/control branch |
| LM2596 OUT | Raspberry Pi 5 | USB-C | Main computing power |
| Pi 5 | IO Expansion HAT | GPIO / I²C | Sensor interface |
| IO Expansion HAT | IMU (BNO055) | I²C | Orientation data |
| Pi 5 | RPLiDAR C1 | USB / serial adapter | Distance-scan data |
| Pi 5 | Camera | MIPI-CSI | Vision data |
| XL4015 OUT | L298P | VIN / motor supply | Motor-side power |
| L298P | DC Gear Motor | Motor output | Drive motor power and control |
| Arduino UNO R4 Minima / L298P | Steering Servo | PWM / servo output | Steering control |
| Arduino UNO R4 Minima / L298P | ZX-Switch01 | GPIO | Start button |
| Drive Motor | Encoder input | Dual-phase encoder | Motor feedback |

The exact pin-level mapping used by the final software should be kept synchronized with the source code in the `src` directory.

---

## Calibration Methods

Calibration is used to make sensor readings consistent enough for the robot's autonomous-control system to use them reliably.

### Camera

Before operation, the red/green/lane color thresholds used by the vision system are set using representative images of the field. The final threshold values used by the program should be kept consistent with the competition setup.

### IMU

The robot is kept stationary during startup so that the required IMU initialization/calibration can be performed before autonomous movement. The heading/offset procedure used by the final software is kept consistent with the robot's starting procedure.

### LiDAR

The manufacturer's usable range is used as the initial reference. The robot's software then uses the distance measurements required by the navigation system for wall and object detection.

### Encoder

Encoder feedback is used by the motor-control system to track motor rotation. The encoder is part of the drivetrain feedback loop and therefore supports more precise control than an open-loop drive command alone.

---

## Testing & Iteration

Our electrical design was developed together with the rest of the robot rather than being fixed before the mechanical and software systems were tested.

One important design consideration was the interaction between the motor system and the Raspberry Pi. Motor loads can change much more rapidly than the computing load, so the electrical architecture was deliberately changed to use separate regulated power branches for the computing and motor/control systems.

Another important iteration involved the sensing system. We previously used a **ZX-IMU**, but experienced significant drift and instability. We therefore selected the **BNO055-based IMU** for the final sensing architecture because it provides onboard sensor fusion and more stable orientation information.

The electrical design also evolved as the physical robot was assembled. Connector choices, wiring layout, and sensor interfaces were organized around the final mechanical structure so that components could be accessed and maintained without unnecessary wiring complexity.

### Current Engineering Decisions

| Area | Decision | Reason |
|---|---|---|
| Computing power | Separate regulated branch for Raspberry Pi | Reduce sensitivity to motor-related voltage changes |
| Motor/control power | Separate regulated branch | Support higher-current actuator loads |
| IMU | BNO055-based module | Replace earlier ZX-IMU after drift/instability issues |
| Vision | Raspberry Pi camera via MIPI-CSI | Direct connection to main computer for image processing |
| Distance sensing | RPLiDAR C1 | 360° distance scanning in a compact package |
| Start control | Separate physical button | Keep competition start action distinct from the main power switch |

> **Documentation note:** Quantitative test measurements will only be included where they were actually recorded by the team. We do not invent measurements that were not taken during development.

---

## Electrical Design Considerations and Failure Modes

The electrical system was designed with several potential failure modes in mind.

### Power instability

A voltage disturbance in the computing branch could interrupt Raspberry Pi operation and therefore stop high-level navigation. Our main mitigation is the separate regulated power branch for the Raspberry Pi.

### Motor-related current changes

Motor startup, acceleration, braking, and high-load conditions can create larger electrical demands than normal operation. The motor/control branch was therefore given its own regulator and current-capable distribution path.

### Sensor reliability

Sensor readings can be affected by environmental conditions, wiring, mounting, and calibration. For this reason, calibration is part of the setup process rather than treating raw sensor values as automatically reliable.

### Wiring and connector reliability

The robot contains multiple sensors, actuators, controllers, and power converters in a compact chassis. The wiring architecture uses structured distribution and dedicated interfaces so that connections can be traced and maintained more easily.

---

## References / Datasheets

The following official or component references were used during the electrical design:

- **Raspberry Pi 5:** https://www.raspberrypi.com/products/raspberry-pi-5/
- **Arduino UNO R4 Minima:** https://docs.arduino.cc/hardware/uno-r4-minima
- **IO Expansion HAT DFR0566:** https://wiki.dfrobot.com/dfr0566
- **RPLiDAR C1:** https://www.slamtec.com/en/C1
- **BNO055 + BMP280:** https://www.dfrobot.com/product-2258.html
- **L298P Motor Shield:** https://wiki.dfrobot.com/dri0017/
- **XL4015:** https://www.xlsemi.com/datasheet/XL4015-5A-36V-DC-DC-Converter.pdf

---

## Final Electrical Configuration Summary

The final electrical architecture combines a Raspberry Pi 5 for high-level processing, an Arduino UNO R4 Minima for low-level actuator control, and a sensor suite consisting of a camera, LiDAR, IMU, and motor encoder.

A single 11.1 V 3S LiPo battery supplies the robot, while the electrical architecture separates the computing and motor/control loads into regulated branches. This was chosen to improve power stability while keeping the system compact and maintainable.

The key electrical design decisions were therefore driven by three priorities:

1. **Stable power for computing and sensing**
2. **Reliable control of motors and steering**
3. **Useful and consistent sensor feedback for autonomous navigation**

Together, these decisions form the electrical foundation for the robot's mechanical and software systems.
