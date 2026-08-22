# Electrical Design Approach

#### Content
* [Electrical Planning](#electrical-planning)
* [Power Budget](#power-budget)
* [Electrical Components](#electrical-components)
  * [Controller](#controller)
  * [Sensors](#sensors)
  * [Power Management and Distribution](#power-management-and-distribution)
* [Wiring Reference Table](#wiring-reference-table)
* [Calibration Methods](#calibration-methods)
* [Testing & Iteration Log](#testing--iteration-log)
* [References / Datasheets](#references--datasheets)

---

## Electrical Planning

*[Insert Block Diagram / Power Architecture Diagram here — a logical overview showing all power rails and data buses]*

```
[Battery 11.1V 3S] → [SPST Switch] → [PCT-21 splitter (+) / D1-2 splitter (-)]
                                            │
                        ┌───────────────────┴───────────────────┐
                        │                                       │
                 [LM2596 → 5V]                          [XL4015 → motor-side supply]
                        │                                       │
                 [Raspberry Pi 5]                        [L298P Motor Driver]
                        │                                       │
              [IO Expansion HAT (I2C)]                  [DC Motor] [Servo]
                   │         │
            [IMU: I2C]  [LiDAR: UART]
                   │
            [Camera: MIPI-CSI, direct to Pi]
            [Touch Sensor: GPIO, connected to Arduino UNO R4 Minima through the L298P Shield]
```

*(Add a short paragraph explaining why the system is divided into two branches (Pi and Motor) — the reason is to prevent motor current spikes from affecting the Pi 5)*

The robot's electrical system uses a single battery pack and divides the circuit into two independent branches through PCT-21 (positive) and D1-2 (negative):
- **Branch 1**: Through LM2596 → supply the Raspberry Pi 5 and its 5V peripherals. The IMU is connected through the IO Expansion HAT.
- **Branch 2**: Through XL4015 → supply the L298P Motor Shield and Arduino UNO R4 Minima. The shield controls the drive motor and steering system.

Reason for separating the circuits: The Raspberry Pi 5 is sensitive to voltage fluctuations. If it shares a power supply with the motors, the high current spikes during starting and braking could cause the Pi to reset or become unstable. Separating the circuits allows both systems to operate independently and more reliably.

---

## Power Budget

| Component                        |              Voltage |            Current (typ./peak) | Power Source           |
| -------------------------------- | -------------------: | -----------------------------: | ---------------------- |
| **Raspberry Pi 5 (8GB)**         |                   5V |             **5A recommended** | LM2596                 |
| **IO Expansion HAT (DFR0566)**   |                   5V |              **Not specified** | Pi header              |
| **Camera (OV5647 Night Vision)** |                   5V |                 **200–250 mA** | From Pi (CSI)          |
| **RPLiDAR C1**                   |                   5V |                     **230 mA** | From Pi / HAT          |
| **IMU (BNO055 + BMP280)**        |               3.3–5V |                       **5 mA** | IO Expansion HAT (I2C) |
| **Arduino UNO R4 Minima**        |                   5V |              **Not specified** | L298P Shield           |
| **L298P Motor Shield**           | 5V logic / 12V motor |           **Up to 2A/channel** | XL4015                 |
| **DC Gear Motor (CHP-20GP-180)** |                  12V |  **550 mA rated / 2.7A stall** | Through L298P          |
| **Steering Servo**               |                 4.8V | **70 mA rated / 900 mA stall** | Through L298P          |
| **Touch Sensor (ZX-Switch01)**   |                   5V |                     **~10 mA** | L298P Shield           |
| **Total (Pi Branch)**            |                   5V |   **~0.495A + Pi consumption** | LM2596                 |
| **Total (Motor/Arduino Branch)** | **12V motor / 4.8V servo** | **Up to ~3.61A load** | XL4015 / regulated servo supply |



### Power Supply Evaluation

Total (**Motor/Arduino Branch**): The drive motor and steering servo can reach about 3.6A combined at their listed stall currents. The XL4015 IC is specified for up to 5A output in its reference application, so the current capacity is sufficient on paper. However, the actual module temperature, wiring, and servo supply voltage should also be checked during testing.

Total (**Pi Branch**): The known external peripherals require about 0.495A, excluding the Raspberry Pi 5 and the small load of the IO Expansion HAT. The LM2596 is rated up to 2A without a heatsink or 3A with a heatsink, so it is sufficient for the listed peripherals. However, the Raspberry Pi 5 itself can require more power under heavy workload, so the actual Pi input voltage should be monitored during testing.

---

## Electrical Components

### Controller

#### Raspberry Pi 5 (8GB)
The Raspberry Pi 5 (8GB) serves as the main processing unit of the robot. It functions like a high-performance mini computer, capable of handling advanced and computationally intensive tasks such as camera processing, LiDAR data analysis, SLAM, navigation algorithms, and real-time decision-making. Compared to previous generations, the Raspberry Pi 5 provides a major leap in CPU, GPU, and I/O performance, making it ideal for robotics applications that require fast data throughput and reliable multitasking.

We use this board because it delivers significantly higher processing power in the same compact form factor, while also providing improved interfaces for high-bandwidth devices such as multiple cameras, high-speed sensors, and NVMe storage.

| Specification | Value |
|---|---|
| Main Board / SOC | BCM2712 |
| Processor | Quad-core 64-bit ARM Cortex-A76, 2.4 GHz |
| Memory | 8 GB LPDDR4X-4267 SDRAM |
| Wireless | Dual-band Wi-Fi, Bluetooth 5.0 / BLE |
| USB Ports | 2 × USB 3.0, 2 × USB 2.0 |
| GPIO | 40-pin header |
| Camera Interface | 2 × 4-lane MIPI CSI/DSI |
| Power Input | 5V DC via USB-C; 5V/5A recommended for high-power peripherals |
| Operating Temp | 0 – 50 °C |

#### Arduino UNO R4 Minima
The Arduino UNO R4 Minima is the low-level controller for the motor shield. It receives control commands and handles the drive motor, steering, and start button. It uses a 32-bit Renesas RA4M1 Arm Cortex-M4 running at 48 MHz and operates at 5V. It provides 14 digital I/O pins and 6 analog inputs. Arduino specifies a maximum of 8mA per GPIO pin, so higher-current devices such as motors and servos must be powered through an appropriate driver or external supply.

| Specification | Value |
|---|---|
| Main MCU | Renesas RA4M1 |
| Processor | 32-bit Arm Cortex-M4, 48 MHz |
| Operating Voltage | 5V |
| Digital I/O | 14 |
| Analog Inputs | 6 |
| Flash Memory | 256 kB |
| SRAM | 32 kB |
| EEPROM | 8 kB |
| DAC | 12-bit |
| USB | USB-C |
| VIN Input | 6–24V |
| Maximum GPIO Current | 8 mA per pin |

#### IO Expansion HAT for Raspberry Pi (DFR0566)
The IO Expansion HAT acts as the interface between the Raspberry Pi and the IMU. In our final system, the camera and LiDAR are connected directly to the Raspberry Pi, while the IMU is connected through the HAT using I2C. This keeps the sensor wiring organized and allows the Raspberry Pi to focus on the main navigation and vision tasks.

| Specification | Value |
|---|---|
| Supported Platforms | Pi 2B/3B/3B+/4B/Zero/Zero W/**Pi 5** |
| I/O Ports | 24 (Gravity-compatible) |
| Analog Inputs | 6 (via ADS7830 ADC) |
| Communication to Pi | I²C |
| Output Voltage | 5V regulated |
| Port Type | Gravity 3-pin (VCC–GND–Signal) |
| Dimensions | 65 × 56 mm |

#### Motor Driver — L298P
We selected the L298P Motor Shield because it provides two independent motor channels and can control motor speed using PWM. The L298P version supports up to 2A per channel. The shield is mounted directly on the Arduino UNO R4 Minima, which sends the control signals. In our robot, the shield is responsible for the drive motor and steering system. The Raspberry Pi does not directly drive the motor; the Arduino handles the low-level motor control.

---

### Sensors

#### Camera — Raspberry Pi Night Vision Camera Module
The Raspberry Pi Night Vision Camera Module is used as the main vision system of our robot. It connects directly to the Raspberry Pi through the MIPI-CSI interface, allowing high-speed image transmission for real-time processing. The built-in infrared capability ensures that the camera can maintain consistent brightness detection even under uneven lighting conditions, which is especially useful when competing abroad where lighting environments may differ from local testing. Its compact size and fixed-focus lens make it stable, lightweight, and easy to mount on our adjustable camera mechanism.

Thanks to its direct connection to the Raspberry Pi, the camera provides low-latency image data, enabling fast color detection, line tracking, and environmental inspection during autonomous operation.

| Specification | Value |
|---|---|
| Resolution | 5 MP |
| Sensor Format | 1/4 inch |
| Lens Type | Fixed focus |
| Infrared Capability | Yes |
| Interface | MIPI CSI |
| Field of View | ~60° |

**Placement & Justification**: The camera is mounted on the robot's adjustable camera mechanism. The final mounting angle and height should be recorded from the completed robot, together with the distance at which the camera must detect the field features needed for autonomous decisions.

#### LiDAR — RPLiDAR C1
The RPLIDAR C1 was selected as the primary distance measurement sensor of the robot because it provides 360° scanning with a reliable detection range in a compact design. This makes it ideal for mapping the environment, detecting obstacles, and assisting in navigation during competition tasks. By connecting the LiDAR directly to the Raspberry Pi, the system can process scan data in real time with minimal latency. The wide field of view ensures continuous situational awareness, while the lightweight and low-power design makes it easy to integrate into the chassis without adding unnecessary load.

| Specification | Value |
|---|---|
| Distance Range | White: 0.05–12 m / Black: 0.05–6 m |
| Sample Rate | 5 kHz |
| Scanning Frequency | 8–12 Hz (10 Hz typical) |
| Angular Resolution | 0.72° |
| Communication | TTL UART, 460800 bps |
| Accuracy | ±30 mm |
| Weight | 110 g |

**Placement & Justification**: The LiDAR is mounted at the front of the robot to see the traffic sign and the walls besides 

#### Gyro/Compass — Gravity: 10 DOF IMU AHRS (BNO055 + BMP280)
The Gravity: 10 DOF IMU AHRS was chosen as the gyroscope and compass module in the years before we used the ZX-IMU, which suffered from significant drift and instability. The BNO055 integrates a 9-axis sensor with onboard sensor fusion, providing absolute orientation data without the need for complex external algorithms, while the BMP280 adds barometric pressure sensing. This combination delivers highly stable and reliable motion tracking, minimizing drift over time and ensuring consistent heading information.

| Specification | Value |
|---|---|
| Operating Voltage | 3.3–5V DC |
| Operating Current | 5 mA |
| Interface | Gravity-I2C |
| Gyroscope Range | ±125°/s ~ ±2000°/s |
| Accelerometer Range | ±2g ~ ±16g |
| Operating Temp | -40℃ ~ 80℃ |


#### Touch Sensor — ZX-Switch01 (start button)
This button provides an easier way to start the robot, since the controller board does not come with a built-in start switch. The switch is mounted to the frame externally using a bolt.

**Justification**: The ZX-Switch01 is used as the robot's separate start button. It is mounted externally on the robot and connected to the Arduino UNO R4 Minima through the L298P Motor Shield. This keeps the start control separate from the main power switch. The team should verify the final placement against the current WRO rulebook before submission.

---

### Power Management and Distribution

#### On/Off Switch — SPST ON/OFF Switch (2-Pin Rocker, DC 125/250V)
This switch cuts power from the battery to the robot. Competition rules require the robot to be completely switched off before being placed on the field — this switch fulfills that requirement. Wiring: the positive wire (red) is soldered to one side of the switch as input, another red wire is soldered to the opposite side as output to the Quick Wire Connectors(D1-2) and to the two step-down the negative wire(black) from the battery goes directly into QuickWire Connector(PCT-21).

#### Step-down — LM2596 (5V rail for Raspberry Pi)
The LM2596 step-down converter supplies the Raspberry Pi with a dedicated 5V power rail. Since the Pi is sensitive to voltage fluctuations and electrical noise, relying on the same power source as the motors could cause sudden resets or instability. By using this compact module exclusively for the Pi, we ensure a stable supply unaffected by high current changes elsewhere in the system. The output is tuned to around 5.1V to compensate for cable and connector losses, providing a consistent 5.0V input to the Pi during operation.

| Specification | Value |
|---|---|
| Input voltage | DC 4.5–40V |
| Output voltage | DC 1.25–37V (adjustable) |
| Output current | Up to 2A (no heatsink), 3A (w/ heatsink) |
| Conversion efficiency | Up to 92% |
| Module dimension | ~43 × 21 × 14 mm |

#### Step-down — XL4015 (motor rail)
The XL4015 is used as the motor-side step-down converter. It separates the higher-current drivetrain supply from the Raspberry Pi power rail, reducing the effect of motor current changes on the main computer. We tune the step-down output voltage to 11.1V enough for the motor to move at a consistent rate until the battery current goes below 11.

| Specification | Value |
|---|---|
| Input voltage | DC 4.0 ~ 38V |
| Output voltage | DC 1.25V ~ 36V continuously adjustable |
| Output current | Max 5A |
| Conversion efficiency | Up to about 96% |

#### Quick Wire Connectors — PCT-21 & D1-2
The robot's power distribution system is built around a single battery pack, split into two branches to supply both the Raspberry Pi and the motor shield. The positive pole connects through the **D1-2** connector, dividing into multiple outputs — one to each step-down converter. The negative pole is handled by the **PCT-21** connector, providing a secure and stable ground reference split between the converters. This arrangement ensures both control logic and drivetrain receive isolated, stable power from the same battery source, while the connectors simplify wiring, improve safety, and make the system easier to maintain.

#### Battery — Helix 1100mah 11.1V 3-Cell LiPo Battery
The robot uses an 11.1V 3-cell LiPo battery.

| Specification | Value |
|---|---|
| Voltage | 11.1V (3S) |
| Capacity | 1100 mAh |
| Discharge rate | 30C |
| Charging current | Up to 5C |
| Connector | Dean-type |

**Reason for selection**: The 3S battery provides a nominal 11.1V supply and a fully charged voltage of 12.6V. Perfect to drive the motor and with a 1100mah capacity it can power the robot for a long enough time. We chose this battery with its compact size and the availability of it, we can order it and receive it in 3 days.

---

## Wiring Reference Table

| From | To | Pin / Port | Function |
|---|---|---|---|
| LiPo Battery (+) | PCT-21 | — | Splits the positive power rail |
| LiPo Battery (–) | D1-2 | — | Splits the ground rail |
| PCT-21 out 1 | SPST Switch | — | Power ON/OFF control |
| SPST Switch | LM2596 IN+ | — | 5V rail input |
| SPST Switch | XL4015 IN+ | — | Motor rail input |
| LM2596 OUT | Raspberry Pi 5 | USB-C | Main computing power |
| Pi 5 GPIO/I2C | IO Expansion HAT | Gravity I2C | Sensor bus |
| IO HAT | IMU (BNO055) | I2C | Orientation data |
| Pi 5 | RPLiDAR C1 | UART | Distance scan data |
| Pi 5 | Camera | MIPI-CSI | Vision |
| XL4015 OUT | L298P | VIN / motor supply | Motor power |
| L298P | DC Gear Motor | Motor output channel | Drive motor power and control |
| Arduino UNO R4 Minima / L298P Shield | Steering Servo | PWM / servo output | Steering control |
| Arduino UNO R4 Minima / L298P Shield | ZX-Switch01 | GPIO | Start button |

*(This table is very important for Criterion 5: Reproducibility — complete it with every actual connection from the wiring diagram)*

---

## Calibration Methods

- **Camera**: Set red/green/lane color thresholds during setup using representative field images. Record the final threshold values used by the program and recalibrate if the competition lighting is significantly different.
- **IMU**: Keep the robot stationary at startup and perform the sensor's required calibration before autonomous movement. Record the final heading/offset procedure used by the program.
- **LiDAR**: Use the manufacturer's usable range as the initial limit, then verify the minimum and maximum reliable distances on the actual competition field before final testing.


---

## References / Datasheets

- Raspberry Pi 5: https://www.raspberrypi.com/products/raspberry-pi-5/
- Arduino UNO R4 Minima: https://docs.arduino.cc/hardware/uno-r4-minima
- IO Expansion HAT DFR0566: https://wiki.dfrobot.com/dfr0566
- RPLiDAR C1: https://www.slamtec.com/en/C1
- BNO055 + BMP280: https://www.dfrobot.com/product-2258.html
- L298P Motor Shield: https://wiki.dfrobot.com/dri0017/
- XL4015: https://www.xlsemi.com/datasheet/XL4015-5A-36V-DC-DC-Converter.pdf
