# Mechanical Design Approach
#### Content
* [Structural Prototypes](#structural-prototypes)
* [Final 3D Structure](#final-3d-structure)
* [Mechanical Choices](#mechanical-choices)
* [Mechanical Parts](#mechanical-parts)
* [Controller](#controller)
___

## Structural Prototypes
info-----
___
## Final 3D Structure
### - Main Base
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/MainBase.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/models/MainBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Main Base is the robot’s main structural layer, supporting the Raspberry Pi 5, steering servo, and drive motors. Mounting the Raspberry Pi 5 low helps reduce the robot’s overall height, simplify cable routing, and keep the camera’s view clear. The base also keeps the servo and motors properly aligned for balanced weight distribution, while connection points allow the upper layer to be securely attached and maintain a stable structure.



### - Supporting Base 1 (Electrical Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/ElecBase.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/ElecBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

info ------

### - Supporting Base 2 (Uno Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/UnoBase.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/models/UnoBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

### - Supporting Base 3 (Raspberry Pi Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/PiBase.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/models/PiBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

### - Gear Adapter
The Gear Adapter is a custom part that connects the GM25 motor to the LEGO differential gear. Its center hole fits tightly onto the motor shaft to prevent slipping, while the two side holes allow LEGO Technic pins to secure it to the gear. This simple design allows the non-LEGO motor to work smoothly and reliably with the LEGO drivetrain.


### - Motor Rack
info ------

### - Bearing Mount
info ------

### - Steering Mount
info ------

### - Servo Bracket
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/ServoBracket.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/d13ff506b7ce7c83859c156f1935925f337665c5/mech/models/ServoBracket.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Servo Bracket securely holds the LEGO-compatible steering servo using built-in cylindrical pegs that act like Technic pins, allowing it to snap into place easily. Its U-shaped design keeps the servo stable and properly aligned with the Main Base while maintaining the correct height and angle for steering. The bracket is also lightweight and easy to 3D print, making it a simple and reliable way to integrate the servo into the custom chassis.


### - Camera Positioning Mechanism
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/CamMount.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/842b297ad1873506880c49be992acc7f23d90d8d/mech/models/CamMount.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Camera Positioning Mechanism allows the camera’s height and angle to be easily adjusted for different environments. This is especially useful during competitions, where lighting and arena conditions may differ from testing. The mechanism enables quick on-site adjustments, allowing the camera position to be fine-tuned without redesigning or reprinting any parts.


<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/CamPlate.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/842b297ad1873506880c49be992acc7f23d90d8d/mech/models/CamMount.stl">Camera Plate</a></p>
    </td>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/CamArm.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/842b297ad1873506880c49be992acc7f23d90d8d/mech/models/CamMount.stl">Camera Arm</a></p>
    </td>
     <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/1aea5b02989326ef6efdf00d6d068e656367b088/mech/models/CamArmConnector.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/842b297ad1873506880c49be992acc7f23d90d8d/mech/models/CamMount.stl">Camera Arm Connector</a></p>
    </td>
  </tr>
</table>

  - __Camera Plate:__
  - __Camera Arm:__
  - __Camera Arm Connector:__

### - Ultrasonic Sensor Mount
info ------

### - Lidar Mount
The LiDAR Mount is designed to hold the LiDAR sensor securely in the right position for accurate operation. It has a recessed center area that allows the mount to sit directly above the gyro module without getting in the way. The mounting tabs on both sides help attach the plate firmly to the support structure, while the rounded top provides a stable surface for the LiDAR. This design also keeps the sensor’s field of view clear, allowing it to scan effectively.

### - 3D Printer
  - __Printer name:__ info ------
  - __Filament :__ info-----

___

## Mechanical Choices

### - Wheel Choice
### - Steering System
### - Differential (Rear Wheels)
### - Dimensions Choice
___

## Mechanical Parts

### __- Driving motor:__ Chihai 25-370K DC 12V (Gear Ratio 1:20)™
<image src="https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/DrivingMotor1.png" width = "400"> <image src="https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/DrivingMotor2.jpg" width = "500">


The Chihai 25-370K DC 12V gear motor with a 1:20 reduction ratio was chosen because it provides a good balance of speed and torque. Its compact size makes it easy to fit into the chassis without adding much weight, while its output is strong enough to drive the robot reliably. The motor also works well with the LEGO differential gear, providing smooth power transmission and responsive handling. Overall, it offers a practical combination of speed, torque, size, and reliability for competition use.

##### Electrical Specifications

| Specification     | Value    |
|-------------------|----------|
| Voltage           | 12V DC      |

##### Mechanical Specifications

| Specification     | Value    |
|-------------------|----------|
| RATIO             | 1: 20  |
| no-load current(mA)    | ≤600   |
| no-load speed(rpm)             | 980  |
| rated torque(Kg.cm)    | 2.3   |
| rated torque(N.m)             | 0.21  |
| rated speed (rpm)             | 780  |
| rated current (A)    | ≤2.7   |
| stall current (A)             | ≤16.0  |
| length of gearbox(L)    | 19.0   |


### __- Motor Drive Gear:__ LEGO Technic Gear 28 Tooth
<image src = "https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/MotorDriveGear.png" width = "250">

The 28-tooth motor drive gear connects directly to the 28-tooth differential gear, creating a 1:1 gear ratio. This allows power to transfer smoothly without changing the motor’s speed or torque. Using matching gears also simplifies the drivetrain, reduces alignment issues, and works seamlessly with LEGO Technic components. Overall, it provides a stable and efficient connection between the motor and differential.


### __- Differential Gear:__ LEGO Technic Differential Gear (28 teeth)
<image src = "https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/DifferentialGear1.png" width = "300"> <image src = "https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/DifferentialGear2.jpg" width = "400">

The LEGO Technic Differential Gear (28 teeth) transfers power to the two wheels while allowing them to rotate at different speeds during turns. Its internal bevel gears distribute power smoothly, reducing wheel slip and improving control. The 28-tooth gear also connects easily with LEGO Technic and custom components, making it a reliable choice for the drivetrain and improving the robot’s cornering and maneuverability.

### - Driving Motor with Differential Gear
<image src="https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/DrivingMotor-DiffGear.png" width = "500">

After discussing the motor and differential gear and the reasons for selecting them, the next step is to evaluate the performance of the drivetrain when these two components are connected. This analysis focuses on determining the resulting speed and torque, which are important factors in understanding and improving the robot’s overall performance.

The following section presents the calculation methods used to determine the system’s speed and torque, along with the results obtained from the analysis.

- #### **Step 1: Calculate the gear ratio**
<p>The gear ratio is determined by the ratio of the number of teeth on the input and output gears:</p>
<div class="equation">
    Gear Ratio = <sup>Number of teeth on input gear</sup> / <sub>Number of teeth on output gear</sub> = <sup>28</sup> / <sub>28</sub> = 1.0
</div>

- #### **Step 2: Calculate output RPM**
<p>The output RPM is calculated based on the motor's rated speed and the gear ratio:</p>
<div class="equation">
    Output RPM = Input RPM × Gear Ratio = 780 × 1.0 = 780 RPM
</div>
<p>(Using the no-load speed would give approximately 980 RPM.)</p>

- #### **Step 3: Calculate output torque**
<p>The torque decreases inversely proportional to the gear ratio. With a 1:1 ratio, the torque remains unchanged from the motor:</p>
<div class="equation">
    Output Torque = <sup>Input Torque</sup> / <sub>Gear Ratio</sub> = <sup>0.21</sup> / <sub>1.0</sub> = 0.21 N·m
</div>
<p>Since the differential splits torque between two wheels, each wheel receives approximately half:</p>
<div class="equation">
    Per-wheel Torque ≈ 0.105 N·m
</div>

#### **Final Results:**
<ul>
    <li><strong>Output RPM @ differential</strong>: ~780 RPM (rated), ~980 RPM (no-load)</li>
    <li><strong>Output Torque @ differential</strong>: ~0.21 N·m (≈ 2.3 kg·cm)</li>
    <li><strong>Per-wheel Torque</strong>: ~0.105 N·m (≈ 1.05 kg·cm)</li>
</ul>

### __- Servo:__ GEEKSERVO 2kg 360 Degrees Servo
<image src="https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/servo.png" width = "400">

This servo controls the robot’s steering and works with the ultrasonic sensor for rotation and positioning. Its LEGO-compatible design makes it easy to mount using studs through the side holes. The dual output shafts allow two axles to be connected, while the internal gears can slip under excessive resistance to help protect the servo and electronics. Overall, it provides a simple, reliable, and easy-to-integrate solution for the steering system.

The wires are a standard servo pinout - 
-	Red - positive
-	Brown - negative
-	Yellow - data


##### Electrical Specifications

| Specification     | Value    |
|-------------------|----------|
| Working voltage   | 3.3V~6V  |
| Rated voltage     | 4.8V     |
| Rated current     | 200mA    |
| Stall current     | 700mA    |
| Sliding current   | 450mA    |

### __- Wheel:__ Lego Tire 43.2 x 22 ZR and Wheel 30.4mm D. x 20mm with No Pin Holes and Reinforced Rim
<image src = "https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/wheel1.png" width = "200"> <image src = "https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/wheel2.png" width = "200">

This wheel was chosen because its size provides a good balance between speed, acceleration, and control. Smaller wheels reduce the distance traveled per rotation, while larger wheels require more torque and can make the robot harder to control. The selected medium-sized wheel works well with the chosen motor, allowing the robot to maintain a good ground speed while providing smooth and stable handling.

___


## Controller

### __ - Main Board:__ Raspberry Pi 5 (8GB) from Raspberry Pi
<img src = "https://github.com/pic.png" width = "400">

The Raspberry Pi 5 (8GB) is the main processing unit of the robot, handling tasks such as camera processing, LiDAR data, SLAM, navigation, and real-time decision-making. It provides significantly more processing power than previous models while keeping a compact size, making it well suited for robotics. Its improved high-speed interfaces also allow it to work efficiently with cameras, sensors, and other high-bandwidth devices.

| Specification | Value |
|---------|---------|
| **Main Board / SOC** | BCM2712 |
| **Processor** | Quad-core 64-bit ARM Cortex-A76, 2.4 GHz, with cryptography extensions, 512 KB L2 per core + 2 MB shared L3  |
| **Memory Options** | 1 GB, 2 GB, 4 GB, 8 GB LPDDR4X-4267 SDRAM |
| **Wireless** | Dual-band Wi-Fi (2.4 GHz & 5 GHz, 802.11ac), Bluetooth 5.0 / BLE |
| **Ethernet** | Gigabit Ethernet, with support for PoE+ via separate HAT |
| **USB Ports** | 2 × USB 3.0, 2 × USB 2.0 |
| **GPIO** | 40-pin header (fully backwards compatible) |
| **Display Output** | 2 × micro-HDMI (dual 4Kp60 with HDR) |
| **Camera Interface** | 2 × 4-lane MIPI CSI / DSI transceivers (i.e., supports two cameras or two displays) |
| **Audio/Video** | 4-pole stereo audio and composite video port |
| **Video Support** | H.265 (4kp60 decode), H.264 (1080p60 decode, 1080p30 encode) |
| **Graphics** | OpenGL ES 3.1, Vulkan 1.0 |
| **Storage** | Micro-SD card slot (supports high-speed SDR104) Raspberry Pi BD Also supports PCIe 2.0 x1 for NVMe SSD via HAT |
| **Power Input** | 5V DC via USB-C (min 3A), 5V DC via GPIO (min 3A), Power over Ethernet (PoE with HAT) |
| **Operating Temperature** | 0 – 50 °C ambient |

## Interface Board — IO Expansion HAT for Raspberry Pi (DFR0566)

## Motor Driver — L298N
