# Mechanical Design Approach

## Structural Prototypes
info-----
___
## Final 3D Structure
### - Main Base
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/123.png" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/models/MainBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Main Base is the robot’s main structural layer, supporting the Raspberry Pi 5, steering servo, and drive motors. Mounting the Raspberry Pi 5 low helps reduce the robot’s overall height, simplify cable routing, and keep the camera’s view clear. The base also keeps the servo and motors properly aligned for balanced weight distribution, while connection points allow the upper layer to be securely attached and maintain a stable structure.



### - Supporting Base 1
info ------

### - Supporting Base 2 (Uno Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/123.png" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/models/UnoBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

### - Supporting Base 3 (Raspberry Pi Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/123.png" width="350">
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
      <img src="https://github.com/123.png" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/d13ff506b7ce7c83859c156f1935925f337665c5/mech/models/ServoBracket.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Servo Bracket securely holds the LEGO-compatible steering servo using built-in cylindrical pegs that act like Technic pins, allowing it to snap into place easily. Its U-shaped design keeps the servo stable and properly aligned with the Main Base while maintaining the correct height and angle for steering. The bracket is also lightweight and easy to 3D print, making it a simple and reliable way to integrate the servo into the custom chassis.


### - Camera Positioning Mechanism
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/123.png" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/842b297ad1873506880c49be992acc7f23d90d8d/mech/models/CamMount.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Camera Positioning Mechanism allows the camera’s height and angle to be easily adjusted for different environments. This is especially useful during competitions, where lighting and arena conditions may differ from testing. The mechanism enables quick on-site adjustments, allowing the camera position to be fine-tuned without redesigning or reprinting any parts.

<table align="center">
  <tr>
    <td><img src="image1.jpg" alt="Camera Arm" width="200"/></td>
    <td><img src="image2.jpg" alt="Camera Arm Connector" width="200"/></td>
    <td><img src="image3.jpg" alt="Camera Plate" width="200"/></td>
  </tr>
</table>


  - __Camera Arm:__
  - __Camera Arm Connector:__
  - __Camera Plate:__

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

## Mechanical parts

### __- Driving motor:__ Chihai 25-370K DC 12V (Gear Ratio 1:20)™
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
The 28-tooth motor drive gear connects directly to the 28-tooth differential gear, creating a 1:1 gear ratio. This allows power to transfer smoothly without changing the motor’s speed or torque. Using matching gears also simplifies the drivetrain, reduces alignment issues, and works seamlessly with LEGO Technic components. Overall, it provides a stable and efficient connection between the motor and differential.


### __- Differential Gear:__ LEGO Technic Differential Gear (28 teeth)
The LEGO Technic Differential Gear (28 teeth) transfers power to the two wheels while allowing them to rotate at different speeds during turns. Its internal bevel gears distribute power smoothly, reducing wheel slip and improving control. The 28-tooth gear also connects easily with LEGO Technic and custom components, making it a reliable choice for the drivetrain and improving the robot’s cornering and maneuverability.

### - Driving Motor with Differential Gear
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
This wheel was chosen because its size provides a good balance between speed, acceleration, and control. Smaller wheels reduce the distance traveled per rotation, while larger wheels require more torque and can make the robot harder to control. The selected medium-sized wheel works well with the chosen motor, allowing the robot to maintain a good ground speed while providing smooth and stable handling.

___
