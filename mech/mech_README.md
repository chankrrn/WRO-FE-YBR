# Mechanical Design Approach

## Structural Prototypes
info-----
___
## Final 3D Structure
### - Main Base
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/123.png" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/models/MainBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Main Base is the main structural layer of the robot and supports several important components. The Raspberry Pi 5 is mounted on this level to keep it as low as possible, since an extension board is placed on top. This helps reduce the overall height of the robot, makes cable routing easier, and keeps the camera’s field of view clear.

The base also provides mounting points for the Servo Bracket and Motor Mount which hold the steering servo and motors connected to the differential gear system. These mounting points are positioned carefully to keep the components aligned and help distribute the robot’s weight evenly.

In addition, the Main Base has dedicated connection points for the next structural layer. This allows the upper parts of the robot to be securely stacked while keeping the overall structure stable.


### - Supporting Base 1
info ------

### - Supporting Base 2 (Uno Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/123.png" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/models/UnoBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

### - Supporting Base 3 (Raspberry Pi Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/123.png" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/models/PiBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

### - Gear Adapter
The Gear Adapter is a custom-designed part that connects the GM25 motor to LEGO differential gears. Since the GM25 motor shaft cannot connect directly to LEGO gears, the adapter provides a simple and secure way to join the two systems.

The center hole is designed to fit tightly onto the GM25 motor shaft, helping prevent the adapter from slipping while the motor is running. The two side holes are made for LEGO Technic pins, allowing the adapter to connect firmly to the LEGO gear.

This design makes it possible to combine a non-LEGO motor with LEGO drivetrain components while maintaining smooth and reliable power transmission.


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
      <img src="https://github.com/123.png" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/d13ff506b7ce7c83859c156f1935925f337665c5/mech/models/ServoBracket.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Servo Bracket is designed to hold a LEGO-compatible steering servo securely in place. Since the servo normally uses Technic pins instead of screws, the bracket has built-in cylindrical pegs that work like Technic pins. This allows the servo to easily snap into the bracket and stay firmly attached.

The bracket has a simple U-shaped design with side mounting plates that help keep the servo stable and properly aligned with the Main Base. Its shape also places the servo at the right height and angle for the steering system while keeping the part lightweight and easy to 3D print.

Overall, the bracket provides a simple and reliable way to connect the LEGO-style servo to the custom 3D-printed chassis, making the steering system easier to assemble and use.


### - Camera Positioning Mechanism
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/123.png" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/842b297ad1873506880c49be992acc7f23d90d8d/mech/models/CamMount.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

///Cam Mount info

<table align="center">
  <tr>
    <td><img src="image1.jpg" alt="Camera Arm" width="200"/></td>
    <td><img src="image2.jpg" alt="Camera Arm Connector" width="200"/></td>
    <td><img src="image3.jpg" alt="Camera Plate" width="200"/></td>
  </tr>
</table>

The Camera Positioning Mechanism makes it easy to adjust the camera’s height and angle to suit different environments. This is especially helpful during competitions, where the lighting and arena setup may be different from the conditions used during testing.

A key advantage of this design is that the camera can be adjusted quickly on-site. This allows us to fine-tune its position when needed without having to redesign or reprint any parts.


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
The Chihai 25-370K DC 12V gear motor with a 1:20 reduction ratio was chosen as the robot’s drive motor because it provides a good balance between speed and torque. Its relatively high output speed allows the robot to move quickly, while the available torque is enough to handle the load during operation.

The motor also works well with the robot’s power system and has a compact size, making it easy to fit into the chassis without adding too much weight. It is also compatible with the LEGO differential gear, allowing power to be transferred smoothly to the wheels and providing responsive handling.

Overall, this motor was selected because it offers a practical combination of speed, torque, size, and reliability for the robot’s competition requirements.

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
The 28-tooth motor drive gear was chosen to work directly with the 28-tooth differential gear, creating a 1:1 gear ratio. This allows power to be transferred smoothly from the motor to the drivetrain without changing the motor’s speed or torque at this stage.

Using gears with the same number of teeth also keeps the drivetrain simple and helps reduce alignment issues and unnecessary friction. Since the 28-tooth gear is compatible with LEGO Technic components, it can be easily connected to the differential system.

Overall, this gear provides a simple, stable, and efficient connection between the motor and the differential, helping the drivetrain operate smoothly and reliably.

### __- Differential Gear:__ LEGO Technic Differential Gear (28 teeth)
The LEGO Technic Differential Gear (28 teeth) is used to transfer power from the motor to the two wheels while allowing each wheel to rotate at a different speed. This is especially important when the robot turns, since the inner and outer wheels need to travel different distances.

Inside the differential, bevel gears distribute the motor’s rotation to both sides. When the robot moves straight, both wheels rotate at similar speeds. During a turn, the inner wheel can slow down while the outer wheel rotates faster, helping reduce wheel slip and making the robot easier to control.

The 28-tooth outer gear also works well with other LEGO Technic gears, making it easy to connect with custom adapters and non-LEGO components. Overall, the differential improves the robot’s cornering, maneuverability, and drivetrain performance.

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
This servo is used to control the robot’s steering and works together with an ultrasonic sensor to support rotation and positioning. Since it is LEGO-compatible, it is easy to install and integrate into the robot. The servo can be securely mounted using LEGO studs through the holes on its sides, making assembly simple and convenient.

Another useful feature is its dual output shafts, which allow two axles to be connected at the same time. This makes it possible to drive two wheels or gears or to use the servo in other mechanical assemblies. The internal gears are also designed to slip when the servo experiences excessive resistance instead of becoming completely stuck. This helps reduce the risk of damage to the servo and the control electronics.

Overall, this servo provides a practical and reliable solution for the robot’s steering system while remaining easy to integrate with the LEGO-based mechanical structure.

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
There are many different wheel options available, but this wheel was chosen because its size works well with the robot’s drivetrain. Wheels that are too small can reduce the robot’s speed because they cover less distance with each rotation. On the other hand, wheels that are too large require more torque, make acceleration slower, and can make the robot harder to control.

The selected medium-sized wheel provides a good balance between speed, acceleration, and control. When combined with the chosen motor, it allows the robot to achieve a competitive ground speed while maintaining smooth handling and stable movement during tasks.

___
