# Mechanical Design Approach
#### Content
* [Structural Prototypes](#structural-prototypes)
* [Final 3D Structure](#final-3d-structure)
* [Mechanical Choices](#mechanical-choices)
* [Mechanical Parts](#mechanical-parts)
* [Controller](#controller)
___

## Structural Prototypes

<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/main/mech/models/Ver1.png" width="400">
    </td>
  </tr>
</table>

In our first prototype we use a 3D printed base with just a 1:1 gear ratio. We just wanted to test the motor and steering system.




<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/main/mech/models/Ver2.png" width="400">
    </td>
  </tr>
</table>

For the next version we added all the components(except the Lidar, because we went with a ultrasonic at first). We changed the driver gear to a 3D printed 21 tooth gear, changed the steering system to be an ackermann steering, and added all the bearings. The robot was usuable at that stage, but there are still some problems.




<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/main/mech/models/Ver Final.png" width="400">
    </td>
  </tr>
</table>

After we spot the problem with the robot, we then went and try to perfected it, the noticeable part that we added was the LiDAR and the second step-down(XL4015), we changed the gear again lowering the driver gear to a 16 tooth gear, we added more pillar connecting the back-section making it stronger and a rear wing as a handle, add some little changes to the layout of the parts.


___
## Final 3D Structure
### - Main Base
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/MainBase.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/MainBase.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Main Base is the robot’s main structural layer, supporting the Battery, Uno R4, steering servo, and drive motors. Mounting the battery, servo, and drive motor low helps keeps the servo and motors properly aligned for balanced weight distribution, while connection points allow the upper layer to be securely attached and maintain a stable structure.



### - Supporting Base 1 (Electrical Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/ElecPlate.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/ElecPlate.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

This plate holds most of the electrical components hence the name. It holds a step-down(LM2596), both Quick Wire Connectors (PCT-21 & D1-2), the power switch, and the start button switch. And also act as a base for the Pi Base

### - Supporting Base 2 (Uno Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/UnoPlate.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/UnoPlate.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Uno also is also connected to the motor shield, and the second step-down(XL4015)

### - Supporting Base 3 (Raspberry Pi Base)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/PiPlate.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/PiPlate.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Pi also have the I/O HAT and the camera on top of it. And it was the pillar holder for the wing

### - Driver Gear (Gear Adapter)
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/GearAdapter.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/GearAdapter.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Driver Gear is a custom 3D print 16 tooth gear that connects the CHP-20-GP180 motor to the LEGO differential gear. Its center hole fits tightly onto the motor shaft to prevent slipping.


### - Motor Bracket
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/MotorBracket.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/MotorBracket.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Motor Mount is a 3D printed part. Even though it is made of plastic it provide exceptional strength and rigidity for securing the motors. The mount includes threaded holes, allowing screws to be fastened directly without the need for nuts, making the part both compact and easy to assemble.

### - Bearing Mount System
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/BearingSystem.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/BearingSystem.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>


<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/BearingMount.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/efd34ff2dac65304741a8c27362adc24ce5496f1/mech/models/BearingMount.stl">Bearing Mount</a></p>
    </td>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/L%26R_AxleSleeve.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/L%26R_AxleSleeve.stl">Left and Right Axle Sleeve</a></p>
    </td>
     <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/Mid_AxleSleeve.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/Mid_AxleSleeve.stl">Middle Axle Sleeve</a></p>
    </td>
  </tr>
</table>


The Rear Wheel Bearing Mount is designed to hold standard ball bearings that support the differential output shaft. The inner race of the bearing interfaces with the shaft through a custom sleeve, allowing smooth and low-friction rotation, while the outer race is securely fixed within the mount. This configuration keeps the rear axle properly aligned, prevents bending under load, and avoids the friction that would occur if the shaft were to rotate directly against 3D-printed surfaces.

### - Steering Mount
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/SteeringSystem.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/SteeringSystem.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/SteeringAxle.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/SteeringAxle.stl">Steering Axle</a></p>
    </td>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/Top_SteeringMount.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/Top_SteeringMount.stl">Top Steering Mount</a></p>
    </td>
     <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/Bottom_SteeringMount.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/Bottom_SteeringMount.stl">Bottom Steering Mount</a></p>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/L_SteeringArm.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/L_SteeringArm.stl">Left Steering Arm</a></p>
    </td>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/R_SteeringArm.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/R_SteeringArm.stl">Right Steering Arm</a></p>
    </td>
     <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/Top_SteeringCap.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/Top_SteeringCap.stl">Top Steering Cap</a></p>
    </td>
     <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/SteeringLinkageArm.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/SteeringLinkageArm.stl">Steering Linkage Arm</a></p>
    </td>
  </tr>
</table>

### - Servo Bracket
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/ServoBracket.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/ServoBracket.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Servo Bracket securely holds the LEGO-compatible steering servo using built-in cylindrical pegs that act like Technic pins, allowing it to snap into place easily. Its U-shaped design keeps the servo stable and properly aligned with the Main Base while maintaining the correct height and angle for steering. The bracket is also lightweight and easy to 3D print, making it a simple and reliable way to integrate the servo into the custom chassis.


### - Camera Positioning Mechanism
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/CamMount.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/CamMount.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Camera Positioning Mechanism allows the camera’s height and angle to be easily adjusted for different environments. This is especially useful during competitions, where lighting and arena conditions may differ from testing. The mechanism enables quick on-site adjustments, allowing the camera position to be fine-tuned without redesigning or reprinting any parts.


<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/CamPlate.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/CamPlate.stl">Camera Plate</a></p>
    </td>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/CamArm.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/CamArm.stl">Camera Arm</a></p>
    </td>
     <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/CamArmConnector.PNG" width="350">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/CamArmConnector.stl">Camera Arm Connector</a></p>
    </td>
  </tr>
</table>

  - __Camera Plate:__
  - __Camera Arm:__
  - __Camera Arm Connector:__

### - Lidar and IMU mount
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/LiDARMount.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/LiDARMount.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The LiDAR Mount is designed to hold the LiDAR sensor securely in the right position for accurate operation. It has a recessed center area that allows the mount to sit directly above the gyro module without getting in the way. The mounting tabs on both sides help attach the plate firmly to the support structure, while the rounded top provides a stable surface for the LiDAR. This design also keeps the sensor’s field of view clear, allowing it to scan effectively. And with the IMU placement keeping it away from any magnetic field that can affect the accuracy of the gyro.

### - Rear Wing
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/RearWing.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/024d1b6555e8ffe13ea26fbf718020a800f10b2f/mech/models/RearWing.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The Rear wing act as the holder of the robot, even tho it is a real aerodynamic arifoil(S1223 airfoil), but with the robot moving at a very low speed and with the camera arm blocking the ariflow, it doesn't add that much down force

### - Step-down Tray
<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/StepdownTray.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/StepdownTray.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

Holds the XL4015 Step-down on top of the Uno motor shield


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

### __- Motor():__ CHP-20-GP180 DC 12V (Gear Ratio 1:19)™
<image src="https://github.com/chankrrn/WRO-FE-YBR/blob/main/other/DrivingMotor1.png" width = "400"> <image src="https://github.com/chankrrn/WRO-FE-YBR/blob/main/other/DrivingMotor2.jpg" width = "500">

We selected the **CHP-20GP-180 DC 12V gear motor** with a **1:19 gear ratio**. It has a dual-phase quadrature encoder, which gives us real-time feedback on speed and rotation. This allows the robot to control its position more accurately.

We chose the **1:19 ratio** instead of the 1:5 ratio because it provides much higher torque, even though it has a lower maximum speed of about **390 RPM**. The higher torque helps the robot move smoothly at low speeds and make more accurate steering and parking movements.

In previous years, our seniors used normal DC motors without encoders and focused more on speed. However, they found that lower torque made the robot harder to control at low speeds, while the lack of an encoder made accurate parking difficult. Because of this, we decided to prioritize **torque, precision, and control over maximum speed**.

We also chose this motor because we have already tested it in both our prototype and final robot. It has worked reliably without any problems, and it is easy for us to purchase and receive within a week.

Overall, the motor gives us a good balance of **torque, speed, size, precision, and reliability** for the WRO Future Engineers competition.

<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/main/other/DrivingMotor3.jpg" width="600">
    </td>
  </tr>
</table>

The wires are a standard dc dual-phase encoder pinout - 
-	Red - Motor Power input(positive)
-	Black -  Hall effect power supply(GND)
-	Yellow - Encoder Signal B phase (Digital 3)
-	Green - Encoder Signal A phase (Digital 2)
-	Blue - Hall effect power supply(5V)
-	White - Motor Power input(negative)

##### Electrical Specifications

| Specification     | Value    |
|-------------------|----------|
| Voltage           | 12V DC   |
| No-Load Current   | ≤ 280 mA |
| Rated Current     | ≤ 550 mA |
| Stall Current     | ≤ 2.7 A  |

##### Mechanical Specifications

| Specification     | Value    |
|-------------------|----------|
| Gear Ratio        | 1:19     |
| No-Load Speed     | ~780 RPM |
| Rated Speed       | ~680 RPM |
| Rated Torque      | 0.40 kg·cm (0.039 N·m) |
| Stall Torque      | ≥ 2.0 kg·cm (0.196 N·m) |
| No-Load Current   | ≤ 280 mA |
| Rated Current     | ≤ 550 mA |
| Stall Current     | ≤ 2.7 A  |
| Gearbox Length    | 21.5 mm  |

##### Encoder Specifications

| Specification     | Value    |
|-------------------|----------|
| Type              | AB Dual-Phase Hall |
| Resolution        | ~211 PPR |
| Supply Voltage    | 3.3V / 5.0V DC |
| Output            | Square Wave |   



### __- Motor Drive Gear:__ 3D printed Gear 16 Tooth

  <tr>
    <td align="center">
      <img src="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/GearAdapter.PNG" width="400">
      <p><a href="https://github.com/chankrrn/WRO-FE-YBR/blob/708c1327a96642dde2c83fabd07bca4eb706b7ec/mech/models/GearAdapter.stl">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

The 16-tooth motor drive gear connects directly to the 28-tooth differential gear, creating a 4:7 gear ratio. This makes the added torque but at a cost of speed.

During the prototype stage, we used a **28:28 spur gear configuration**, giving us a **1:1 ratio**. Our initial goal was to maximize speed while keeping the drivetrain simple.

However, testing showed that the robot did not have enough torque. It struggled to accelerate smoothly and was difficult to control at low speeds, which made steering corrections and autonomous navigation less consistent.

For the second version, we first changed the gearbox to a **21:28 ratio**. We designed and 3D-printed a 21-tooth gear that could be mounted directly onto the motor shaft, making the drivetrain simpler and more rigid. This increased the torque and improved low-speed control and steering precision.

Later in the final version, we changed the gearbox again to a **16:28 ratio** to further increase the output torque. Although this reduced the maximum speed, it gave the robot even better acceleration, stability, and low-speed control. After testing, we found that this setup gave us the best balance between speed and precision.



### __- Differential Gear:__ LEGO Technic Differential Gear (28 teeth)
<image src = "https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/DifferentialGear1.png" width = "300"> <image src = "https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/DifferentialGear2.jpg" width = "400">

The LEGO Technic Differential Gear (28 teeth) transfers power to the two wheels while allowing them to rotate at different speeds during turns. Its internal bevel gears distribute power smoothly, reducing wheel slip and improving control. The 28-tooth gear also connects easily with LEGO Technic and custom components, making it a reliable choice for the drivetrain and improving the robot’s cornering and maneuverability.

### - Driving Motor with Differential Gear
<image src="https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/DrivingMotor-DiffGear.png" width = "500">

After discussing the motor and differential gear and the reasons for selecting them, the next step is to evaluate the performance of the drivetrain when these two components are connected. This analysis focuses on determining the resulting speed and torque, which are important factors in understanding and improving the robot’s overall performance.

The following section presents the calculation methods used to determine the system’s speed and torque, along with the results obtained from the analysis.


- #### **Step 1: Calculate the gear ratio**
<p>The external gear ratio is determined by the ratio of the number of teeth on the input (driver) and output (driven) gears:</p>
<div class="equation">
    External Gear Ratio = <sup>Number of teeth on driven gear</sup> / <sub>Number of teeth on driver gear</sub> = <sup>28</sup> / <sub>16</sub> = 1.75
</div>

- #### **Step 2: Calculate output RPM**
<p>The output RPM is calculated by applying the external gear reduction to the motor's rated speed (680 RPM for the 1:19 CHP-20GP-180):</p>
<div class="equation">
    Output RPM = Motor Rated RPM / External Gear Ratio = 680 / 1.75 ≈ 389 RPM
</div>
<p>(Using the no-load speed of 780 RPM would give approximately 446 RPM.)</p>

- #### **Step 3: Calculate output torque**
<p>The torque increases proportionally to the gear ratio. With a 1.75:1 reduction, the motor's rated torque (0.0392 N·m) is multiplied:</p>
<div class="equation">
    Output Torque = Motor Torque × External Gear Ratio = 0.0392 × 1.75 ≈ 0.069 N·m
</div>
<p>Since the differential splits torque between two wheels, each wheel receives approximately half:</p>
<div class="equation">
    Per-wheel Torque ≈ 0.035 N·m
</div>

#### **Final Results:**
<ul>
    <li><strong>Output RPM @ differential</strong>: ~389 RPM (rated), ~446 RPM (no-load)</li>
    <li><strong>Output Torque @ differential</strong>: ~0.069 N·m (≈ 0.70 kg·cm)</li>
    <li><strong>Per-wheel Torque</strong>: ~0.035 N·m (≈ 0.35 kg·cm)</li>
</ul>   


### __- Servo:__ GEEKSERVO 2kg 360 Degrees Servo
<image src="https://github.com/chankrrn/WRO-FE-YBR/blob/f7a4c0febed255ff74d3f7db9998b655cd9f77bb/other/servo.png" width = "400">

The GEEKSERVO has a good speed and torque, enough for steering the robot. This servo is compatible with LEGO, making it easy and convenient to build the robot by just putting studs in the hole on the side. For the reliability part we had use the servo since the first Future Engineering even and hasn't have any problem since then. Additionally, the gears inside these servos will 'slip' when the blocking load is too high instead of jamming, helping avoid damage to your servos and boards.

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
