# Welcome to YBR's Documentation!
This documentation present [Robot Name], YBR's robot for the WRO 2026 Future Engineers competition. It includes an overview of our team, our robot, and the development process behind our competition entry.

# Content

### File's Content
* `team-photos` - This folder contains photos of our team.
* `robot-photos` - This folder contains six photos of the robot, showing it from all sides, as well as the top and bottom.
* `video` - This folder contains a video.md file with links to videos demonstrating the vehicle in operation.
* `schemes` - This folder contains schematic diagrams to illustrate ...!!!!
* `src` - This folder contains the control software code for all components programmed for participation in the competition.
* `mech` - ...!!!!
* `other` - This folder contains additional files and resources related to our robot.
* `LICENSE.md` - This file is the license and terms of use for the materials included in our project.
* `README.md` - This file provides an overview of our team and robot, including the key details of or development process.


### README's Content
* [Our team](#our-team)
* [Our Robot](#our-robot)
* [Mobility and Mechanical Design](#mobility-and-mechanical-design)
* [Power and Sensor Architecture](#power-and-sensor-architecture)
* [Software Architecture and Obstacle Management](#software-architecture-and-obstacle-management)
* [System Thinking and Engineering Decisions](#system-thinking-and-engineering-decisions)
* [Build/Compile/Upload Instructions](#build-compile-upload-instructions)
___

# Our Team
We are YBR, a team from the Science–Mathematics English Program at Yothinburana School, Thailand. Our team consists of three students: Chanakarn Yimsakul, Peradon Nimsongprasert, and Thanphisit Sakulvitulthai, with our mentor, Punnapon Tanasnitikul. Brought together by our interest in robotics, we enjoy learning, solving problems, and turning our ideas into working solutions through every competition. 

![Image](image.jpg)
<table>
  <tr>
    <td><img src="image1.jpg" alt="Image 1" width="200"/></td>
    <td><img src="image2.jpg" alt="Image 2" width="200"/></td>
  </tr>
</table>

### 1. Peradon Nimsongprasert
   - Role
### 2. Chanakarn Yimsakul
   - Role
### 3. Thanphisit Sakulvitulthai
   - Role
___

# Our Robot
Meet [Robot Name], our compact yet powerful robot, built to take on every challenge with confidence.

![Image](image.jpg)

<table>
  <tr>
    <td align="center"><strong>Top View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/top.jpg" width="300"></td>
    <td align="center"><strong>Front View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/front.jpg" width="300"></td>
    <td align="center"><strong>Left View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/left.jpg" width="300"></td>
  </tr>
  <tr>
    <td align="center"><strong>Bottom View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/bottom.jpg" width="300"></td>
    <td align="center"><strong>Back View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/back.jpg" width="300"></td>
    <td align="center"><strong>Right View</strong><br><img src="https://github.com/chankrrn/WRO-FE-YBR/blob/5f683eff9a8eb1d3e4e44c368519a343264e0744/robot-photos/right.jpg" width="300"></td>
  </tr>
</table>


### Performance Video
  - Test: [Run Test](video/RunTest_video.md) 
  - Challenge 1: [Open Challenge Round](https://www.example.com)
  - Challenge 2: [Obstacle Challenge Round](https://www.example.com)

___

# Mobility and Mechanical Design

## Robot Design

### Overview
// Add video animation expplain everything about the robot and how they work

// Or add vdo explain all the component of the robot and find somewhere new to put the vdo above

<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/ThanyawutII/Essent1/blob/main/Screenshot%202025-11-24%20113020.png" width="500">
      <p><a href="https://github.com/Book2009/YB-SUNFLOWER/blob/main/3DModels/Main%20Base.stl" target="_blank">Click here to view the 3D model.</a></p>
    </td>
  </tr>
</table>

- [All 3D-printable models used for our robot can be found here](mech/models)

## Mobility Management

// components' pic (+summarize detail)

- __Dimension:__
- __Drive Motor:__
- __Steering Motor:__
- __3D-Printed Structures:__
- __Customized Mounts:__
- __Steering__
- __Wheels__
- 

[Details of Each 3D Component](https://github.com/chankrrn/WRO-FE-YBR/blob/22eeaa753822f26e70fd81ebcdf6a2261e38e23b/mech/mech_README.md)

### Mechanical parts
![Image of Mechanical parts](image.jpg)
- __Motor(CHP-20-GP180):__ The motor connects to the rear drivetrains to move the robot forward or backwards.
- __Servo(GEEKSERVO 2kg 360 Degrees Servo):__ Servo for steering

### Expansion board/hat
![Image of Expansion board/hat](image.jpg)
- __DFRobot Pi’s OI Expansion HAT:__ The hat enhances the Pi by providing additional IO ports and compatibility with various sensors and modules
- __Arduino R4 Motor Shield:__ An extension with motor driver to control the motor and add IO ports for servo
___

# Power and Sensor Architecture

### Sensors
![Image of Sensors](image.jpg)
- __Lidar (RPLiDAR C1):__ Laser scanner to scan the surroundings in 2D view, detect and avoid the walls, traffic signs and parking space
- __Camera (3.6mm Raspberry Pi IR Camera):__ Use to detect and identify traffic signs and parking spaces by the color
- __IMU (Gravity BNO055):__ Gyroscope and Compass module with 9-axis sensor making a very stable and reliable motion and heading tracking
- __Touch Sensor (ZX-Switch01 by INEX):__ Switch to start the robot since the Arduino or the Pi doesn’t have a switch 
- __Motor Encoder (CHP-20-GP180 Encoder):__ Using two sensors that reads magnetic pulse from a disk connected to the motor reading the speed and direction of the motor


### Electrical Components 
![Image of Electrical Components ](image.jpg)
- __On/Off Switch(SPST ON/OFF Switch 2 Pin Rocker Switch DC 125/250V):__ For cutting power from the battery to the robot
- __Step-down(LM2596):__ The step-down is used to supply the Raspberry Pi with a 5V supply. Because the Pi is sensitive to Voltage fluctuations and electrical noise. We tune the output to 5.1V to compensate for cable and connector losses
- - __Step-down(XL4015):__ The step-down is used to supply the Arduino with a 11V supply. Because the motor will vary in speed and torque related to the voltage of the battery. We tune the output to 11.1V to compensate for cable and connector losses
- __Quick Wire Connector__
    - (PCT-21 Connector): This connector is for the gnd line combining all the negative current to the same spot
    - (D1-2): For this robot’s power distribution system we use a single battery pack, that split into 2 separate branches, one to the LM2596 step-down to power the Pi, and another one to the XL4015 step-down and to the Arduino Motor Shield
- __Battery(Helix 1100 mah 11.1V 3s Lipo-Battery )__ A 3 cells Battery to power the robot

### Computing Components
![Image of Computing Components](image.jpg)
- __Raspberry Pi 5 (Main Board):__ Compute all the values from the LiDAR,Camera,Imu to calculate the walking path and sends the driving command to the Aruduino
- __Arduino r4 minima:__ Commands the driving motor and steering servo. It also calculates the signals from the encoder for precise movements. And it is connected to the starting button

___

# System Thinking and Engineering Decisions



# Build/Compile/Upload Instructions
build instruction (optional)
___

# Software Architecture and Obstacle Management
info การทำงาน^แนวคิด + Flow chart
___

# Conclusion and Result
summarize + future dev
