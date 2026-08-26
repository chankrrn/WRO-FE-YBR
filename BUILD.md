# Build / Compile / Upload Instructions

This document is the full step-by-step guide to assemble, wire, and program the YBR-SUNFLOWER
robot from scratch. It is meant to be detailed enough that another team could reproduce the
vehicle using only this file plus the design-reasoning docs in `mech/mech_README.md`,
`elec/elec_README.md`, and `software/software_README.md`.

---

## 1. Bill of Materials

## 1. Bill of Materials

List every part needed, with the exact model/part number, quantity, and where in the repo its
CAD/schematic lives. Example table — fill in with your actual parts:

| Category                                                  | Component | Qty | Notes | Where we bought it |
|---|---|---|---|---|
| Compute | Raspberry Pi 5                                  | 1 | The 8GB from where we brough from is no longer available       | https://gammaco.com/gammaco/Raspberry_Pi_GB_89RD014.html |
| Compute | Arduino UNO R4 Minima                           | 1 | We use the normal non wifi Ver                                 | https://www.ortech-online.com/product/1370 |
| Motor | CHP-20GP-180 DC geared motor w/ encoder           | 1 | 1:19 Gear Ratio                                                | https://th.shp.ee/dcTw6X4o |
| Steering | GEEKSERVO 2kg 360° servo                       | 1 | Also no longer available                                       | https://th.shp.ee/xjXZcp6A |
| Sensing | RPLiDAR C1                                      | 1 |                                                                | https://www.dfrobot.com/product-2803.html |
| Sensing | Raspberry Pi Night Vision Camera                | 1 |                                                                | https://th.cytron.io/p-fish-eye-lense-raspberry-pi-5mp-ir-camera |
| Sensing | Gravity BNO055 IMU                              | 1 |                                                                | https://www.dfrobot.com/product-1793.html |
| Control | L298P Motor Shield                              | 1 |                                                                | https://th.shp.ee/DExzLgGb |
| Control | IO Expansion HAT for Raspberry Pi 5 / 4B / 3B+  | 1 |                                                                | https://www.dfrobot.com/product-1930.html |
| Control | ZX-Switch01 start button                        | 1 |                                                                | https://inex.co.th/home/product/zx-switch01/ |
| Power | Helicox 1100mAh 11.1V 3S LiPo                     | 1 |                                                                | https://sl1nk.com/tpaqu28 |
| Power | LM2596 step-down converter                        | 1 | tuned to 5.1V                                                  | https://www.ortech-online.com/product/212 |
| Power | XL4015 step-down converter                        | 1 | tuned to 11.1                                                  | https://www.ortech-online.com/product/206 |
| Power | Quick Wire Connectors                             | 2 | PCT-21 & D1-2                                                  | https://th.shp.ee/pKZ9582e |


---

## 2. Mechanical Assembly

Step-by-step, in build order. Reference photos/CAD renders where possible.

1. **Print/prepare structural parts** — print all the .stl parts in the 'mech/models'. import it into your printer slicer app(For us we use the bambu studio app)


2. **Assemble the Back Part** — we recommend assemble the back part from top to bottom

<div align="center">

```text

Camera
|
v
I/O HAT
|
v
Pi 5
|
v
Electrical Base

```
</div>

<img width="300" height="400" alt="Pi5 Layer" src="other/Pi5 Layer.jpg" />  <img width="400" height="400" alt="Elec Layer" src="other/Elec layer.jpg" />  <img width="300" height="400" alt="Back Section" src="other/Back Part.jpg" /> 

3. **Assemble drivetrain** — Mount motor, install differential and bearing to the chassis.

<img width="300" src="other/Drivetrains.jpg" />

4. **Mount the Uno base and Servo**

<img width="300" src="other/Servo and Uno.jpg" />

5. **Mount steering system** — Install the [Steering Axle](mech/models/SteeringAxle.stl), assemble the [Steering Mount](mech/models/Top_SteeringMount.stl) to the [Steering Arm](mech/models/L_SteeringArm.stl), and mount it to the Axle

<img width="300" src="other/Steering system.jpg" />

6. **Assemble the parts** — Mount the Uno and connect the step-down(XL4015) to the Uno motor shield , and mount on the Back Part

<img width="300" src="other/Arduino mount.jpg" />

7. **Mount LiDAR and IMU** — Mount the IMU on the plate and the screw to the pillar before putting on the Lidar. And mount it to the Base

<img width="300" src="other/Lidar mount.jpg" />

   **Note:** *why* each is placed there (see `mech/mech_README.md` for full reasoning)


---

## 3. Electrical Assembly / Wiring

1. **Battery and power distribution** — Connect the battery **Positive** to the power switch and to the Quick Wire Connector(D1-2) which will branch to the two step-down

-	The LM2596(on the elec base) step-down to the USB-C power to the pi
-	The XL4015(on the motor shield) step-down to the motor shield

	And all the 3 **Negative** (Battery, LM2596, XL4015) all connect to the **PCT-21**Quick Wire Connector

   
3. **Connect Raspberry Pi 5** — Connect the IMU IIC(I2C) wire to the first IIC port on the I/O HAT

4. **Connect Arduino UNO R4 Minima** — Connect the Motor Shield on top of it, connect the starting button to the analog port(1), and the Servo to the servo port on the shield

	For the motor encoder

	The wires are a standard dc dual-phase encoder pinout - 
	-	Motor Pin(+)   | Red - Motor Power input(positive)
	-	GND			   | Black -  Hall effect power supply(GND)
	-	Digital Pin(3) | Yellow - Encoder Signal B phase (Digital 3)
	-	Digital Pin(2) | Green - Encoder Signal A phase (Digital 2)
	-	5V			   | Blue - Hall effect power supply(5V)
	-	Motor Pin(-)   | )White - Motor Power input(negative)

4. **Pi ↔ Arduino communication link** — Connect the USB-C of the Arduino to the **Lower** USB-A Port(USB3.0, the blue one)

5. **LiDAR ↔ Pi** — Connect the UART of the Lidar to the adapter and connect the USB-C from the adapter to the **Upper** UBS-A Port(USB3.0, the blue one)

Link to the full wiring diagram in `schemes/Wiring Diagram.png` and the schematic in `schemes/Schematic Diagram.png`

---

## 4. Software Setup

### 4.1 Raspberry Pi environment

1. Install the Pi OS if you have not **[Install Guilde](https://www.raspberrypi.com/documentation/computers/getting-started.html#imager-install)**

	**Note:** We use the normal Pi OS (64-bit)

3.	After downloading Pi OS you have to Enable **[Remote SSH](https://www.raspberrypi.com/documentation/computers/remote-access.html#ssh)**
4.	Now open the terminal and run the following command to install all the libraries  


```bash
# downloads the code and unpacks it. 
curl -L https://github.com/chankrrn/WRO-FE-YBR/archive/refs/heads/main.tar.gz | tar xz

# moves into the folder with the robot code
cd WRO-FE-YBR-SUNFLOWER-main/src/Raspberrypi

# Install apt packages, enables I2C and the camera, adds your user to dialout/i2c/gpio, installs uv, then builds the Python environment from uv.lock. It'll ask for your password at the sudo steps.
bash setup_pi.sh

# Required. The group permissions and the I2C setting don't take effect until you restart.
sudo reboot

```
After running the command you should test if the installing works

```bash

cd ~/WRO-FE-YBR-SUNFLOWER-main/src/Raspberrypi
i2cdetect -y 1                    # expansion board + BNO055 should show up
ls /dev/ttyACM0 /dev/ttyUSB0      # Arduino + lidar
uv run python main.py qualification --dry-run

```


### 4.2 Arduino environment
In the Arduino IDE you need to Install two libraries **[Servo.h](https://docs.arduino.cc/libraries/servo/)** and **[PID_v2.h](https://docs.arduino.cc/libraries/pid_v2/)**
1. Click on the Library Manager on the left bar
2. 
<img src="other/Screenshot-Open-lib_manager.png" width="800">

3. Search **Servo** and **PID_v2** and click install

<img src="other/Screenshot-Install-Servo.png" width="395"> <img src="other/Screenshot-Install-PID_V2.png" width="395">


## 5. Compile & Upload

### 5.1 Arduino (low-level motor/steering control)

1. Open `src/<arduino-folder>/<sketch-name>.ino` in the Arduino IDE.
2. Select board: **Arduino UNO R4 Minima**.
3. Select the correct COM/serial port.
4. Click Upload.
5. Confirm upload success (describe expected serial monitor output / LED blink pattern).

### 5.2 Raspberry Pi (high-level navigation)

1. Navigate to `src/<pi-folder>/`.
2. Run the main script:
   ```bash
   # go to the Program folder
   cd FutureE/src/Raspberrypi/

   # run the program
   uv run python main.py final
   ```
3. You should expected "waits for start button press", or the camera preview should appear if you added "--debug--"

---

## 6. First Run / Verification Checklist

- [ ] Motor spins in the correct direction when commanded forward
- [ ] Servo centers correctly and turns both directions
- [ ] Encoder counts increment as expected
- [ ] LiDAR returns a valid point cloud / distance readings
- [ ] Camera feed is live and traffic sign colors are detected correctly
- [ ] IMU heading updates correctly when the robot is rotated
- [ ] Start button correctly transitions robot from OFF → WAITING → RUNNING
- [ ] Robot completes a straight-line test drive
- [ ] Robot completes a basic turn test

---

## 7. Troubleshooting

Common issues and fixes — fill in as you encounter them during testing, e.g.:

| Symptom | Likely Cause | Fix |
|---|---|---|
| Pi doesn't see Arduino | The Uno wasn't rest or in bootloder | Click the reset button **ONCE**
| Pi doesn't see LiDAR | USB permissions / wrong port | `sudo chmod 666 /dev/ttyUSBx` or fix udev rule |
| Servo jitters | Insufficient power to servo rail | Check step-down converter output under load |
| Arduino not detected | Wrong board package installed | Reinstall UNO R4 board definitions |

---

## 8. References

- Mechanical design reasoning: [`mech/mech_README.md`](./mech/mech_README.md)
- Electrical design reasoning: [`elec/elec_README.md`](./elec/elec_README.md)
- Software architecture reasoning: [`software/software_README.md`](./software/software_README.md)
- Wiring diagram: [`schemes/Wiring Diagram.png`](./schemes/Wiring%20Diagram.png)
- Schematic diagram: [`schemes/Schematic Diagram.png`](./schemes/Schematic%20Diagram.png)
