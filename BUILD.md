# Build / Compile / Upload Instructions

This document is the full step-by-step guide to assemble, wire, and program the YBR-SUNFLOWER
robot from scratch. It is meant to be detailed enough that another team could reproduce the
vehicle using only this file plus the design-reasoning docs in `mech/mech_README.md`,
`elec/elec_README.md`, and `software/software_README.md`.

---

## 1. Bill of Materials

List every part needed, with the exact model/part number, quantity, and where in the repo its
CAD/schematic lives. Example table — fill in with your actual parts:

| Category | Component | Qty | Notes | Where we bought it |
|---|---|---|---|---|
| Compute | Raspberry Pi 5 | 1 | The 8GB from where we brough from is no longer available | https://gammaco.com/gammaco/Raspberry_Pi_GB_89RD014.html |
| Compute | Arduino UNO R4 Minima | 1 | We use the normal non wifi Ver | https://th.shp.ee/tbrwLwsx |
| Motor | CHP-20GP-180 DC geared motor w/ encoder | 1 | 1:19 Gear Ratio | https://th.shp.ee/dcTw6X4o |
| Steering | GEEKSERVO 2kg 360° servo | 1 | Also no longer available | https://th.shp.ee/xjXZcp6A |
| Sensing | RPLiDAR C1 | 1 | | https://www.dfrobot.com/product-2803.html |
| Sensing | Raspberry Pi Night Vision Camera | 1 | | https://th.cytron.io/p-fish-eye-lense-raspberry-pi-5mp-ir-camera |
| Sensing | Gravity BNO055 IMU | 1 | | https://www.dfrobot.com/product-1793.html |
| Control | L298P Motor Shield | 1 | | |
| Control | IO Expansion HAT for Raspberry Pi 5 / 4B / 3B+ | 1 | | https://www.dfrobot.com/product-1930.html |
| Control | ZX-Switch01 start button | 1 | | https://inex.co.th/home/product/zx-switch01/ |
| Power | Helicox 1100mAh 11.1V 3S LiPo | 1 | | https://sl1nk.com/tpaqu28 |
| Power | LM2596 step-down converter | 1 | tuned to 5.1V | |
| Power | XL4015 step-down converter | 1 | tuned to 11.1 | |
| — | (add fasteners, wire gauges, connectors, etc.) | | | |

Add tools required too: soldering iron, hex driver set, 3D printer + filament type, etc.

---

## 2. Mechanical Assembly

Step-by-step, in build order. Reference photos/CAD renders where possible.

1. **Print/prepare structural parts** — list which STL/STEP files to print, printer settings
   (layer height, infill) if relevant. Link to `mech/models/`.
2. **Assemble drivetrain** — mount motor, install gearbox (21:28 ratio), attach to chassis.
3. **Mount steering system** — install servo, connect to steering linkage.
4. **Attach wheels.**
5. **Mount chassis plates / frame.**
6. **Mount sensors** — LiDAR position, camera position and angle, IMU placement. Note *why*
   each is placed there (see `mech/mech_README.md` for full reasoning) but give the *physical
   how-to* here (screw pattern, height, angle in degrees).
7. **Final dimension check** — confirm the assembled vehicle is within 300×200×300 mm and
   ≤1.5 kg (WRO requirement).

Include a photo or diagram after each major step if possible.

---

## 3. Electrical Assembly / Wiring

1. **Battery and power distribution** — how the 3S LiPo connects to the two step-down
   branches (Pi branch via LM2596, motor branch via XL4015 + L298P).
2. **Connect Raspberry Pi 5** — power input, camera ribbon cable, LiDAR (USB or UART?),
   IMU (I2C pins).
3. **Connect Arduino UNO R4 Minima** — motor shield, encoder wires, servo signal wire,
   start button wiring.
4. **Pi ↔ Arduino communication link** — specify interface (USB serial / I2C / UART) and
   baud rate.
5. **Switches** — placement and wiring of the two power switches (per WRO rule 9.10, only
   one switch may power on the whole vehicle).

Link to the full wiring diagram in `schemes/Wiring Diagram.png` and the schematic in
`schemes/Schematic Diagram.png`. State the exact pin mapping (e.g. a table: Arduino pin → function).

---

## 4. Software Setup

### 4.1 Raspberry Pi environment

```bash
# OS used, version
# e.g. Raspberry Pi OS (64-bit), kernel version X

# Clone the repo
git clone https://github.com/chankrrn/WRO-FE-YBR.git
cd WRO-FE-YBR/src

# Install dependencies
# e.g. pip install -r requirements.txt
# apt install <lidar driver package>, etc.
```

List every library/package needed (OpenCV version, LiDAR SDK, IMU library, etc.) and how to
install each — this is what makes the build reproducible for a judge or another team.

### 4.2 Arduino environment

```bash
# Arduino IDE version, or arduino-cli setup
# Board package needed for UNO R4 Minima
# Libraries to install (motor shield lib, servo lib, encoder lib)
```

Specify exact library names + versions (Library Manager names) so version mismatches don't
break the build.

---

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
   python3 main.py
   ```
3. Describe expected console output / startup behavior (e.g. "waits for start button
   press", "camera preview should appear if DEBUG=True").

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
