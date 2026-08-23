# Software Design Architecture

#### Content

- [Introduction](#introduction)
- [Code Structure](#code-structure)
  - [Files and What They Do](#files-and-what-they-do)
  - [How the Pi Talks to the Arduino](#how-the-pi-talks-to-the-arduino)
  - [The Main Loop](#the-main-loop)
  - [Running the Code Yourself](#running-the-code-yourself)
- [Knowing Where We Are](#knowing-where-we-are)
- [Following the Track](#following-the-track)
- [Obstacle Strategy](#obstacle-strategy)
- [Parking](#parking)
- [Sensor Fusion](#sensor-fusion)
- [Testing and Tuning Log](#testing-and-tuning-log)
- [Code Documentation](#code-documentation)
- [What We Would Do Next](#what-we-would-do-next)

---

## Introduction

Most teams solve this competition by looking at the track: find the walls in the camera image,
work out which way they lean, and steer away from them. We started that way too. The problem is
that a camera-only robot only knows what is in front of it **right now**, so it is always reacting
late, and a shadow or a bright reflection can make it forget where it is entirely.

<img width="500" alt="robot_sim" src="https://github.com/user-attachments/assets/af5d16d1-98a7-4732-ac49-425d32fd90b3" />

So we changed the whole idea. Instead of asking *"what does the track look like from here?"* our
robot asks **"where am I on the mat?"** The field is a known shape — a 3 m x 3 m box with a 1 m
block in the middle — so if we can figure out our own position, we can drive a path we planned in
advance instead of improvising one from the camera every frame.

That turns the software into three fairly simple jobs:

1. **🗺️ Where am I?** — the LiDAR and the IMU together give us an (x, y, heading) on the mat.
2. **📍 Where should I be?** — a smooth loop (we call it the *racing line*) drawn between the wall and
   the centre block.
3. **❓ How do I get from 1 to 2?** — a steering controller called *pure pursuit* that aims at a point
   a little way ahead on that loop.

The camera only has one job: find the red and green pillars and say what colour they are. When it
sees one, we do not switch to a separate "avoid mode" — we just **bend the racing line sideways**
around that pillar, and the same steering controller follows the bent line without knowing anything
special happened.

---

## Code Structure

### Files and What They Do

**`src/Arduino/Main.ino`** — one file, about 160 lines. It reads the encoder, drives the motor,
moves the steering servo, watches the start button, and listens for commands on the serial port.
It has no idea the competition exists.

**`src/Raspberrypi/`** — everything else. We split it into four folders so that "how the hardware
works" and "what the robot should do" never end up in the same file.

**`main.py`** — Entry point. Picks which round to run and passes the command-line options along.

To start each round, we can run:
```py
uv run main.py qualification # or
uv run main.py final
```

---

### Raspberrypi Program

| Folder / file | What lives there |
|---|---|
| `tasks/` | The rounds themselves — the actual competition behaviour. |
| `tasks/base_task.py` | The skeleton every round shares: wait for start → setup → loop → finish. |
| `tasks/path_task.py` | The driving. Lap counting, steering, speed, safety. Both rounds inherit this. |
| `tasks/qualification/` | Open Challenge. It is literally just `path_task` with a config file. |
| `tasks/final/` | Obstacle Challenge. Adds the pillar dodging (about 250 lines on top). |
| `tasks/*/config.toml` | Every tunable number — speeds, clearances, lookahead. No editing Python on the field. |
| `classes/` | One file per subsystem. Each one wraps a piece of hardware or one idea. |
| `utils/` | Small shared helpers — angle maths, colour ranges, image cropping. |
| `test_*.py` | Our own test and calibration tools. Several of them run with no robot attached. |

The important files inside `classes/`:

| File | Job |
|---|---|
| `task_context.py` | Starts every subsystem in the right order and shuts them all down safely. Tasks never build hardware themselves. |
| `motor_manager.py` | The serial link to the Arduino. |
| `lidar_manager.py` | Talks to the RPLiDAR C1 and keeps a 360-slot array of distances. |
| `compass_manager.py` | Reads the BNO055 heading. |
| `navigation_manager.py` | Works out where the robot is on the mat (particle filter). |
| `field_map.py` | The mat as geometry — walls, centre block, legal start cells. |
| `racing_line.py` | The loop we want to drive, and the maths to ask "how far along am I?" |
| `pure_pursuit.py` | Turns "where I am" + "where I want to be" into a steering angle. |
| `object_solver.py` | Finds red/green pillars in a camera frame and works out how far away they are. |
| `block_map.py` | Remembers those pillars in field coordinates after the camera looks away. |
| `steering_calibrator.py` | Measures the real steering geometry from a normal lap. |
| `debug_view.py` | The live top-down window we watch while testing. |

### How the Pi Talks to the Arduino

<img width="1040" height="540" alt="image" src="https://github.com/user-attachments/assets/b98b1351-9356-4a73-996c-f44eea7c8f42" />

Yes, they are separate programs. They talk over the USB serial cable at **115200 baud**, and the
protocol is deliberately tiny — one line of plain text per command:

```
<steering angle>,<speed>,<distance>\n
```

* **steering angle** — degrees, negative is left, positive is right
* **speed** — motor power, sign gives direction
* **distance** — degrees of motor-shaft rotation to travel. `0` means "just keep going", which is
  what the racing loop always uses.

The Arduino replies with a single short line so the Pi knows the message landed: `OK` for a normal
command, `t` when a fixed-distance move has finished, or `ERR` if the line was malformed. It also
prints `READY` when it boots and `Start` when the physical button is pressed.

---

### The Main Loop

There is one main loop and it lives in `tasks/base_task.py`. Every round has the same shape, which
is why the two competition rounds share almost all of their code:

```python
def run(self):
    self.context.wait_for_start()              # physical button, via the Arduino
    self.context.compass.set_initial_heading()
    self.start_time = time.monotonic()

    try:
        self.setup()                           # build the path, localize, start rolling
        while not self.is_finished():
            if self.timed_out:                 # 180 s - the WRO limit
                print(f"Time limit ({self.max_runtime_s}s) reached - stopping.")
                break
            self.tick += 1
            self.step()                        # ONE control tick
            if self.status_every and self.tick % self.status_every == 0:
                print(self.status())
            time.sleep(LOOP_DELAY_S)           # 0.02 s -> about 50 Hz
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        self.finish()                          # always stops the motor, whatever happened
```

That `finally` block matters more than it looks. Whether the round ends normally, we press Ctrl-C,
or the code throws an exception, the motor gets stopped and the steering gets centred. The robot
never keeps driving because Python crashed.

Inside one tick (`tasks/path_task.py`), in order:

```python
def step(self):
    # 1. tell the localizer how far we moved since last tick
    distance = self._travelled(dt)
    context.nav.report_motion(distance, self._turned(distance))

    # 2. ask it where we are
    pose = context.nav.get_pose()

    # 3. where are we on the lap, and how far off the line?
    self._track_progress(pose)
    self._update_lost_state(pose, now)

    # 4. pick a point ahead on the path (bent sideways if a pillar says so)
    self.target = self.target_point(pose)

    # 5. steering, then limit how fast the servo is allowed to swing
    wanted = self.pursuit.steering(self._lead_pose(pose), self.target)
    self.steer_command = self._limit_steer_rate(wanted, dt)

    # 6. speed: slowest of corner speed, lost-pose speed, and LiDAR clearance
    self.speed = self._choose_speed(pose)

    # 7. one serial message for both
    context.motor.drive(self.steer_command, self.speed)
```

The obstacle round adds exactly one thing to this — a camera frame every second tick:

```python
def step(self):
    # The camera pipeline costs far more than a control tick, so detection
    # runs on its own slower cadence. The block map is what the steering
    # reads, and that persists between frames.
    if self.tick % CAMERA_EVERY_N_TICKS == 0:
        self._update_detections()
    super().step()
```

---

### Running the Code Yourself

**Raspberry Pi 5**, Python 3.11 or newer. We use [`uv`](https://docs.astral.sh/uv/) to manage
packages.

```bash
cd src/Raspberrypi
uv sync                                   # install everything

uv run python main.py qualification       # Open Challenge
uv run python main.py final --debug       # Obstacle Challenge, with the debug window
uv run python main.py qualification --dry-run   # print the plan, touch no hardware
```

For the Arduino: open `src/Arduino/Main.ino` in the Arduino IDE, board **Arduino UNO R4 Minima**,
and upload. The two libraries it needs (`PID_v2` and `Servo`) are bundled in
`src/Arduino/libaries/` so nobody has to hunt for the right versions.

<img width="700" alt="image" src="https://github.com/user-attachments/assets/8ed4a1a7-0e8a-4b86-bc44-4d5995eef306" />

---

## Knowing Where We Are

The mat is a known shape, so for any position there is exactly one pattern of distances the LiDAR
*should* see. We do the reverse: take the pattern it actually sees and ask which position explains
it.

The technique is a **particle filter**. We scatter 500 guesses across the mat; every update each
guess moves forward by however far we drove and is scored against the real scan; good guesses get
copied, bad ones thrown away. Within about a second the cloud collapses onto the real position.

```python
def _resolve_heading(self, scan):
    """The lidar's precise angle snapped into the quadrant the IMU says we are in."""
    coarse = self.field_heading()                  # IMU
    fine, quality = self.scan_orientation(scan)    # walls, mod 90 degrees
    if fine is None:
        return coarse
    delta = (angle_difference(fine, coarse) + 45.0) % 90.0 - 45.0
    return normalize_angle(coarse + delta)
```

The output is a `Pose` — x, y in mm from the middle of the mat, heading in degrees, and a
**confidence** from 0 to 1. Everything downstream checks that confidence before trusting it; `0.35`
is our "good enough to steer on" threshold.

| Point Cloud scattered             |  Point Cloud converged |
:-------------------------:|:-------------------------:
<img width="760" height="760" alt="image" src="https://github.com/user-attachments/assets/f5e9c17d-f9f0-44cf-8a63-f246ad602407" /> | <img width="760" height="760" alt="image" src="https://github.com/user-attachments/assets/9c6c275c-b07b-4db6-acb4-3db6f86927ec" />

---

## Following the Track

We generate the whole lap as geometry before moving — a rounded square between
the outer wall and the centre block — then follow it. The path is stored both as maths (so we can
ask "how sharp is the bend 400 mm ahead?") and as points every 20 mm (so "how far along am I?" is one
array operation).

We also do not look for the blue and orange lines. The robot can be put down anywhere facing any
direction; at setup it picks whichever way round the loop needs the smaller turn, which caps the
initial heading error at 90°.

**How much do we steer?** **Pure pursuit**, not PID. It picks a target point a set distance ahead
(the *lookahead*), draws the circle through the rear axle and that point, and steers that arc:

<img width="579" height="189" alt="image" src="https://github.com/user-attachments/assets/97c75c68-0340-4ea8-b675-555a94420d5b" />

```python
curvature  = 2.0 * math.sin(math.radians(alpha)) / distance   # alpha = angle to the target
road_wheel = math.degrees(math.atan(self.wheelbase_mm * curvature))
command    = road_wheel / self.max_road_wheel_deg * self.max_steer_command
```

The steering has no gains to fight each other; it has essentially one knob:

* **short lookahead** → tight tracking, oscillates when fast
* **long lookahead** → smooth and stable, cuts corners

The lookahead distance scales with speed.

<img width="1000" height="720" alt="image" src="https://github.com/user-attachments/assets/3d993cf3-0534-49f1-8d91-8b340fb0c89d" />

---

## Obstacle Strategy

The rule is simple — **red passes on its right, green passes on its left** — and so is our code.

### Finding the pillars

Frames go to HSV rather than RGB, because hue survives a change in brightness and RGB does not. Our
colour ranges are written as an anchor colour you can eyedropper out of a real frame plus how far the
hue may drift:

```python
COLOR_SPECS = {
    #                anchor RGB     hue_tol  saturation   value
    Color.GREEN:  ((70, 120, 60),        9,   (90, 230),  (20, 250)),
    Color.RED:    ((140, 30, 30),        6,  (125, 210),  (55, 200)),
}
```

These values are obtained with `test_color_picker.py`.

### How far away is it?

Not from the LiDAR — it sees a pillar as a few centimetres of wall and cannot tell its colour.
Instead we use the fact that the signs are a known **10 cm tall**: measure the pixel height, convert
to an angle, and trigonometry gives the distance.

```python
BOX_HEIGHT_CM = 10.0
DEG_PER_PX = CAPTURED_HORIZONTAL_FOV_DEG / CAPTURED_FRAME_WIDTH_PX   # 80 deg / 640 px

angular_height_deg = max(rect[1]) * DEG_PER_PX
distance_cm = (BOX_HEIGHT_CM / 2.0) / math.tan(math.radians(angular_height_deg / 2.0))
bearing_deg = self._horizontal_pixel_to_angle(center_x)
```

### Deciding left or right

The whole decision:

```python
# `lateral` is positive to the right of travel. Passing a block on its LEFT
# means ending up to the left of it, i.e. a negative offset from its position.
SIDE_FOR_COLOR = {Color.GREEN: -1.0, Color.RED: +1.0}
```

And the amount — note that we aim relative to **where the pillar actually is**, not by a fixed
amount from the centre line, so a pillar already near the wall needs only a small correction:

```python
def _required_offset_mm(self, color):
    return (self.setting("blocks.clearance_mm")          # gap we want from our body
          + self.setting("blocks.robot_half_width_mm")   # our centreline isn't our edge
          + self.setting(extra_key)                      # optional per-colour padding
          + BLOCK_SIZE_MM / 2.0)                         # pillar centre to its face
```

The offset fades in and out on a smoothstep curve rather than a straight ramp, so the path curves
into the dodge instead of kinking at the joints, and it is clamped so a dodge can never push us into
a wall to give a pillar more room. Current numbers: 140 mm clearance, 160 mm robot half-width, 80 mm
extra on red only, easing across from 600 mm before and holding until 450 mm past.

<img width="1345" height="576" alt="image" src="https://github.com/user-attachments/assets/a84a6d0e-dd47-4796-9998-58df81fc10c8" />

---

## Parking

The parking steps are fixed. We landed on the 4 steps parking manoeuvre:
1. Position the robot so that the back of the robot align with the parking walls.
2. Turn and move backwards into the parking space
3. Move backwards straight
4. Turn and move backwards so the robot is parallel with the outer walls

---

## Sensor Fusion

| Sensor | Trusted for | *Not* used for |
|---|---|---|
| **LiDAR (RPLiDAR C1)** | Position, precise heading, forward clearance | Colour, anything about the pillars |
| **IMU (BNO055)** | Which quarter of the mat we face | Precise heading (it drifts) |
| **Camera** | Pillar colour, bearing, distance from apparent height | Position, walls, steering |
| **Motor command** | How far we moved between scans | Absolute position |

The genuinely *fused* parts are two: the heading (LiDAR angle + IMU quadrant, as above — neither
sensor could do it alone), and the block map (a camera detection is meaningless by itself; combined
with the current pose it becomes a fixed point on the mat).

**When they disagree:**

* **Geometry** → the LiDAR wins. If the pose is unreliable we do not place the detection at all, we
  throw the whole frame away. A pillar placed with a bad pose is worse than no pillar, because it
  lands somewhere real and looks just as trustworthy as a good one.
* **Heading** → the disagreement is folded into ±45°, so the IMU only ever contributes the quadrant.
* **Camera says clear, LiDAR says blocked** → the LiDAR wins, always.
* **A detection that lands inside a wall** → dropped; it must be a reflection.

<img width="1080" height="700" alt="image" src="https://github.com/user-attachments/assets/7cc79c9f-e000-4f05-85a6-48cf61c505c4" />

---

## Testing and Tuning Log

### Testing without the robot

The most useful thing we built this year is not on the robot. Three test scripts run the **real**
code — real task, real particle filter, real config — against a simulated robot, so a change takes
seconds instead of another mat setup:

```bash
uv run python test_navigation.py            # watch localization converge, no hardware
uv run python test_driving.py --trials 24   # 24 runs from random placements, pass/fail summary
uv run python test_steering.py --sweep speed.corner 40,50,60,70   # trade-off table
```

`test_driving.py` reports the number that actually matters — the closest we came to a wall or the
centre block — and deliberately simulates servo lag, because a lag-free simulator will never show you
the weave that lag causes. It does not replace mat testing (it cannot show glare, dust, or a sagging
battery); it stops us wasting mat time on settings that were never going to work.

### Measuring instead of guessing

Two numbers cannot be derived from first principles and both cause a hunting robot, so we wrote tools:

* **`test_steering.py --calibrate`** drives arcs and works out the true full-lock angle from the path
  traced. Result: the robot turns about **21% more** than the 40° we had assumed.
* **`SteeringCalibrator`** does the same during any normal lap, so every run is also a measurement. It
  integrates over the whole run rather than comparing instant by instant — the wheels lag the command
  by ~0.1 s, so pairing them directly compares a cause with an effect that has not arrived (on the
  simulator that reported 30° for a robot whose real lock was 45°).
* **`test_steering.py --lag`** measures the steering response: about **0.35 s** to 63%.

* ### Watching what the robot thinks

* **Status line** every 25 ticks — time, lap, position, off-line distance, steering, speed, confidence:

  ```
  [ 12.4s] lap 0.87/3  (+102.4,-140.1)cm  off-line +1.2cm  steer=-8.3 (wheel -4.5deg)  speed=70  conf=0.78
  ```

* **`--debug`** — live top-down window: field, particle cloud, LiDAR points, racing line, bent line,
  and the point we are chasing. ESC ends the run. **`--ascii`** prints the same as text for plain SSH.
* The object solver has its own two windows: the camera frame with each pillar boxed and labelled, and
  a top-down radar view.
* **`test_color_picker.py`** — click a pixel, get its RGB and HSV, ready to paste into the colour spec.
* The camera manager can record processed video to `videos/` for reviewing a run afterwards (currently
  commented out in the final round to save processing time).

  <img width="1120" height="560" alt="image" src="https://github.com/user-attachments/assets/64f29e14-323e-4c5f-b2cd-3190da0c62e1" />

---

## Code Documentation

Every module starts with a docstring explaining *why* it exists, and every non-obvious constant has a
comment explaining how the number was arrived at — a rule we made after losing an afternoon to a magic
number nobody could remember setting. The style is to explain the reasoning, so the next person can
tell whether it is safe to change:

```python
# Real-world HEIGHT of the boxes. The WRO traffic signs are 5cm x 5cm on the
# floor and 10cm tall - this is the 10, not the 5. Distance scales linearly
# with it, so the 5.0 that used to be here reported every pillar at half its
# true range.
BOX_HEIGHT_CM = 10.0
```

The config files carry it further: `tasks/qualification/config.toml` opens with a numbered "if the
robot weaves, change these in this order" guide, so whoever is at the mat can fix a problem without
reading the control loop first.

**Still to clean up before this is public:**

* The legacy files are much less documented — delete them, or move them to `legacy/` with a note that
  they are not the competition code path.
* `utils/enums.py` still defines `RunStates` and `SpeedStates` from the old state machine. Nothing uses
  them, and they make the design look more complicated than it is.
* `field_map.py`'s docstring says 3200 mm while the constant next to it is the correct 3000 mm.
* `qualification/config.toml` sets `speed.base = 255` although the loop clamps the command to 100. It
  works, but the number is misleading.
* With the current final-round numbers, a pillar sitting exactly on the racing line would need a 405 mm
  dodge (red) or 325 mm (green), but only 304 mm of corridor is available — so the startup check prints
  a warning. The check working is good news; the numbers need one more pass.

---

## What We Would Do Next

* **Keep proper run statistics.** We tuned by watching, and the simulator gave us steering numbers, but
  we never kept a disciplined success-rate log on the mat.
* **Use the wheel encoder for odometry.** The Arduino already counts ticks; the Pi currently estimates
  distance from the commanded speed instead of reading them back.
* **Lock the camera exposure.** Auto-exposure shifting mid-run is the biggest remaining threat to colour
  detection, and it is fixable in the camera config rather than in the thresholds.
