"""
The driving half of both rounds: follow a loop with pure pursuit, count laps,
and do not hit anything.

What the two rounds share is everything except one method. Qualification
drives the racing line as generated; the final round shifts it sideways to
pass each pillar on the required side. That is the ONLY difference in the
driving, so it is the only thing `target_lateral_mm()` exists for - see
tasks/final/task.py.

The loop per tick:

    pose      ask NavigationManager where we are
    progress  project the pose onto the path -> how far round, how far off
    laps      accumulate progress, wrapped, to count laps
    target    a point one lookahead ahead on the path, shifted sideways by
              whatever target_lateral_mm() asks for
    steer     pure pursuit onto that point
    speed     back off for corners, for a bad pose, and for anything the lidar
              sees close ahead

Placement is unconstrained: the robot may be set down anywhere on the mat
facing any direction. Setup waits for the localizer to converge, then picks
whichever way round the loop needs the smaller turn, so the round starts
clockwise or counter-clockwise depending on how the robot was placed.
"""
import math
import time
from dataclasses import replace

from classes.debug_view import DebugView
from classes.pure_pursuit import PurePursuit
from classes.steering_calibrator import SteeringCalibrator
from classes.racing_line import RacingLine
from tasks.base_task import Task
from utils.angle_utils import clamp

# ============================================================================
# Defaults. Anything here can be overridden per round in its config.toml -
# these are the values used when a key is missing from the file.
# ============================================================================
DEFAULTS = {
    "laps.goal": 3,

    "speed.base": 70,              # straights
    "speed.corner": 60,            # sharpest part of a corner
    "speed.lost": 50,              # pose not trustworthy
    "speed.minimum": 40,           # never crawl slower than this while driving

    "path.wall_margin_mm": None,   # None = centre the loop in the corridor
    "path.corner_radius_mm": None,  # None = biggest that clears the block
    "path.resolution_mm": 20.0,

    "pursuit.wheelbase_mm": 165.0,
    "pursuit.lookahead_base_mm": 260.0,
    "pursuit.lookahead_per_speed_mm": 3.0,
    "pursuit.lookahead_min_mm": 250.0,
    "pursuit.lookahead_max_mm": 700.0,
    "pursuit.max_road_wheel_deg": 30.0,
    "pursuit.max_steer_command": 80.0,
    "pursuit.rear_axle_offset_mm": 0.0,

    # How fast the steering command is allowed to move, in command units per
    # second. None scales it to the rack: 3 x max_steer_command, i.e. lock to
    # centre in a third of a second.
    #
    # A RATE LIMIT, not a filter. A low-pass would lag every steering change
    # including the slow ones that make up normal cornering, costing tracking
    # everywhere to fix a problem that only happens on sharp steps. This does
    # nothing at all until the command tries to move faster than the limit,
    # which is exactly when the servo snaps and shakes the camera.
    "steer.max_rate_units_s": None,
    # Cap on the lookahead inside a bend, as a fraction of the bend's radius.
    # Lower it if the robot still cuts corners; raise it if it weaves.
    "pursuit.corner_lookahead_fraction": 0.8,

    # How near a pillar has to be before it starts pulling the lookahead in,
    # and how short it may pull it. 0 for the window disables the whole thing
    # and the lookahead is whatever speed and curvature asked for. See
    # FinalTask.block_lookahead_cap_mm.
    "pursuit.block_lookahead_window_mm": 700.0,
    "pursuit.block_lookahead_near_mm": 120.0,
    "pursuit.block_lookahead_margin_mm": 0.0,
    # Seconds of actuator lag to steer ahead of. The wheels do not reach the
    # angle you asked for instantly, so by the time they get there the robot
    # has moved - steering off the pose it has THEN, rather than the pose it
    # has now, cancels that delay. Set it to the steering system's response
    # time; test_steering.py measures it.
    "pursuit.lag_compensation_s": 0.0,
    # Time constant of the steering servo, from test_steering.py --lag. 0 says
    # the wheels reach the commanded angle instantly, which is what everything
    # here assumed before - see PurePursuit.advance_servo.
    "pursuit.servo_lag_s": 0.0,

    # Speed cap while the planner reports a pass it could not give the
    # clearance asked for. Such a plan is still the best available and is
    # still driven - just slowly, because every margin that makes a tight
    # pass survivable is bought with speed.
    "speed.compromised": 45,

    # ---- Goal-based planning (final round) - classes/goal_planner.py -------
    # The gap the planner ASKS for between the robot's body and a pillar.
    # Unlike the old blocks.clearance_mm this is a preference, not a promise:
    # where the corridor cannot give it, the plan takes less AND SAYS SO.
    "goals.min_clearance_mm": 45.0,      # below this a pass is "compromised"
    "goals.horizon_mm": 2200.0,          # how much lap one plan covers
    "goals.route_spacing_mm": 550.0,     # goal spacing on empty stretches
    "goals.approach_mm": 450.0,          # route goals stop this short of a pass
    "goals.exit_mm": 350.0,              # plan holds its offset this far past
    "goals.align_mm": 200.0,             # straight run-in before a pass, 0 = none
    "goals.max_gates": 3,                # pillars one plan reaches through
    "goals.replan_interval_s": 0.1,      # cadence when nothing has changed
    "goals.replan_cross_track_mm": 120.0,  # drift that forces a new plan
    # The robot's own extent forward of and behind THE POINT THE POSE
    # DESCRIBES, which with pursuit.rear_axle_offset_mm at 0 is the rear axle -
    # so robot_front_mm is nearly the whole length of the car, not half of it.
    # Getting that wrong is not a rounding error: the front overhang is the
    # part that swings widest through a corner and the part that reaches a
    # pillar first, so understating it plans passes that clear on paper and
    # clip on the mat.
    #
    # Used to sweep the body along a candidate path as three discs rather than
    # one - a single disc at the centre understates the swing by that whole
    # overhang. MEASURE BOTH: rear axle to front bumper, rear axle to rear
    # bumper.
    "goals.robot_front_mm": 200.0,
    "goals.robot_rear_mm": 40.0,

    # Leaving the bay the round starts in is [unpark], driven off the lidar by
    # UnparkController - see FinalTask._setup_manoeuvres. There is nothing to
    # default here, because the three numbers that make up the manoeuvre have
    # to be measured on the mat and it refuses to drive without them.

    "safety.min_pose_confidence": 0.35,
    "safety.lost_timeout_s": 4.0,
    "safety.front_slow_mm": 450.0,
    "safety.front_stop_mm": 220.0,
    "safety.front_sector_deg": 25.0,
    # Stopping in front of something is only half a backstop - a stopped robot
    # cannot drive away from what stopped it. These back it out instead.
    "safety.stuck_timeout_s": 1.0,
    "safety.reverse_time_s": 1.2,
    "safety.reverse_speed": 50,

    "startup.localize_timeout_s": 8.0,
    # Which way the robot is PLACED facing, degrees clockwise from +Y (0=+Y,
    # 90=+X, 180=-Y, 270=-X). Unset leaves the compass unanchored, and an
    # unanchored compass picks the robot's quadrant off its own boot-time
    # zero - the same wrong quadrant every run, on a field the lidar cannot
    # tell the quadrants of. One number, and it is one you know: it is how you
    # put the robot down.
    "startup.start_heading_deg": None,
    # The rules only allow the robot to be set down in the four non-corner
    # cells. Seeding the filter's search there instead of over the whole ring
    # locks on faster and cannot pick a pose that was never legal.
    "startup.assume_start_zone": True,
    # The scale that turns a speed COMMAND into millimetres per second, as
    #     mm/s = speed / 100 * mm_per_s_at_full
    #
    # Read that "100" carefully, because the name does not describe it. The
    # speed field of the serial message is handed to the Arduino unchanged and
    # used as a raw PWM duty, which analogWrite constrains to 0-255 (see
    # src/Arduino/Main.ino, motor_dc) - so a speed command is NOT a percentage,
    # and the two rounds bear that out: qualification asks for 255 and the
    # final round for 60. The divisor here is therefore an arbitrary constant
    # rather than a top-speed reading, and mm_per_s_at_full is whatever makes
    # the product come out right. That is fine - the model only has to be
    # linear and self-consistent - but it means this number is meaningless
    # unless it was MEASURED against this same formula.
    #
    # 700 was right through the original gearing. 390 is MEASURED with a tape
    # after the 16:28 reduction was fitted (700mm in 3.0s at command 60),
    # within 3% of the 400 the ratio predicts. Re-measure it the same way if
    # the drivetrain changes again:
    #     python test_steering.py --tape 3 --drive-speed 60
    # Not with --calibrate: that derives distance from the pose track, and
    # this number is what makes the pose track trustworthy in the first place.
    #
    # It is worth measuring, because this is the particle filter's entire
    # motion model between lidar scans. Set it too high and the filter is told
    # the robot has driven further than it has, every tick, and the pose runs
    # ahead of the truth until the next scan drags it back. Pillars mapped
    # during that walk land in the wrong place, and the round plans around
    # pillars that are not where it thinks they are.
    "startup.mm_per_s_at_full": 390.0,

    # ---- Final round: the pillars. See classes/goal_planner.py -----------
    # These live here so that setting() has a default for them whatever
    # config file is loaded.
    #
    # This section used to be twice this size. The round drove a lateral
    # OFFSET PROFILE - a pillar bent the racing line sideways and the bend was
    # shaped by padding_mm, approach_mm, past_mm, two Bezier handles, a
    # max_lean_deg and a catchup_mm - and none of those describe anything any
    # more. The round plans goal poses and joins them with curvature-bounded
    # paths, so the shape of a pass is a consequence of the geometry rather
    # than something to be dialled in. They are deleted rather than left
    # inert: a tuning knob that does nothing is worse than no knob, because
    # somebody will spend a practice session turning it.
    #
    # The two per-colour clearance pads went the same way. They existed to
    # live with a detection that mis-ranged one colour, by making that
    # colour's pass wider - which under the new planner is not a fix but a
    # request the corridor will refuse and report. Fix the detection.
    "blocks.clearance_mm": 150.0,        # gap ASKED for, robot body to pillar
    "blocks.robot_half_width_mm": 100.0,  # centreline to the widest point
    # How far off a pillar may be mapped, overriding block_map's own
    # MAX_MAPPING_RANGE_MM. None leaves that alone.
    "blocks.map_range_mm": None,
    # How long a pillar keeps its goals after its track is lost. A pass must
    # not be cancelled by a dropped frame - that is a collision - so it is
    # held, and this bounds how long a wrong detection can hold it.
    "blocks.memory_s": 1.5,
    # Real gap from the robot's BODY to the outer wall or the centre block.
    "blocks.wall_clearance_mm": 80.0,
    # How much of the robot's own turning circle to keep in hand when drawing
    # a plan. A path at exactly full lock is one the follower can only just
    # hold, with nothing left to correct tracking error with.
    "blocks.turn_radius_margin": 1.15,

    # Final round only - the parking bay. See classes/parking.py for the
    # geometry these describe and why the manoeuvre is shaped the way it is.
    "parking.enabled": True,
    # Real gap from the robot's BODY to the tips of the bay walls while
    # driving PAST them. They stick 200mm out from the outer wall, so the
    # ordinary wall_clearance_mm - which assumes a flat wall - would happily
    # steer a dodge straight into one.
    "parking.wall_margin_mm": 40.0,
    # A bay is only believed after this many scans agree about it.
    # Bay DETECTION. The park itself no longer needs it - the sequence finds
    # the bay wall with the side lidar as it drives past. This is still what
    # keeps a pillar dodge from clipping a bay wall, and what puts the walls
    # on the map the filter matches against.
    "parking.detect_min_depth_mm": 120.0,
    "parking.detect_min_gap_mm": 250.0,
    "parking.detect_max_gap_mm": 400.0,
    "parking.detect_min_scans": 3,
    # Scans one blade has to be seen in before a bay is placed from it ALONE.
    # Half the bay is often all there is to see: the far blade is edge-on from
    # most of the approach and only 10mm thick, and the near one shadows the
    # floor behind it. Waiting for two clusters to agree can mean never
    # parking. One is enough because the WIDTH is a rule, not a measurement -
    # but it is held to a higher bar than a pair, because a pillar standing
    # near the wall looks exactly like one blade and cannot look like two the
    # right distance apart. Raise it if a pillar ever gets parked at.
    "parking.detect_single_scans": 6,
    # Reversing speed for the manoeuvre, and the slower creep used for the
    # last stretch up to the staging point. The approach is deliberately slow:
    # at racing speed one control tick is 8mm, and every millimetre of
    # overshoot there lands on the tightest clearance of the whole manoeuvre.
    # How much further the robot will keep lapping, looking for the bay,
    # once the laps are done. The fallback is deliberately "keep driving" -
    # laps already scored are worth more than a blind park - but it cannot be
    # unbounded, or a round with no bay in front of it never ends at all.
    "parking.extra_laps": 1.5,
    # How many times a FAILED park may be retried. An attempt that aborts is
    # thrown away rather than ending the round - the laps are already scored
    # and the clock is still running, so driving on beats stopping dead
    # wherever the guard tripped. Each retry costs most of a lap, because the
    # approach needs a clean run at the bay rather than a restart from where
    # the last attempt gave up.
    "parking.retries": 2,
    # How much of the FINAL lap the manoeuvre may start before the lap counter
    # says the laps are done - i.e. how far short of the starting point to
    # park. Zero means "finish the lap first", which puts the approach right
    # where the robot set off from.
    #
    # This is a distance off the END of the last lap only; the first laps are
    # always driven in full. It never fires while a mapped pillar still lies
    # between the robot and the bay, so "park early" means "park once the last
    # block is behind us", not "cut the corner off the last pillar".
    #
    # Costs whatever the judges count as a lap: park 1m short and the last lap
    # is 1m short. Worth it when the alternative is arriving at the bay with
    # no room left to line up.
    # Parking starts THIS far before the lap would have finished, so the
    # follow has a run at the wall before the bay turns up in it - it needs
    # about 1.4m to settle onto the wall from a bad handover.
    "parking.start_early_mm": 1500.0,

    # ---- the park itself: a script the lidar starts ----------------------
    # See ParkingSequence in classes/parking.py. Drive alongside the outer
    # wall, wait for the range to step in - that is the bay wall, which stands
    # 200mm proud of the outer wall and is the only thing on the field that
    # does it - then run a fixed sequence of angles and distances. Nothing
    # here reads the pose.
    #
    # How far off the outer wall to run while looking, how hard to correct,
    # and how much correction is allowed. The follow only has to hold the
    # robot roughly parallel for the length of one wall.
    # How much wall the FOLLOW phase expects to drive along before it meets
    # the bay. Not a limit - the follow runs until it sees the step or times
    # out - but it is what start_early_mm has to buy back, and how far the
    # robot goes round before a failed attempt is retried.
    "parking.follow_mm": 1200.0,
    # WIDE, for two separate reasons. The bay walls stand 200mm proud of the
    # outer wall, and the body is 150mm across - so anything under 275mm here
    # drives the robot's flank straight through them. And a 90deg turn at full
    # lock drops the rear axle 204mm toward the wall, so from 300mm out the
    # nose would finish 74mm THROUGH it; 450mm leaves the nose 76mm short,
    # which the nose-in step then closes on the front lidar.
    "parking.wall_distance_mm": 450.0,
    "parking.wall_gain": 0.08,
    "parking.wall_max_steer": 20.0,
    # Which lidar sector watches the wall, measured from straight ahead. The
    # sign is applied for you - it looks at whichever side the wall is on.
    "parking.side_bearing_deg": 90.0,
    "parking.side_sector_deg": 30.0,
    # HOLDING THE HEADING, not just the distance. A line is fitted through the
    # whole visible arc of wall either side of the perpendicular, and its
    # slope is the yaw relative to that wall. angle_gain is steering command
    # per degree of it: 0 turns the heading term off and leaves a
    # distance-only follow, which holds a number and lets the robot crab.
    # Raise it if it drifts off the wall between corrections, drop it if it
    # weaves. angle_arc_deg is the half-width of the arc - wider is quieter
    # (the fit averages more returns) but reaches further forward, where the
    # bay wall shows up early; it is clamped to the chassis FOV regardless.
    # Anything past angle_max_deg is a corner or a step, not the wall being
    # followed, and is discarded rather than steered on.
    "parking.angle_gain": 0.6,
    "parking.angle_arc_deg": 25.0,
    "parking.angle_min_points": 12,
    "parking.angle_max_deg": 30.0,
    # The range that says "bay wall". Unset means wall_distance_mm - 120,
    # which a 200mm-proud wall clears easily and range noise does not.
    "parking.trigger_below_mm": None,
    #
    # ---- THE PARK: head in, square to the wall ---------------------------
    # Not a parallel park. At 39deg of lock a 210mm body cannot be threaded
    # lengthwise into a 340mm slot on an arc - the best of 630,000 swept
    # combinations still swept 50mm through a bay wall on the way in. Driving
    # in nose-first uses the slot's 340mm against the body's 150mm WIDTH,
    # which leaves 95mm either side, and the turn clears the walls by 18mm.
    #
    # CREEP. The trigger fires off the WIDE sector, which reports its closest
    # return and so catches a bay wall well before the robot is level with it.
    # The NARROW beam is what says "level", and that is the reference the turn
    # is measured from. turn_after_mm rolls on further before turning: the
    # turn lands the robot 204mm past the wall it started from against a bay
    # half-width of 170, so it sits about 30mm past centre with 60mm still to
    # spare - raise this only if it needs to sit deeper along the bay.
    # Pure pursuit is off from the moment parking starts, so the sequence has
    # to watch where it is going itself. Closer than this dead ahead means a
    # corner, and the follow gives up so the retry can try the next lap. It is
    # NARROW and it has to persist: a wide sector reads the wall it is
    # following as an obstacle the moment the robot is angled at all.
    "parking.front_stop_mm": 450.0,
    "parking.front_sector_deg": 10.0,
    "parking.front_hold_s": 0.4,
    # PURE PURSUIT IS OFF ONCE PARKING STARTS, and the pillar dodging with it.
    # Nothing else is watching where the approach is going, so it watches for
    # itself: 30deg covers +-80mm at 300mm ahead, which is the body's own
    # width, while the wall being followed is 1700mm down that ray and a bay
    # wall 970mm, so neither trips it. Anything closer is something the body
    # would hit, and the attempt is handed back so the lap can drive round it.
    "parking.body_stop_mm": 300.0,
    "parking.body_sector_deg": 30.0,
    # KEEPING STATION WHILE THE WALL SIDE IS BLIND. Alongside the bay the side
    # lidar reads a bay wall rather than the wall, so an ordinary follow would
    # see 250mm, decide it was too close and steer AWAY from the thing it is
    # trying to park in. The centre block on the far side of the road is used
    # instead: the road's width is measured on the way in, so the distance to
    # the block that matches the right distance from the wall is known.
    # inner_slack_mm is how far off that it may read before being disbelieved
    # - near a corner the block is not there at all, and a beam that sees
    # across the field instead would pull the robot into the wall.
    "parking.inner_sector_deg": 30.0,
    "parking.inner_slack_mm": 250.0,
    "parking.mouth_sector_deg": 6.0,       # the NARROW beam's width
    "parking.blade_below_mm": None,        # unset = the same as the trigger
    # THE LIDAR IS NOT AT THE AXLE - it sits this far forward of it, which
    # moves both ends of the sequence: the beam goes level with a bay wall
    # while the axle is still that far short of it, and it reads the outer
    # wall from that much closer than the nose is.
    "parking.lidar_ahead_mm": 130.0,
    # Unset means "land in the middle of the bay": half the bay, plus how far
    # the lidar leads the axle, minus the radius the turn carries the axle
    # through - 170 + 130 - 204, about 96mm. At zero it lands 96mm off centre,
    # which is enough to graze the far wall.
    "parking.turn_after_mm": None,
    # Drive past BOTH bay walls to measure the bay, then reverse to its
    # middle, instead of turning a fixed distance after the first wall. Costs
    # a bay-length of travel and the reverse that undoes it, and buys a turn
    # aimed at the bay actually in front of the robot rather than a nominal
    # 340mm one. Falls back to turn_after_mm if the second wall is never seen.
    "parking.measure_bay": True,
    # A bay wall is 10mm thick and the beam's footprint is wider, so at the
    # crossing it flickers clear-then-blocked within 20mm - which reads as a
    # 20mm bay. The mouth only counts once the beam has been clear for this
    # far, and the far wall only counts beyond bay_min_mm from the near one.
    "parking.mouth_clear_mm": 60.0,
    "parking.bay_min_mm": 170.0,
    # SQUARE UP BEFORE BACKING IN. Everything after the reverse is open loop,
    # so the pose it starts from is the one the whole manoeuvre is measured
    # from - and alongside the bay is the worst place to take it, because the
    # side lidar has been looking at a bay wall rather than the wall. Past the
    # far wall the beam is clean again, so it is given a little road to settle
    # on. What it spends is added to the reverse, so it costs position
    # nothing. settle_max_mm caps it; the tolerances say "close enough".
    "parking.settle_max_mm": 600.0,
    "parking.settle_tolerance_mm": 40.0,
    "parking.settle_angle_deg": 3.0,
    "parking.settle_relax": 2.0,
    "parking.creep_max_mm": 700.0,
    # TURN IN, counted on the COMPASS. Unset steer = full lock, the tightest
    # arc, which is what the 204mm drop above assumes.
    "parking.turn_in_deg": 90.0,
    "parking.turn_in_steer": None,
    # The least distance off the wall the turn may START from. Unset means
    # radius + nose + 30, about 404mm: the turn carries the axle a whole
    # radius toward the wall before the nose comes round, so from closer than
    # that the body is through a bay wall before it is halfway round - at
    # 360mm it clips by 40mm, at 320mm the nose ends 54mm INSIDE the outer
    # wall. Under this the attempt is given up rather than driven into it.
    "parking.turn_in_min_mm": None,
    # Steering command per degree of heading error on the legs that are meant
    # to be straight - the creep and the drive into the bay. Steering zero is
    # not the same as going straight: those legs run on from a servo that has
    # just been at full lock, and two degrees held for half a metre is the
    # difference between arriving square in the bay and across it. The
    # reference is the wall's own heading, learned by the follow while the
    # side lidar could still measure it. 0 turns the correction off.
    "parking.heading_gain": 1.0,
    # How far the NOSE stops off the outer wall. Converted to what the front
    # beam reads at that moment, which is lidar_ahead_mm less. The one place
    # an error in everything upstream is absorbed rather than added to.
    "parking.nose_stop_mm": 20.0,
    # THE CAMERA'S JOB. The lidar sees a step 200mm proud of the wall and
    # cannot tell a bay wall from a pillar standing next to one; the camera
    # can see that it is pink but, through an object of unknown size, not how
    # far away it is. So the lidar picks the moment and the camera only has to
    # agree the thing is pink and roughly on the expected side. Setting
    # camera_confirms false falls back to the lidar on its own.
    "parking.camera_confirms": True,
    "parking.camera_bearing_deg": 60.0,
    # Forward and reverse speeds, and the pause that lets the servo reach a
    # new angle before the wheels move - asking for lock and drive on the same
    # tick spends the first part of the leg going straight.
    "parking.speed": 25,
    "parking.reverse_speed": 25,
    "parking.servo_settle_s": 0.4,
    "parking.timeout_s": 20.0,
    # How many times a failed park may be retried. An attempt that aborts is
    # thrown away rather than ending the round - the laps are already scored
    # and the clock is still running.
    "parking.retries": 2,

    # Final round only - leaving a bay the robot STARTED in. Off by default,
    # because a run that starts on the track and switches this on drives a
    # pointless swerve before its first lap. See UnparkController.
    "unpark.enabled": False,
    # THE TWO NUMBERS THAT ARE THE MANOEUVRE, and both are left unset on
    # purpose - they are what a tape measure on the mat tells you, not what
    # geometry does, because they depend on where in the bay the robot was
    # placed and how much lock the servo really has. With either unset the
    # exit refuses to run and says so.
    #   reverse_mm:    how far to back up, wheels centred, before turning.
    #                  The turn out is an arc, and an arc needs length before
    #                  it has moved the robot sideways at all; this is where
    #                  that length comes from. Zero means no reverse.
    #   steer_command: how hard to turn out, as a MAGNITUDE in MotorManager's
    #                  units. Which way it points is not set here - it is
    #                  whichever side the lidar found open.
    #   forward_mm:    how far to drive on that lock before the path follower
    #                  takes over. Long enough to be clear of the bay walls
    #                  and pointing down the track, short enough not to cross
    #                  it.
    "unpark.reverse_mm": None,
    # Counter-steer for the reverse leg, as a MAGNITUDE - applied toward the
    # side the robot is NOT leaving by. Yaw rate reverses with the direction
    # of travel, so backing up on the opposite lock rotates the robot the same
    # way the forward leg will, and the reverse arrives at the mouth of the
    # bay with the nose already aimed at the way out. Zero is straight back,
    # which is what a bay too tight to give the TAIL room wants.
    "unpark.reverse_steer_command": 0.0,
    "unpark.steer_command": None,
    "unpark.forward_mm": None,
    "unpark.speed": 25,
    # Slower than the way out: what is behind the robot is the wall it was
    # parked against, and the commanded speed is the only thing measuring the
    # distance to it.
    "unpark.reverse_speed": 20,
    # How long the robot stands still reading the lidar before it decides
    # which way is out, and how long it then holds still while the servo
    # actually gets to the lock it has been asked for.
    "unpark.look_s": 0.5,
    "unpark.servo_settle_s": 0.4,
    # The two sectors compared to find the open side: centred on straight
    # left and straight right, this wide. Wide enough to survive a nan or
    # three, narrow enough not to include what is in front of the robot.
    "unpark.side_bearing_deg": 90.0,
    "unpark.side_sector_deg": 45.0,
    # How much further one side has to reach before it counts as the open
    # one. Under this the two are the same wall seen twice, and
    # default_side decides (+1 right, -1 left).
    "unpark.side_margin_mm": 100.0,
    "unpark.in_bay_mm": 250.0,
    "unpark.default_side": 1,
    "unpark.timeout_s": 15.0,
}


class PathDrivingTask(Task):
    """
    Base for any round that drives laps of the racing line.

    Subclasses override `target_lateral_mm()` to bend the line, and otherwise
    inherit the whole control loop. Requires the lidar - without a pose there
    is no path following, so this round cannot run blind.
    """

    name = "path"
    requires_lidar = True

    def __init__(self, context, config=None, **kwargs):
        super().__init__(context, **kwargs)
        self.config = config
        self.laps_goal = int(self.setting("laps.goal"))

        self.path = None
        self.pursuit = None
        self.direction = 1

        self.progress = 0.0          # travel-frame progress this tick
        self.aim_progress = 0.0      # progress of the point being chased
        self.lateral = 0.0           # how far off the line we are, + right
        self.distance_driven = 0.0   # along the path, for lap counting
        self.speed = 0
        self.steer_command = 0.0
        self.target = None

        self._last_tick_at = None
        self._front_mm = None        # forward lidar clearance, refreshed per tick
        self._lost_since = None
        self._stop_reason = None
        self._blocked_since = None
        self._reversing_until = None
        self.calibrator = None
        self._steer_rate_limit = None    # units/s, resolved at setup
        self._debug_view = None

    # ========================================================================
    # CONFIG
    # ========================================================================

    def setting(self, key):
        """A tunable, from this round's config.toml or the shared default."""
        default = DEFAULTS.get(key)
        return default if self.config is None else self.config.get(key, default)

    # ========================================================================
    # SETUP
    # ========================================================================

    def setup(self):
        context = self.context
        context.motor.steer_center()

        self.path = RacingLine(
            field_map=context.nav.map,
            wall_margin_mm=self.setting("path.wall_margin_mm"),
            corner_radius_mm=self.setting("path.corner_radius_mm"),
            resolution_mm=self.setting("path.resolution_mm"))
        self.pursuit = PurePursuit(
            wheelbase_mm=self.setting("pursuit.wheelbase_mm"),
            lookahead_base_mm=self.setting("pursuit.lookahead_base_mm"),
            lookahead_per_speed_mm=self.setting("pursuit.lookahead_per_speed_mm"),
            lookahead_min_mm=self.setting("pursuit.lookahead_min_mm"),
            lookahead_max_mm=self.setting("pursuit.lookahead_max_mm"),
            max_road_wheel_deg=self.setting("pursuit.max_road_wheel_deg"),
            max_steer_command=self.setting("pursuit.max_steer_command"),
            rear_axle_offset_mm=self.setting("pursuit.rear_axle_offset_mm"),
            corner_lookahead_fraction=self.setting("pursuit.corner_lookahead_fraction"),
            servo_lag_s=self.setting("pursuit.servo_lag_s"))

        # Always on: it costs one arithmetic step per tick and turns every
        # lap into a measurement of the one gain that cannot be derived.
        self.calibrator = SteeringCalibrator(
            wheelbase_mm=self.pursuit.wheelbase_mm,
            max_steer_command=self.pursuit.max_steer_command,
            assumed_max_road_wheel_deg=self.pursuit.max_road_wheel_deg)

        rate = self.setting("steer.max_rate_units_s")
        self._steer_rate_limit = (float(rate) if rate
                                  else 3.0 * self.pursuit.max_steer_command)
        self.steer_command = 0.0

        print(self.path)
        print(f"Steering slew limit: {self._steer_rate_limit:.0f} units/s "
              f"({self.pursuit.max_steer_command / self._steer_rate_limit * 1000:.0f}ms "
              f"centre to full lock)")
        self._warn_if_corners_too_tight()
        self._warn_if_lookahead_unusable()

        pose = self._wait_for_localization()
        # Only now: everything the camera maps before the filter converges is
        # rejected on the way in, so there is nothing to gain by looking
        # earlier and a core to save by not.
        if context.vision is not None:
            context.vision.resume()
        self.direction = self.path.direction_for(pose)
        self._warn_if_started_outside_a_zone(pose)
        self._setup_manoeuvres(pose)
        self.progress, self.lateral = self.path.project(pose.x, pose.y, self.direction)
        self.aim_progress = self.progress
        self.distance_driven = 0.0
        self._last_tick_at = time.monotonic()

        print(f"Placed at {pose} -> running {RacingLine.direction_name(self.direction)}, "
              f"{self.laps_goal} laps of {self.path.length / 1000:.2f}m")
        # Not always forward: a round that starts inside a parking bay has a
        # wall in front of it, and setup() ends some milliseconds before the
        # first tick. Rolling for those milliseconds and stopping again is the
        # difference between the exit starting from where it was placed and
        # starting from against a bay wall.
        self.speed = self._start_speed()
        if self.speed:
            context.motor.forward(self.speed)
        else:
            context.motor.stop()

    def _setup_manoeuvres(self, pose):
        """
        Builds whatever owns the wheels before the path follower does, now
        that the pose is known and believable.

        Nothing here: the qualification round starts on the track. The final
        round overrides it to build the bay exit - see tasks/final/task.py.
        """

    def _start_speed(self):
        """
        What the drive motor is set to at the end of setup, before the first
        tick. Zero holds the robot still until a manoeuvre takes the wheels.
        """
        return int(self.setting("speed.base"))

    def _wait_for_localization(self):
        """
        Holds the robot still until the particle filter has actually found it.
        Driving off on a pose that has not converged is how you end up
        confidently steering into the center block.

        A timeout is not fatal: the filter reports its own confidence, and the
        loop already slows down when that is low, so it is better to roll
        cautiously than to refuse to start a competition run.
        """
        timeout = float(self.setting("startup.localize_timeout_s"))
        minimum = float(self.setting("safety.min_pose_confidence"))
        deadline = time.monotonic() + timeout

        placed = self.setting("startup.start_heading_deg")
        if self.setting("startup.assume_start_zone"):
            zones = self.context.nav.map.start_zones()
            # The heading matters more than it looks. The field is 90-degree
            # rotationally symmetric, so the lidar cannot tell the four
            # quadrants apart - only the IMU can, and the IMU's own zero is
            # wherever it booted until something anchors it. Passing the
            # heading here is what anchors it; without it the filter snaps
            # every pose into a quadrant chosen by an arbitrary but REPEATABLE
            # offset, which reads as "the robot always thinks it is facing the
            # same way, whichever way I put it down". See Pose's docstring and
            # NavigationManager.set_start_heading.
            self.context.nav.start(heading_deg=placed, zones=zones)
            print(f"Searching the {len(zones)} legal start cells...")
        self._warn_if_the_compass_is_adrift(placed)

        pose = self.context.nav.get_pose()
        while time.monotonic() < deadline:
            pose = self.context.nav.update()
            if pose.confidence >= minimum:
                print(f"Localized in {timeout - (deadline - time.monotonic()):.1f}s")
                return pose
            time.sleep(0.05)

        print(f"WARNING: still unsure of the pose after {timeout:.0f}s "
              f"(confidence {pose.confidence:.2f}) - starting slowly anyway")
        return pose

    def _warn_if_the_compass_is_adrift(self, placed):
        """
        Says so when the round is about to trust an unanchored compass.

        Not fatal, and not always wrong: without a compass at all the lidar
        picks one of the four quadrants and stays consistent with it, which is
        a valid alias of the truth and drives perfectly well. What does not
        work is a compass whose zero nobody has set, because the filter
        believes it over the lidar.
        """
        nav = self.context.nav
        if nav.compass is None or nav.compass_anchored:
            return
        print("WARNING: the compass has not been anchored, so the quadrant it "
              "puts the robot in is whatever its zero happened to be at boot - "
              "the same wrong way every run. Set startup.start_heading_deg to "
              "the direction the robot is PLACED facing (0=+Y, 90=+X, 180=-Y, "
              "270=-X), or pass --start-pose.")
        if placed is None:
            print("         Until then the lap direction, the start section and "
                  "the parking bay's position are all read off that quadrant.")

    def _warn_if_lookahead_unusable(self):
        """
        Says out loud what the lookahead settings actually do.

        Two ways to tune a number that has no effect. The corner cap overrides
        the speed-based lookahead inside every bend, and on this loop the bends
        are most of the lap - so a big lookahead_base_mm can be quietly ignored
        almost everywhere while looking like the thing being tuned. And a
        lookahead approaching the lap length aims the robot most of a lap
        ahead, which on a closed loop is a point behind the centre block.
        """
        pursuit, path = self.pursuit, self.path
        speed = float(self.setting("speed.base"))
        asked = pursuit.lookahead_distance(speed)
        corner_cap = pursuit.corner_lookahead_fraction * path.corner_radius

        if asked > path.length * 0.25:
            print(f"WARNING: lookahead {asked:.0f}mm is a quarter of the "
                  f"{path.length:.0f}mm lap - the target point is most of a lap ahead, "
                  f"which on a closed loop is across the field. Lower "
                  f"pursuit.lookahead_base_mm.")
        if pursuit.lookahead_min_mm > corner_cap:
            print(f"WARNING: in corners the lookahead is capped at {corner_cap:.0f}mm "
                  f"({pursuit.corner_lookahead_fraction} x {path.corner_radius:.0f}mm "
                  f"radius), below lookahead_min_mm={pursuit.lookahead_min_mm:.0f}. "
                  f"Most of this lap is corner, so lookahead_base_mm/min_mm barely "
                  f"matter - tune pursuit.corner_lookahead_fraction instead.")
        print(f"Lookahead: {asked:.0f}mm on the straights, "
              f"{corner_cap:.0f}mm in the corners")

    def _warn_if_started_outside_a_zone(self, pose):
        """
        The robot can only be placed in one of the four non-corner cells, so a
        pose in a corner means the filter has not really converged. It is not
        worth refusing to run over - the round drives the same loop wherever
        it starts - but it is worth saying, because it usually means the lidar
        or the mat dimensions are wrong.
        """
        for name, low, high in self.context.nav.map.start_zones():
            if low[0] <= pose.x <= high[0] and low[1] <= pose.y <= high[1]:
                print(f"Start cell: {name}")
                return
        print(f"WARNING: localized to {pose}, which is not one of the four legal "
              f"start cells - check the field dimensions in classes/field_map.py")

    def _warn_if_corners_too_tight(self):
        """
        A corner tighter than the robot's own turning circle cannot be driven,
        however good the controller is. Worth saying out loud at setup rather
        than discovering it as understeer into the block.
        """
        needed = self.pursuit.min_turn_radius_mm
        if self.path.corner_radius < needed:
            print(f"WARNING: path corner radius {self.path.corner_radius:.0f}mm is "
                  f"tighter than the robot's {needed:.0f}mm turning circle - "
                  f"raise path.corner_radius_mm or check pursuit.max_road_wheel_deg")

    # ========================================================================
    # CONTROL LOOP
    # ========================================================================

    def step(self):
        context = self.context
        now = time.monotonic()
        dt = 0.0 if self._last_tick_at is None else now - self._last_tick_at
        self._last_tick_at = now
        # Once per tick, before anything asks. Both the stop test and the
        # speed ramp want it, and two reads inside one tick can disagree
        # about what is in front of the robot.
        self._front_mm = self._measure_front_clearance()

        # Report BOTH how far we drove and how far we turned. The filter uses
        # them to predict with, and get_pose() uses them to carry the last
        # scan-derived pose forward to now - which is what lets this loop run
        # at 50Hz off a lidar that only produces a pose at 10Hz.
        distance = self._travelled(dt)
        # ONCE per tick, before anything reads the wheel angle. The servo is
        # still on its way to the last command, and the mean angle it actually
        # held over this dt - not the command - is what the robot turned on.
        held = self.pursuit.advance_servo(dt)
        context.nav.report_motion(distance, self._turned(distance, held))
        pose = context.nav.get_pose()

        self._track_progress(pose)

        # Parking owns the wheels outright once it starts: it is driving arcs
        # and short reverse strokes that pure pursuit has no way to express,
        # and the safety stops below would fight it (the front sector is
        # SUPPOSED to be full of bay wall). Odometry above is untouched, so
        # the filter keeps tracking through the manoeuvre - see
        # _drive_parking.
        # Same deal at the other end of the run: an exit from a bay the robot
        # started in owns the wheels for its first second or so. First,
        # because on tick 1 there is no useful racing line to follow from
        # inside a slot - see _drive_unparking.
        if self._drive_unparking(dt):
            if context.debug:
                self.show_debug()
            return

        if self._drive_parking(dt):
            if context.debug:
                self.show_debug()
            return

        self._update_lost_state(pose, now)

        self.target = self.target_point(pose)
        # Steer off where the robot WILL be once the wheels have caught up.
        wanted = self.pursuit.steering(self._lead_pose(pose), self.target)
        # ... then hold the servo to a sane speed. `steer_command` is the
        # LIMITED value from here on, so the odometry, the calibrator and the
        # status line all describe what the wheels were actually told to do.
        self.steer_command = self._limit_steer_rate(wanted, dt)

        if self._reverse_out(now):
            # Backing away from something; _reverse_out has already commanded
            # the motor, so leave the normal speed logic alone this tick.
            if context.debug:
                self.show_debug()
            return

        # Fit the steering gain off the RAW filter pose - the extrapolated one
        # is dead-reckoned from the gain we are trying to measure.
        self.calibrator.observe(context.nav.get_pose(max_age_s=None, extrapolate=False),
                                self.steer_command)

        self.speed = self._choose_speed(pose)
        cap, _ = self.parking_caps()
        if cap is not None:
            self.speed = min(self.speed, int(cap))
        # One serial message for both, and none at all when neither moved -
        # see MotorManager.drive().
        context.motor.drive(self.steer_command, self.speed)

        if context.debug:
            self.show_debug()

    def parking_caps(self):
        """
        What the parking approach wants the path follower limited to, as
        (speed, lookahead_mm) - either may be None for "no limit".

        Separate from parking_command because this applies while the PATH is
        still driving. A long lookahead is what makes pure pursuit cut a
        corner, and cutting it on the way into the bay means arriving at the
        staging pose off line and off square, which nothing downstream can
        correct - every step of the manoeuvre is an open arc measured from
        that pose.

        Nothing here: the qualification round never parks.
        """
        return (None, None)

    def block_lookahead_cap_mm(self):
        """
        What a nearby pillar wants the lookahead limited to, or None for no
        limit.

        Same job as the lookahead half of parking_caps, for the same reason: a
        long lookahead is what makes pure pursuit cut a corner, and the dodge
        around a pillar IS a corner. Aiming a full lookahead past a block puts
        the target point beyond the far side of the swerve, and the robot
        drives the chord instead of the curve - which is the line clipping the
        very block it was bending around.

        Nothing here: the qualification round has no pillars. The final round
        overrides it - see tasks/final/task.py.
        """
        return None

    def parking_command(self, dt):
        """
        The parking manoeuvre's (steer_command, speed) for this tick, or None
        when it is not driving.

        Nothing here: the qualification round never parks. The final round
        overrides it - see tasks/final/task.py.
        """
        return None

    def unparking_command(self, dt):
        """
        The exit manoeuvre's (steer_command, speed) for this tick, or None
        when it is not driving.

        Nothing here: the qualification round never starts in a bay. The final
        round overrides it - see tasks/final/task.py.
        """
        return None

    def _drive_unparking(self, dt):
        """
        Hands the wheels to the exit manoeuvre for a tick. Same contract as
        _drive_parking, and the same reasons for it.

        I/O:
            return: True if this tick was spent leaving the bay
        """
        return self._apply_manoeuvre(self.unparking_command(dt))

    def _drive_parking(self, dt):
        """
        Hands the wheels to the parking manoeuvre for a tick.

        The controller returns a command rather than touching the motor
        itself, so that `steer_command` and `speed` stay the single record of
        what the wheels were told - which is what _travelled() and _turned()
        dead-reckon from, and what the status line reports. The road-wheel
        angle has to be published to the pursuit object explicitly, because
        the usual writer of it (PurePursuit.steering) is not running.

        I/O:
            return: True if this tick was spent parking
        """
        return self._apply_manoeuvre(self.parking_command(dt))

    def _apply_manoeuvre(self, command):
        """
        Puts one manoeuvre command on the wheels, or reports that there was
        none to put there.

        I/O:
            command: (steer, speed) from a manoeuvre controller, or None
            return: True if the manoeuvre drove this tick
        """
        if command is None:
            return False
        steer, speed = command
        self.steer_command = clamp(float(steer), -self.pursuit.max_steer_command,
                                   self.pursuit.max_steer_command)
        self.speed = int(speed)
        self.pursuit.set_road_wheel_command(self.steer_command)
        self.context.motor.drive(self.steer_command, self.speed)
        return True

    def _travelled(self, dt):
        """
        Distance covered since the last tick, from the commanded speed.

        Zero with the drive motor disabled (--no-drive): the commanded speed
        then says nothing about how fast a hand is pushing the robot, and
        feeding the filter motion the wheels never made walks the pose away
        from where the robot really is. The lidar carries it instead, at scan
        rate rather than tick rate.
        """
        if not self.context.motor.drive_enabled:
            return 0.0
        return self.speed_mm_per_s(self.speed) * dt

    def speed_mm_per_s(self, command):
        """
        A speed command in millimetres per second - see
        startup.mm_per_s_at_full for why the 100 is not a percentage.
        """
        return command / 100.0 * float(self.setting("startup.mm_per_s_at_full"))

    def _limit_steer_rate(self, wanted, dt):
        """
        Caps how far the steering command may move in one tick.

        A servo driven straight from a pure-pursuit output takes whatever step
        the controller asks for, and a big step makes it snap - which shakes
        the mast, which blurs the camera frame the pillar detection depends on.
        Bounding the change per second turns that snap into a sweep without
        touching the slow, ordinary steering that makes up a corner.

        Costs a little tracking, because a correction that wanted to happen now
        happens over the next few ticks instead. Measure before lowering it:
            python test_steering.py --sweep steer.max_rate_units_s 60,120,240
        """
        if not self._steer_rate_limit or dt <= 0.0:
            return wanted
        step = self._steer_rate_limit * dt
        return clamp(wanted, self.steer_command - step, self.steer_command + step)

    def _lead_pose(self, pose):
        """
        The pose projected forward by `pursuit.lag_compensation_s`.

        Steering is an actuator with a delay: ask for 20 degrees and the wheels
        arrive there a tenth of a second later, by which time the robot is
        somewhere else. Feeding the controller the present pose therefore
        produces a correction that is always one lag too late, and a late
        correction is an over-correction - the robot swings past the line,
        corrects back, and weaves.

        Projecting the pose forward by the lag closes that gap: the command
        that lands is the one that was right for the moment it lands. This is
        the same dead reckoning NavigationManager uses between scans, just run
        a little further into the future, so it costs nothing.

        Left at 0 this returns the pose unchanged.
        """
        lead_s = float(self.setting("pursuit.lag_compensation_s"))
        if lead_s <= 0.0 or not self.speed:
            return pose

        distance = self.speed_mm_per_s(self.speed) * lead_s
        # Over the lead the wheels are still ARRIVING at the last command, so
        # predict the yaw with the mean angle they will hold over it. Assuming
        # they are already there over-turns the projected pose, which is what
        # forced lag_compensation_s to be held at less than half the measured
        # lag; with the approach modelled it can carry the real figure.
        turn = self._turned(distance, self.pursuit.mean_road_wheel_deg(lead_s))
        midpoint = math.radians(pose.heading + turn / 2.0)
        return replace(pose,
                       x=pose.x + distance * math.sin(midpoint),
                       y=pose.y + distance * math.cos(midpoint),
                       heading=(pose.heading + turn) % 360.0)

    def _turned(self, distance_mm, road_wheel_deg=None):
        """
        Yaw change over that distance, from the bicycle model and the angle the
        wheels were actually at.

        `road_wheel_deg` is that angle; left out it falls back to the modelled
        current one. NOT the commanded angle: the servo takes
        `pursuit.servo_lag_s` to get there, so during a correction the command
        and the wheels disagree by most of the correction, and dead-reckoning
        off the command hands the filter yaw the robot never did. See
        PurePursuit.advance_servo for what that costs on the mat.
        """
        if not distance_mm:
            return 0.0
        if road_wheel_deg is None:
            road_wheel_deg = self.pursuit.actual_road_wheel_deg
        road_wheel = math.radians(road_wheel_deg)
        return math.degrees(distance_mm / self.pursuit.wheelbase_mm * math.tan(road_wheel))

    def _track_progress(self, pose):
        """
        Folds this tick's projection into the lap counter.

        The delta is wrapped into +/- half a lap, so crossing the seam at the
        start of the loop counts as a small step forward rather than a whole
        lap backwards. Reversing counts backwards, which is what you want.
        """
        progress, self.lateral = self.path.project(pose.x, pose.y, self.direction)
        self.distance_driven += self.path.gap(self.progress, progress)
        self.progress = progress

    def _update_lost_state(self, pose, now):
        minimum = float(self.setting("safety.min_pose_confidence"))
        if pose.confidence >= minimum:
            self._lost_since = None
            return
        if self._lost_since is None:
            self._lost_since = now
        elif now - self._lost_since > float(self.setting("safety.lost_timeout_s")):
            self._stop_reason = (f"lost for {now - self._lost_since:.1f}s "
                                 f"(confidence {pose.confidence:.2f})")

    def _reverse_out(self, now):
        """
        Backs the robot away from whatever the front sector is stuck against.

        Without this the front-clearance stop is a trap: it sets the speed to
        zero, and a stopped robot cannot drive out of what stopped it, so the
        round sits there until it times out. Reversing for a moment and trying
        again is what a driver would do.

        The steering is INVERTED while reversing. Backwards, a given steering
        angle swings the nose the other way, so mirroring the pursuit command
        rotates the robot toward its target rather than further from it.

        I/O:
            return: True if this tick was spent reversing
        """
        if self._reversing_until is not None:
            if now < self._reversing_until:
                self.speed = -int(self.setting("safety.reverse_speed"))
                self.context.motor.drive(-self.steer_command, self.speed)
                return True
            self._reversing_until = None
            self._blocked_since = None

        front = self._front_clearance_mm()
        if front is None or front >= float(self.setting("safety.front_stop_mm")):
            self._blocked_since = None
            return False

        if self._blocked_since is None:
            self._blocked_since = now
            return False
        if now - self._blocked_since < float(self.setting("safety.stuck_timeout_s")):
            return False

        self._reversing_until = now + float(self.setting("safety.reverse_time_s"))
        print(f"Blocked at {front:.0f}mm - backing out")
        return True

    # ========================================================================
    # TARGET - the one thing the final round changes
    # ========================================================================

    def target_point(self, pose):
        """
        The point to chase: one lookahead along the path, shifted sideways.

        The lookahead is probed at speed first, then re-taken with the
        curvature over that stretch, so a corner shortens it - see
        PurePursuit.lookahead_distance.
        """
        reach = self.pursuit.lookahead_distance(self.speed)
        curvature = self.path.max_curvature_between(self.progress, reach, self.direction)
        lookahead = self.pursuit.lookahead_distance(self.speed, curvature)
        _, reach_cap = self.parking_caps()
        if reach_cap is not None:
            lookahead = min(lookahead, float(reach_cap))
        block_cap = self.block_lookahead_cap_mm()
        if block_cap is not None:
            lookahead = min(lookahead, float(block_cap))

        ahead = self.progress + lookahead
        # Published because the lateral target is a function of THIS point,
        # not of where the robot is - anything reasoning about when a shift in
        # the line starts has to measure from the same place, a lookahead
        # further on. See FinalTask._update_ramp_anchors.
        self.aim_progress = ahead
        return self.path.point_at(ahead, self.direction, self.target_lateral_mm(ahead))

    def target_lateral_mm(self, progress):
        """
        How far to the right of the racing line to aim at that progress.

        Zero here: the qualification round drives the line as generated. The
        final round overrides this to step around pillars, which is the whole
        of the difference between the two rounds' driving.
        """
        return 0.0

    # ========================================================================
    # SPEED
    # ========================================================================

    def _choose_speed(self, pose):
        """
        Slowest of everything that wants us slow: corner geometry, a shaky
        pose, and whatever the lidar can see straight ahead.
        """
        base = float(self.setting("speed.base"))
        corner = float(self.setting("speed.corner"))
        minimum = float(self.setting("speed.minimum"))

        speed = self._corner_speed(base, corner)
        if pose.confidence < float(self.setting("safety.min_pose_confidence")):
            speed = min(speed, float(self.setting("speed.lost")))

        front = self._front_clearance_mm()
        if front is not None:
            if front < float(self.setting("safety.front_stop_mm")):
                return 0
            slow = float(self.setting("safety.front_slow_mm"))
            if front < slow:
                # Ramp down over the last stretch rather than stepping, or the
                # robot lurches every time a wall comes into the front sector.
                stop = float(self.setting("safety.front_stop_mm"))
                fraction = (front - stop) / max(1.0, slow - stop)
                speed = min(speed, minimum + (speed - minimum) * fraction)

        if self._stop_reason:
            return 0
        return int(round(clamp(speed, minimum, 100.0)))

    def _corner_speed(self, base, corner):
        """
        Interpolates between the straight and corner speeds by how sharp the
        path gets within one lookahead - so the robot is already slow when it
        arrives at the corner rather than braking in it.
        """
        reach = self.pursuit.lookahead_distance(self.speed)
        curvature = self.path.max_curvature_between(self.progress, reach, self.direction)
        if curvature <= 0.0:
            return base
        sharpness = clamp(curvature * self.path.corner_radius, 0.0, 1.0)
        return base + (corner - base) * sharpness

    def _front_clearance_mm(self):
        """
        Closest lidar return in the forward sector this tick, or None if it is
        blind. Measured once per tick by step() - see _measure_front_clearance.
        """
        return self._front_mm

    def _measure_front_clearance(self):
        """
        Reads the forward sector out of the lidar.

        Called once a tick and cached, because _reverse_out and _choose_speed
        both want the answer and each call copies the whole 360-slot scan out
        from under the lidar thread's lock and re-applies the staleness mask.
        Correctness, not just cost: the two are deciding whether to stop and
        how fast to go, and they should be deciding it about the same frame.
        """
        lidar = self.context.lidar
        if lidar is None:
            return None
        half = float(self.setting("safety.front_sector_deg"))
        distance, _ = lidar.get_min_distance(-half, half)
        return None if math.isnan(distance) else distance

    # ========================================================================
    # FINISHING
    # ========================================================================

    @property
    def laps_done(self):
        return self.distance_driven / self.path.length if self.path else 0.0

    def is_finished(self):
        if self._stop_reason:
            print(f"Stopping: {self._stop_reason}")
            return True
        return self.laps_done >= self.laps_goal

    def finish(self):
        super().finish()
        if self._debug_view is not None:
            self._debug_view.close()
        if self.context.object_solver is not None:
            self.context.object_solver.close_debug()
        if self.calibrator is not None:
            print(self.calibrator.report())
        print(f"{self.laps_done:.2f} laps driven "
              f"({self.distance_driven / 1000:.1f}m of {self.laps_goal} x "
              f"{self.path.length / 1000:.2f}m)" if self.path else "no path built")

    # ========================================================================
    # REPORTING
    # ========================================================================

    def status(self):
        pose = self.context.nav.get_pose(max_age_s=None)
        return (f"[{self.elapsed:5.1f}s] lap {self.laps_done:4.2f}/{self.laps_goal}  "
                f"({pose.x / 10:+6.1f},{pose.y / 10:+6.1f})cm  "
                f"off-line {self.lateral / 10:+5.1f}cm  "
                f"{self.pursuit.status_line()}  speed={self.speed}"
                f"{'(off)' if not self.context.motor.drive_enabled else ''}  "
                f"conf={pose.confidence:.2f}")

    def show_debug(self):
        """
        Field view with the racing line and the pursuit target drawn on. ESC
        (window mode) ends the round, same as any other stop reason.

        Also the one place the object solver's two windows get painted: its
        detect() runs on the vision thread, which must not touch HighGUI, so
        it only renders (see ObjectSolver.show_debug). They go up before the
        field view, so DebugView's cv2.waitKey() paints all three at once.
        """
        if self._debug_view is None:
            self._debug_view = DebugView(self.context.nav,
                                         ascii_mode=self.context.ascii_debug)
        solver = self.context.object_solver
        if solver is not None and solver.debug:
            # ascii mode means there is no window loop to pump them - and no
            # waitKey either, so anything already up has to come down.
            if self._debug_view.ascii_mode:
                solver.close_debug()
            else:
                solver.show_debug()
        if not self._debug_view.show(draw=self._draw_overlay):
            self._stop_reason = self._stop_reason or "debug window closed (ESC)"

    def _draw_overlay(self, canvas, to_px):
        import cv2

        self.path.draw(canvas, to_px)
        if self.target is not None:
            cv2.circle(canvas, to_px(*self.target), 6, (60, 220, 240), 2)
            cv2.line(canvas, to_px(*self.target),
                     to_px(*self.path.point_at(self.progress, self.direction)),
                     (60, 220, 240), 1)
