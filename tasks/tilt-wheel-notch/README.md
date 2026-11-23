# Raise a car's tilt steering wheel 6 degrees on its stepless clamp column: free the lock lever, carry the wheel, clamp it

A car's tilt steering column stands on the bench: a dashboard block with the column coming out of it at 23 degrees, a 370 mm wheel on its end, and the lock lever under the column on the driver's left, as on the 2019 to 2022 Honda Insight column. The wheel starts clamped at its lowest position. The robot does what the owner's manuals give: pull the lock lever down, raise the wheel to the wanted height (here 6 degrees up the column's arc, read on an angle gauge), push the lever back up so the column clamps there, and let go.

**What makes it hard.** The column is stepless: a friction clamp holds it at any angle in its travel once the lever shuts, so there is no notch to drop into and the wheel is fixed wherever it sits when the lever closes. With the lever open the column is free and does not stay put: gravity and its tilt spring leave a net torque that either lowers the wheel (a droop) or raises it (a spring-up), so one hand has to carry the wheel at the target while the other closes the lever. Pushing the lever shut pushes on the column too, so the carrying hand has to hold the angle against that push. The lever stays where it is left and needs a firm push either way.

**Who does it, and how often.** Every driver who shares a car resets the wheel before driving, and fleet, bus and truck drivers do it at the start of a shift: 240 million licensed US drivers ([FHWA Highway Statistics 2024](https://www.fhwa.dot.gov/policyinformation/statistics.cfm)) and about 2.2 million US heavy and tractor-trailer truck drivers ([BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/transportation-and-material-moving/heavy-and-tractor-trailer-truck-drivers.htm); many heavy-truck columns tilt by an air valve instead, so read it as an upper bound). The steps are the usual ones for a lever-lock tilt column, as Kia's owner's manual gives them ([Tilt steering wheel, 2022 Kia Sonet](https://ownersmanual.kia.com/full_webhelp/QY/2022/en_IN/topics/chapter4_9_2.html)): pull the lock-release lever down, set the wheel, pull the lever up to lock it, then push the wheel up and down to check it is locked. The column itself, its lever and its forces are modelled on Honda's column below, a lever-lock column of the same kind.

**How the demo does it.** The right hand grips the rim 15 degrees above its right-hand spoke, pointing into the wheel with its jaws across the rim. The left hand comes in from the side between the wheel and the dashboard, pinches the lever's handle on its top and bottom faces and swings it down 24 degrees about the lever's pivot, turning the grip with the handle; the lever stays open and the hand lets go. The right hand then carries the rim up the column's arc in small steps until the column reads 6 degrees. The left hand grips the handle again and swings it back up to its stop while the right hand keeps the column at 6 degrees, correcting the angle as the lever's push moves it. The clamp engages, both hands open and back off, and the scorer's 5 N push at the rim checks the lock.

## Real items

- Honda Insight (2019 to 2022) steering column, part 53200-TBA-A02 (replaces 53200-TBA-A01; also fits the 2019 Civic), about $36 used: [ahparts.com listing](https://ahparts.com/buy-used/2019-honda-insight-shaft-steering-column-assy-53200-tba-a02-53200tbaa02/432373-1). Its forces are from Honda's shop manual page for the column ([Steering Column inspection, Insight 2019 to 2022](http://downloads.laboratoryb.org/insight/gen3/Inspection/Steering%20Column.pdf)):
  - "Using a commercially available push-pull gauge, push up the lock lever 10 mm (0.39 in) in from its end, and measure the force required to move the lock lever." "Lock lever force: Standard: 82 N (8.4 kgf, 18.4 lbf) max."
  - "Attach a commercially available spring scale (A) to the highest point of the steering wheel ... Pull the spring scale straight up, and read the force required to move the steering column." "Tilting force (upward/downward): Standard: 69 N (7.0 kgf, 15.5 lbf) max."
- 82 N is Honda's maximum for a column in spec, not its typical force; Honda publishes no typical figure for this column. The real YAM arm pushes about 20 N at this grip (the simulator's arm reaches 82 N in the range runs), so the simulator's default is 15 N; the act's difficulty does not rest on that force level, and the real trial sets the unit's lever to 15 to 20 N and checks that the clamp still holds, Real trial (e). The values above come from Honda's manual and the parts listing; the trial runner measures the bought unit in Real trial (e) before the first trial. The simulator's lever runs cover 15 to 82 N.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder, the scripted demo (the README command):

```bash
python demokit.py task.py OUT --mode full                                                                # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode none,liftonly,leveronly,overshoot,leveropen,restwheel,drop   # failure checks, below
TW_LEVER_N=82 python demokit.py task.py OUT --no-record --mode full                                     # range run: Honda's maximum lever force
TW_NET_NM=5.9 python demokit.py task.py OUT --no-record --mode full                                     # range run: the column springs up
```

`TRIAL_TRACE=1` prints the column angle, lever angle, lock state, hands-clear distance and score every 10 steps. The demo reads the column and lever angles from the simulator, as `demokit` allows; a policy gets the cameras only. The environment variables `TW_NET_NM`, `TW_LEVER_N`, `TW_PADDLE_MM`, `TW_LEVER_OPEN`, `TW_LEVER_SHUT`, `TW_DAMP` and `TW_FRIC` set the range runs below; unset, the scene is the default one.

**Result** (scripted, GPU physics, 20 Hz, RTX 4070 Ti SUPER): 1.0 at step 587 (29.35 s of simulated time), the column clamped at 5.95 degrees, against a limit of 1,200 steps.

**Dream-1** (the Dream-1 model on this task, one trial, 1,200 steps): 0 (the session held to the end).

**Range runs** (`full`, the same demo, each at an end of the window the real trial accepts, Real trial (c) to (e)):

| Setting | Column at the end | Score |
|---|---|---|
| default: droop, net 1.2 N m down; lever 15 N; handle 25 mm; lever frees at 10 degrees, clamps at 3; damping 4, friction 0.3 | 5.95 degrees | 1.0 (step 587) |
| lever 82 N (Honda's maximum; simulation only, the real trial runs at 15 to 20 N) | 5.95 degrees | 1.0 (step 595) |
| spring-up, net 5.9 N m up (as large as gravity's whole torque) | 5.97 degrees | 1.0 (step 482) |
| droop, net 3.0 N m down | 5.97 degrees | 1.0 (step 601) |
| handle 20 mm; lever frees at 20 degrees, clamps at 1; damping 1, friction 0 | 5.98 degrees | 1.0 (step 583) |
| outside the window: droop 4.5 N m | clamped at 6 degrees, but the arm holding 4.5 N m cannot back the rim hand 3 cm clear | 0 |
| lever 90 N (simulation only) | the lever stalls short of freeing the column | 0 |
| outside the window: handle 45 mm | the pinch does not hold it; the lever is not freed | 0 |

| Failure check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 |
| `liftonly` | the rim hand lifts; the lever is never touched | 0 (the column stays clamped at 0) |
| `leveronly` | the lever is pulled down and pushed back up; the wheel is not lifted | 0 (the column clamps where it drooped to, off the target) |
| `overshoot` | as `full`, but the wheel is lifted to 10 degrees before the lever closes | 0 (clamped outside 6 +/- 1) |
| `leveropen` | the lever is pulled down, the wheel lifted to 6 degrees and let go with the lever open | 0 (the wheel droops back to its lowest stop) |
| `restwheel` | as `full`, but the rim hand stays on the rim at the end | 0 (a hand on the wheel) |
| `drop` | the wheel is lifted to 12 degrees and let go, and the lever is shut as the falling wheel passes about 7 degrees; both hands clear | 0 (void: the clamp closed at 8.8 degrees on a column that moved 2.0 degrees in the 0.5 s before) |

## Sizes and forces

| Number | Scene | Real value and source | Effect on difficulty |
|---|---|---|---|
| Lever force at the handle | 15 N (default), the real trial's adjusted setting; simulation range run at 82 N | Honda's service setting: up to 82 N at 10 mm from the lever end (Insight shop manual). The real trial adjusts the bought unit to 15 to 20 N and measures it, Real trial (e) | a stiffer lever pushes the column harder while it shuts, so the carrying hand must correct more |
| Unlocked net torque at the target | 1.2 N m down (default); range runs 3.0 N m down and 5.9 N m up; 1 N m is about 3.1 N at the rim top | bound by calculation: with no tilt spring the net torque is gravity's alone, 3 kg x 9.81 x 0.23 m x cos 29 degrees = 5.9 N m down; a tilt spring moves it up from there. Honda's 69 N tilting force (lever loose) is the most it takes to move the column, friction included, not the net. Measured on the bought unit, Real trial (d) | the hand must carry the wheel either way |
| Column travel | 0 to 13 degrees, stepless | estimate; the bought unit is checked to give at least 7 degrees, Real trial (c) | a shorter real travel would still contain the target |
| Target | 6 +/- 1 degrees above the start | set by the task | |
| Column angle | 23 degrees from horizontal | estimate, a typical car rake | none on the act |
| Tilt pivot to wheel centre | 260 mm | estimate | |
| Wheel | 370 mm across, 25 mm rim | estimate: passenger-car wheels are 36 to 38 cm across | |
| Moving mass | 3 kg (wheel and upper column), centre 0.23 m from the pivot | estimate | sets the gravity part of the net torque |
| Lever | arm 68 mm from pivot to the handle; 25 degrees of travel; frees the column past 10 degrees down, clamps within 3 degrees of its stop (range run 20 and 1) | estimate from the Insight manual's lever figure; measured on the bought unit with the angle gauge, Real trial (c) | a lever that frees later or clamps closer to its stop needs a longer swing |
| Lever handle | 25 x 25 mm face, 10 mm thick (range run 20 mm) | estimate: the manual's push point is 10 mm in from the end, about half the handle; measured with calipers, Real trial (c) | the pinch closes across the handle's top and bottom, so it must open wider than the handle |
| Column hinge damping and friction (lever open) | 4 N m s/rad and 0.3 N m (range run 1 and 0) | estimate; the bought unit's free fall is timed from video, Real trial (d) | lower damping lets a let-go wheel fall faster |
| Clamp | a stiff joint lock engaged when the lever shuts | the real clamp is friction plates; the 5 N test push checks it holds | |
| Lock test | 5 N down the arc at the rim top, less than 1 degree over 0.5 s | set by the task, as the manual's own check of pushing the wheel up and down | |
| Void window | the column moved more than 1.5 degrees in the 0.5 s before the clamp closed (about 8 mm at the rim top, 319 mm from the pivot) | set by the task: a wheel carried by hand at the target moves far less | |

**Forces in the demo.** Pinch 40 N a finger (the YAM's figure). The rim lift is a few newtons at the rim (up to about 19 N in the 5.9 N m spring-up run). The lever push is the set lever force: 15 N by default, up to 82 N in the simulation range runs.

## Real trial

(a) **Bench.** A used Honda Insight 2019 to 2022 steering column with its wheel (part 53200-TBA-A02, about $36 used), bolted to a bench frame by its two column brackets at 23 degrees from horizontal, the lever under the column on the robot's left, a dashboard board 0.56 to 0.70 m in front of the arm bases with a yellow tape mark at the rim-top height of the 6 degree target. Pad covers on the gripper fingers.

(b) **Tools.** Two digital angle gauges, Wixey WR300 (0.1 degree resolution, about $30; [wixey.com](https://www.wixey.com/)), one on the upper column shroud and one on the lever; calipers, Mitutoyo 500-196-30 ([mitutoyo.com](https://www.mitutoyo.com/)); a force gauge, Wagner FDIX-25 (0 to 25 lbf, about 111 N; [wagnerinstruments.com](https://wagnerinstruments.com/)), used as the spring scale (pull) and as the push-pull gauge (push); a camera at 30 frames per second, Logitech C920 ([logitech.com](https://www.logitech.com/en-us/shop/p/c920-pro-hd-webcam.960-001055)), facing the column side-on, with a steel ruler, Starrett C604R-12 ([starrett.com](https://www.starrett.com/)), in view beside the rim top.

(c) **Bench check, geometry.** With the lever open, raise the column from its lowest to its highest position and read the gauge: at least 7 degrees of travel. With the gauge on the lever, read the angle below its stop at which the column frees (accepted: 20 degrees or less) and the angle at which it clamps again on the way up (accepted: 1 degree or more below the stop). Measure the handle with calipers: 20 to 25 mm from its top face to its bottom face, where the gripper closes on it. A unit outside any of these is outside the task's range: choose another unit.

(d) **Bench check, unlocked column.** By Honda's method: open the lever, set the column at 6 degrees, hook the spring scale at the rim top and read the force to start the column moving up (F up) and down (F down). The net force is (F up - F down) / 2 and the friction (F up + F down) / 2. Accepted net: from 9.4 N down (3.0 N m) to 18.5 N up (5.9 N m); the range runs pass at both ends. Honda's own limit, F up and F down each 69 N or less, must also hold. Then raise the wheel to the top of its travel with the lever open, let go, and time its fall to the bottom stop from the video; record it with the unit (the runs pass from damping 1 to 4 N m s/rad). A unit outside a window is outside the task's range: choose another unit and report it.

(e) **Lever force, set and checked.** Honda's limit for this column is 82 N, a maximum for a column in spec, not its typical force; the real YAM arm pushes about 20 N at this grip (the simulator's arm reaches 82 N in the range runs). The act's difficulty does not rest on the force level: the 5 N hold below is what the trial judges. Set the bought column to the trial's range: work the lever three to five times, set the column mid-travel, push the lever up with the push-pull gauge 10 mm in from its end (the shop manual's method), and loosen the clamp's lock-bolt nut at the lever pivot a little at a time until the reading is 15 to 20 N. Then check that the clamp still holds at that setting: with the lever shut, pull the spring scale at the rim top down the arc to 5 N and then up the arc to 5 N; the gauge moves less than 1 degree each way, and the wheel let go holds its angle for 60 s. A column that does not hold at 15 to 20 N is not used. The simulator runs the act from 15 to 82 N (each 1.0; 90 N fails).

(f) **Reset.** Pull the lever down, lower the wheel by the rim to its lowest stop, push the lever up, zero the angle gauge. About 20 s; nothing is consumed.

(g) **One trial.** Start the policy with both arms at home; stop at 60 s.

(h) **Scoring by hand.** 0.5: the lever is shut, the gauge reads 6 +/- 1 degrees above zero, and the end video shows no part of either gripper within 3 cm of the wheel, column or lever for at least 1 s. 1.0: then pull the 5 N spring scale reading down the arc at the rim top for 0.5 s; the gauge moves less than 1 degree.

(i) **Void.** From the side video: in the 15 frames (0.5 s at 30 frames per second) before the lever's last movement, the rim top is more than 8 mm from where it was 15 frames before (read against the ruler; 1.5 degrees of column), a dropped or swung wheel clamped as it passes. Score 0.

(j) **Numbers.** Every number above is the one in `task.py`: target 6 +/- 1 degrees, the lever frees past 10 degrees and clamps within 3 degrees of its stop, hands clear 3 cm, hold 1 s, test push 5 N for 0.5 s and less than 1 degree, void when the column moved more than 1.5 degrees in the 0.5 s before the clamp closed.

(k) **Hands-clear in the simulator** uses a lattice of points 2 to 3 cm apart over each gripper (fingertips, pads, palm, housing) against the wheel, column and lever; the real trial judges it from video as in (h).
