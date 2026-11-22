# Fix a GoPro to a UMI gripper: its two mount fingers between the holder's three, the thumbscrew through all five

The UMI handheld data-collection gripper, as printed from its released files, stands upright in a bench vise. Its only sensor, an action camera, is fixed on its top cover by the standard GoPro mount: the camera's two folding fingers go between the holder's three, and one thumbscrew runs through all five fingers into a captive M5 nut. The five holes are hidden inside the stack, so one hand must hold the camera square in the gaps, lens toward the gripper's fingers, while the other pushes the screw's tip through each hole to the nut and then turns it home (about 1.4 turns in the simulated nut, about 2.5 in a real one), in wrist-roll strokes with a regrip after each. Only turning counts: a screw pushed in without turning is credited no deeper than the nut's face.

Handheld grippers like this are how robot-learning labs now gather demonstrations at scale. One data-scaling study collected over 40,000 demonstrations with them in 32 environments (Lin et al., ICLR 2025), and FastUMI-100K released over 100,000 trajectories, about 600 hours (2025). Every gripper built and every camera swapped is fixed with this mount.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,camera_only,no_turn,push_seat,light_spin,slow_light_spin
```

The harness's instruction (Dream-1 run): `--instruction "Fix the GoPro to the gripper's holder, lens toward the gripper's fingers: thumbscrew through all five fingers, snug in the nut. Then let go."`

**Result** (scripted, GPU physics, 20 Hz): the full mode scores 1.0 on an RTX 3090 Ti (2,365 steps, 118.25 s, against a limit of 3,200). Each route in the table below ran on GPU on the final scorer.

**Dream-1** (the harness's model policy, one trial, 3,200 steps): 0.0. The model's right arm reached toward the gripper and the holder, but the camera was never fitted between the holder's fingers and the screw was never fetched.

## Scoring, and the routes that score less

The tier numbers are simulation scores. A real trial reads every tier at the end of the run, by the means in the right-hand column.

| Tier (sim) | Reached when | Real check |
|---|---|---|
| 0.25 | the camera's two fingers are interleaved with the holder's three: the camera within 1.5 mm of its mounted pose along the hole line, 3 mm and 2.5 mm across it, and within 8 degrees of the mounted pose (lens toward the gripper's fingers) | close photo: both camera fingers in the holder's gaps, lens toward the gripper's fingers |
| 0.5 | and the screw is through the stack from the near side: its axis within 2 mm of the holes' line, its knob's face short of the first finger and its tip at the nut's thread (no more than 0.3 mm short of it) | ruler photo of the exposed shaft: the knob's face no more than about 2 mm short of the first finger (18 mm shaft, thread starting 16 mm behind the first finger's near face), which puts the hidden tip at the nut |
| 0.75 | and the screw is seated by its turns: its tip and its turned depth both within 0.4 mm of the seat, the knob's face on the first finger. The depth counts only as far as the screw has turned since its tip reached the nut (0.8 mm a turn, the M5 pitch, plus 0.5 mm of play): the simulated nut seats at about 1.4 turns, (2.0 mm from the thread's start to the seat, less 0.5 mm of play and 0.4 mm of tolerance) / 0.8 mm a turn | no light gap between the knob's face and the first finger (about 2.5 turns in a real nut, 2.0 mm at 0.8 mm a turn); a paint stripe on the knob's end shows it was turned |
| 1.0 | and each whole gripper (fingers, pads, palm and housing) is at least 5 cm from the camera and the screw, with both still, for 1 s | video: both grippers 5 cm clear, nothing moving for 1 s |

The tiers are read live; the 1.0 is judged once and then holds. Once the screw is seated, the camera and the screw are fixed where they stand (a tightened thumbscrew's friction), so the judge reads a mount that no longer moves. `check_score.py` runs the scorer on hand-built states with no simulator (`python3 check_score.py`), including routes with no GPU mode: a screw held on the hole line past the far finger (0.25), the screw pushed to the seat before the camera is squared in its gaps (0.5), turns in and then turned back out to the start (0.5), the camera turned round (0) and 2 mm out of its gaps along the hole line (0).

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | right picks the camera off the bench across its width (35 N), carries it lens down, pitches it fingers down away from the gripper's body and lowers its fingers into the holder's gaps, levelling it by its measured rotation; left takes the thumbscrew's knob where it overhangs the vise jaw, lifts it, levels it, aims the tip at the first hole and pushes it through to the nut; then 35-degree wrist-roll strokes about the hole line, the hand opening and swinging back after each, following the screw in; both hands let go and draw back | 1.0 (2,365 steps, 118.25 s) |
| push_seat | as full to the nut, then the left pushes the screw on to the seated depth without turning it; both hands let go | 0.5 (pushed to the seat; the turned depth stays at 0.5 mm, short of the 1.6 mm credited (1.1 mm turned plus 0.5 mm play) the 0.75 needs) |
| no_turn | as full to the nut, then both hands let go, the screw not turned | 0.5 (the tip at the nut, no turns) |
| light_spin | as full to the nut, then the knob is held at only 2 N while it is rolled, six regrips | 0.5 (0.10 turn in six regrips, logged: a 2 N grip cannot turn the snug thread, far short of the 1.4 turns, 1.6 mm credited (1.1 mm turned plus 0.5 mm play), the 0.75 needs) |
| slow_light_spin | as light_spin, the strokes taken at 2 deg/s, under the speed at which the thread's resisting torque reaches its cap | 0.5 (each 35-degree stroke takes 17.5 s, so the 3,200-step run fits five strokes, 0.49 turn commanded; the script's cap is six strokes, 0.58 turn. The run logged 0.71 turn, read from the screw's pose: 0.13 to 0.22 turn more than commanded, measured, its cause not isolated. Either way it is far short of the 1.4 turns the 0.75 needs) |
| camera_only | the right mounts the camera and holds it; the screw is not touched | 0.25 |
| none | nothing | 0.0 |

## Real items

- GoPro HERO13 Black, model CHDHX-131: 71.8 x 50.8 x 33.6 mm, 159 g with battery, built-in folding mounting fingers (2.7 mm thick in the model). https://gopro.com/en/us/shop/cameras/buy/hero13black/CHDHX-131-master.html
- Fotodiox GoTough Long thumbscrew, model GT-SCW45: a 45 mm aluminium knob on an 18 mm M5 stainless shaft, 17 g. Its long knob gives a parallel gripper a grip that GoPro's own flat-paddle thumbscrew, which sits under the camera body, does not. https://fotodioxpro.com/products/gt-scw45

## Sizes

| Item | Size |
|---|---|
| Holder | the UMI top cover's three fingers 4.0, 3.3 and 4.0 mm thick with 3.3 and 3.4 mm gaps, 18 mm in all, 15.2 mm wide round the holes; the holes' line along x at y 0, 136.4 mm up, from x 0.331 to 0.349 m; a hex M5 nut in a boss behind the far (+x) finger |
| Gripper (fixed) | the real UMI body from its print files: top cover with the mount and the two mirror wings, bottom plate, finger holders and two Fin Ray fingers toward -y; the body is 172 mm long and 91 mm across; a 30 x 35 mm handle bar under it, with a trigger, is what the vise holds |
| Vise | jaws 80 mm wide, tops 65 mm up, closed on the handle bar; movable jaw at y 20 to 60 mm |
| Camera | GoPro HERO13 Black: 71.8 x 50.8 x 33.6 mm, 159 g with its battery; two mounting fingers 2.7 mm thick, 15 mm wide, 20.5 mm out of the body, holes 8 mm from the tips; starts lens up at (0.21, -0.16) |
| Thumbscrew | Fotodiox GoTough Long: knob 45 mm long, 15 mm across; M5 shaft 18 mm, 5 mm across; 17 g; starts on the movable jaw along x, its knob's end 18 mm over the jaw's -x edge |

## Colliders

| Body | Colliders | Free or fixed |
|---|---|---|
| Holder | each finger as boxes round a square hole (5.4 mm in the outer two, 5.8 mm in the middle one), with chamfered wedges on top; the nut's boss as boxes round a 5.4 mm hole | fixed |
| Gripper body, handle bar, vise | one box for the body, a box for the mount's foot, a convex hull of each mirror wing, boxes for the handle bar and the vise; the visual meshes of the real cover, plate, finger holders and Fin Ray fingers carry no collider of their own | fixed |
| Camera | one box for the body; each mounting finger as three boxes round a 5.8 mm square hole and a chamfered wedge for its tip | free, 159 g |
| Thumbscrew | a 16-sided hull for the knob, one for the shaft | free, 17 g |
| Fingers | the harness's YAM finger colliders | the robot |

## Robot figures the demo keeps within

The YAM's jaws open to 94 mm and grip about 40 N a finger; each arm reaches about 60 cm. The right grips the 71.8 mm camera across its width at 35 N and holds it in the holder until the screw is seated. The left grips the 15 mm knob at 30 N and turns it in 35-degree wrist rolls, within the wrist's roll range. The thread's friction is a drive on the screw's turn: a resisting torque of 0.05 N m per deg/s, capped at 0.15 N m (reached at 3 deg/s), about the finger-tight torque of a small thumbscrew; at full speed a 2 N grip cannot turn it, and at 2 deg/s a 35-degree stroke takes 17.5 s, so the run limit fits five strokes (0.49 turn commanded, 0.71 logged), and the script stops at six (0.58 commanded): too few to seat. A real trial is stricter after seating: the simulator clamps and latches the camera and screw, while a real thumbscrew is held only by its thread's friction.

## Real trial

Print a UMI gripper from its open files and clamp it upright in a bench vise (by a bar under the body, or the body's own base) so the holder's hole line runs left to right at about 136 mm above the bench, as in the setup. Any GoPro-pattern three-finger mount with a captive M5 nut, screwed or clamped to a block in the vise at the same height, also serves. Check the nut sits in its pocket. Fold the camera's mounting fingers out and lay it lens up on the bench 13 cm nearer the robot than the holder and 16 cm to the robot's right of it, its fingers pointing right. Unscrew the thumbscrew fully and lay it on the vise's movable jaw, pointing at the holder, its knob's end 18 mm over the jaw's edge.

What the video and a close photo of the mount see, by the scorer's terms:
- INTERLEAVED: the camera's two fingers in the holder's gaps, the camera upright and its lens toward the gripper's fingers.
- THROUGH: the screw's tip through the holes to the nut, the screw on the holes' line.
- SEATED: the knob's face down on the first finger, reached by turning; a screw pushed in without turning does not hold in a real nut.
- RELEASED: both robot grippers well clear (5 cm) of the camera and the screw, and nothing moving, for 1 s.

| Real-trial kit | Use | Price (about) |
|---|---|---|
| A camera with GoPro folding fingers (a used HERO8 or later) | the camera | $100 |
| Fotodiox GoTough Long thumbscrew | the screw | $15 |
| GoPro-pattern three-finger mount with a captive M5 nut (or a UMI gripper, printed from its open CAD) | the holder | $10 |
| 4 in bench vise | holds the gripper or mount | $30 |
| Phone and tripod | video and photo | $0 to $15 |
| Total | | under $170 |

Reset, about 2 minutes: unscrew the thumbscrew fully and lay it on the jaw as above; fold the camera's fingers out and lay it lens up at its start mark; check the nut is in its pocket and the holes are clear.

## Safety for a real trial

Use an unpowered camera with its battery out. Keep hands clear of the vise and the holder while the arms work; the screw turns under a 30 N grip. Fit rubber pad covers on the grippers' fingers so the aluminium knob and the camera's edges do not mar them, and keep the benchtop under the holder clear so a dropped screw cannot roll away.

## What is modelled, and what is not

- All bodies are rigid. The camera and the thumbscrew are free bodies of their real size and mass, with colliders as above; the gripper, holder and vise are one fixed body.
- The thread is a rule. Once the screw's tip reaches the nut's thread from the near side, its axis on the holes' line and the camera in its gaps, a joint made on the live stage leaves the screw only its turn and its slide along that line, up to the seated depth; the tip cannot back out of the nut, and the turn is resisted by a drive (0.15 N m at any turning speed above 3 deg/s, about a finger-tight small thumbscrew's torque). The scorer counts depth past the nut's thread start only as far as the screw has turned (0.8 mm a turn, the M5 pitch, plus 0.5 mm of play), so a push without turning reaches 0.5 at most. The far finger and its captive nut are taken as one threaded part.
- The holes are square in sim and 5.4 to 5.8 mm across; a real mount's are round, about 5.2 to 5.5 mm. With the real fit, the long knob's weight droops a released screw off the hole line in sim.
- Once seated, fixed joints hold the camera and the screw where they stand: the friction of a tightened thumbscrew, which is not modelled as contact.
- The holder, cover, plate, finger holders and soft fingers are the UMI's released print meshes; collision uses simple shapes (a box for the body, hulls for the wings, boxes round each hole), so the soft fingers do not collide and do not flex. The handle bar is a stand-in for the part a vise would clamp. The camera's finger length and thickness (2.7 mm) are estimates of the GoPro mount pattern; the maker does not publish it.
- The demo reads the camera's and the screw's poses to level them and to aim the tip.

## Sources

- UMI, the handheld gripper, its GoPro as the only sensor: Chi et al., "Universal Manipulation Interface", RSS 2024, https://arxiv.org/abs/2402.10329 ; open CAD, https://cad.onshape.com/documents/a8a98201374f5cc9c6538790 ; print files and code (MIT licence, copyright the UMI authors), https://github.com/real-stanford/universal_manipulation_interface
- Over 40,000 demonstrations with handheld UMI grippers in 32 environments: Lin et al., "Data Scaling Laws in Imitation Learning for Robotic Manipulation", ICLR 2025, https://arxiv.org/abs/2410.18647
- Over 100,000 trajectories, about 600 hours, with UMI-style grippers: FastUMI-100K, 2025, https://arxiv.org/abs/2510.08022
- GoPro HERO13 Black: 71.8 x 50.8 x 33.6 mm, 159 g with battery, built-in folding mounting fingers (retailer spec sheet), https://cliftoncameras.co.uk/uploads/downloads/Product%20Specifications/GoPro/GoPro%20Hero13%20Black%20Full%20Spec.pdf
- Fotodiox GoTough Long thumbscrew: 45 mm aluminium knob on an M5 stainless shaft, https://fotodioxpro.com/products/gt-scw45
- M5 coarse thread pitch 0.8 mm: ISO 261.
- The camera mount's finger length and thickness, hole sizes, the vise and the handle bar: estimates.

GoPro and HERO are trademarks of GoPro, Inc.; Fotodiox and GoTough are named for identification only. This task is not affiliated with them.

## Licence

This task: MIT. The gripper meshes in umi_body.npz are derived from the print files in the UMI project's repository (real-stanford/universal_manipulation_interface), which is released under the MIT licence; its copyright notice belongs to its authors.
