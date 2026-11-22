# Twist-lock a camlock tension rod: push both ends out to the shelf walls at an angle, turn the tubes to lock, and swing it level so it wedges

A twist-lock rod holds only if the length is locked a few millimetres long: two hands must push its ends out to both walls, hold the outer tube still while the inner tube is turned against it, then swing the rod level so its rubber caps squeeze into the walls. RV owners set such a rod across each open shelf before travel; the RV Industry Association counted 11.2 million US households owning an RV in 2021, and the 2025 Go RVing owner profile reports about nine RV trips a year.

**Instruction.** Set the tension rod at an angle between the shelf's side walls, twist its tubes to lock the length, then swing it level so it wedges.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --seed 1 \
  --instruction "Set the tension rod at an angle between the shelf's side walls, twist its tubes to lock the length, then swing it level so it wedges."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                              # full score, with a video
python demokit.py task.py OUT --no-record --mode nolock                # the lazy routes, each in its own run
python demokit.py task.py OUT --no-record --mode squeeze
python demokit.py task.py OUT --no-record --mode hold
ROD_HOLD_N=80 ROD_CAM_NM=0.2 ROD_CAP_N_PER_MM=40 ROD_INNER_MM=14.3 ROD_MU=0.5 \
  python demokit.py task.py OUT --no-record --mode full                # one corner of the estimates' real-trial ranges
```

The demo reads the rod's pose and joint angles to plan its grasps; it reads no contact force and moves the rod only through the grippers. Both hands grip the rod from above (left on the outer tube, right on the inner, 40 N each), lift it 12 cm, and push its two caps out to the side walls with the rod 6 degrees off square, each cap against its wall, which locks the length about 2 mm longer than the gap straight across. The left hand holds the outer tube while the right turns the inner tube five eighth-turns (225 degrees commanded; the grip slips some of it), letting go and regripping between turns and keeping the last grip. After each regrip the right hand lifts its end back level with the left (a pinch grip lets the rod pivot). Both hands then carry the rod level, which squeezes the caps about 1 mm each, and trim it by eye from the rod's pose; the left lets go and moves clear, the right trims its end in line with the left once more, lets go and moves clear.

## The scene and what is simplified

Station: two YAM arms with parallel grippers, bases 61 cm apart at the near edge of the table. A cupboard bay stands on the table: an 18 mm board shelf, side walls 80 cm apart (inner faces) and 32 cm tall, a back panel, and three 15 oz food cans (67 x 111 mm) at the back. The rod lies on the shelf near the front, set to 75 cm tip to tip (tape), unlocked, its right cap by the right wall. The guard-bar line is 9 cm in from the shelf's front edge and 12 cm above the shelf, clear of the cans' 11.1 cm.

- The rod is one articulated body: the outer tube is its frame; the inner tube slides along and turns about the rod's axis inside it; each end cap is a 4 mm slider on a spring, the rubber's give. The rod has no spring of its own.
- The cam is a coupling in code. Within 20 degrees of the as-found angle the slide moves against 2 N; from 80 degrees it holds the length with up to 150 N; between, the hold rises linearly. Turning the inner tube takes 0.15 N m (a stick-slip hold of its angle).
- Walls and shelf are rigid. Friction 1.0 static, 0.9 moving, on the caps, the tubes and the boards. Rigid bodies at 240 Hz. Rubber on painted board is nearer 0.5 to 0.8; at 0.5 the demo's squeezed caps (about 0.85 mm at 25 N/mm, 21 N each) still give 2 x 21 x 0.5 = 21 N of hold against the rod's 2.4 N plus the 10 N push.

## Sizes

The rod is built to the maker's sizes. Values with no maker figure are estimates; each has a real-trial measurement and a range, and the demo scores 1.0 at the ends of the ranges listed under Results. Each estimate except the unlocked slide force can be set by environment variable for a run.

| Item | Sim | Real | Source | Measured in the real trial | If the real value differs |
|---|---|---|---|---|---|
| Outer tube | 15.9 mm, white steel | 5/8 in | Maker listing | | |
| Length range | 71 to 122 cm | 28 to 48 in | Maker listing | | |
| Mass | 245 g | 8.64 oz | Maker listing | | |
| Shelf span | 80 cm | Any span in the rod's range | Scene choice | Tape | |
| Inner tube (`ROD_INNER_MM`) | 13 mm | 12.7 to 14.3 mm | Estimate: it slides inside the 15.9 mm tube, whose wall and cam sleeve take 0.8 to 1.6 mm a side | Calipers, accepted 12.7 to 14.3 mm | A thinner tube is harder to grip and turn |
| Cam hold, locked (`ROD_HOLD_N`) | 150 N | 80 to 200 N | Estimate. Lower bound: above the caps' force when the rod is level (up to 40 N/mm x 1.6 mm = 64 N a cap), else the rod shortens and drops. Upper bound 200 N: a cam of this size is turned by a 0.1 to 0.2 N m finger twist, so a hold beyond about 200 N along the tube would need a cam with more lever than fits in a 13 mm tube; the scorer needs only the lower bound and is unchanged above it | Rod locked, outer tube clamped, a hanging luggage scale pulls the inner tube's end along the axis until it slips: accepted 80 to 200 N | A weaker cam lets a long lock slip shorter |
| Turn to lock (`ROD_LOCK_DEG`) | Free within 20 deg, locked from 80 deg | 60 to 120 deg | Estimate: a cam set by a quarter turn or so | Protractor under a tape flag on the inner tube, turned until firm: accepted 60 to 120 deg | More turn to lock needs more regrips |
| Torque to turn the cam (`ROD_CAM_NM`) | 0.15 N m | 0.1 to 0.2 N m | Estimate: the cam is set by a finger-and-thumb twist; fingertips squeezing a 13 mm tube at about 20 N with friction 0.8 turn at most 2 x 20 x 0.8 x 6.5 mm = 0.21 N m | A luggage scale on a 10 cm lever taped to the inner tube: 1 to 2 N, accepted 0.1 to 0.2 N m | More torque needs a firmer grip and a firmer hold on the outer tube |
| Unlocked slide (`SLIDE_N`, free within 20 deg of the as-found angle) | 2 N to push the inner tube in or out | 1 to 3 N | Estimate: a camlock rod slides by hand when unlocked | Luggage scale pulling the unlocked inner tube's end along the axis: accepted 1 to 3 N | A stiffer slide lets an unlocked rod hold its length, so the unlocked drop is rarer |
| End cap size | 32 mm long (20 mm sleeve, flare to a 30 mm face), 4 mm of it gives | 25 to 35 mm | Maker photo | Ruler on the cap | None: only the 4 mm give acts |
| End cap give | Up to 4 mm | 3 to 5 mm | Estimate from the cap's flared rubber cup (maker photo) | Calipers on the cap, pressed until firm | The demo uses about 0.85 mm (0.80 to 0.97 across the runs) |
| End cap stiffness (`ROD_CAP_N_PER_MM`) | 25 N/mm | 20 to 40 N/mm | Estimate, bounded below by the maker's 4 lb (17.8 N) rating: set as instructed the rod holds that load plus its own 2.4 N, which at rubber-on-paint friction 0.5 needs (17.8 + 2.4) / (2 x 0.5) = 20 N at each cap, from about 1 mm of give | Cap pressed on a kitchen scale, compression by calipers over the first 2 mm: accepted 20 to 40 N/mm | Softer caps hold with less force; stiffer caps need a more exact length |
| Friction, caps and tubes (`ROD_MU`) | 1.0 static, 0.9 moving | 0.5 to 1.0 | Estimate: rubber on painted board about 0.5 to 1.0 | Tilt a board with the cap on it until it slides, tan of the angle: accepted 0.5 to 1.0 (or a luggage scale drag) | Lower friction needs a more exact length; the demo scores 1.0 at 0.5 |
| Grip force | 40 N | YAM gripper | Robot | | |

## Scoring

Measured at the end with a tape, a ruler and an inclinometer on the rod. Score 0 if the rod rests on the shelf or on the cans, or if either cap face is off its wall (more than 3 mm from the wall face or outside it). Rungs are cumulative:

- 0.33: the rod is off the shelf with both cap faces on the side walls.
- 0.67: also the cam is locked: the inner tube turned to the cam's lock angle (80 degrees in the sim; on the real rod, measure it beforehand with a protractor, 60 to 120 degrees, or check that the rod holds its length when pulled by hand).
- 1.0: also, with the rod level within 3 degrees and its middle 12 cm (plus or minus 1.5 cm) above the shelf, both whole grippers (fingertips, pads, palm and housing) more than 3 cm from the rod for 5 s, then a 10 N push down at mid-span for 2 s sags it less than 5 mm, and it stays on both walls.

An unlocked rod held level between the walls has its caps pushing its tubes shorter: it drops when let go. A rod locked short drops; a rod locked level with no overshoot does not wedge and sags under the push.

## Results

Scripted demo, RTX 3090 Ti:

| Run | Score | Rod at the end |
|---|---|---|
| `full`, sim values | 1.0 | success at step 1015 (50.75 s); caps squeezed 0.84 and 0.85 mm, level 0.7 deg, 116 mm above the shelf, sag under the 10 N push 0.0 mm, grippers 56 mm clear |
| `full`, hold 80 N, torque 0.2 N m, caps 40 N/mm, inner 14.3 mm, lock 120 deg | 1.0 | caps 0.95 and 0.97 mm, sag 0.0 mm |
| `full`, 200 N, 0.1 N m, 20 N/mm, 12.7 mm, 60 deg | 1.0 | caps 0.80 and 0.81 mm, sag 0.0 mm |
| `full`, 80 N, 0.1 N m, 20 N/mm, 14.3 mm, 120 deg | 1.0 | caps 0.91 and 0.92 mm, sag 0.0 mm |
| `full`, 200 N, 0.2 N m, 40 N/mm, 12.7 mm, 60 deg | 1.0 | caps 0.85 and 0.86 mm, sag 0.0 mm |
| `full`, 80 N, 0.2 N m, 20 N/mm, 12.7 mm, 120 deg | 1.0 | caps 0.94 and 0.96 mm, sag 0.0 mm |
| `full`, friction 0.5 (`ROD_MU=0.5`) | 1.0 | caps 0.95 and 0.97 mm, sag 0.02 mm |
| `full`, friction 0.7 | 1.0 | success at step 1016 |
| `nolock`: pushed out and levelled without the twist | 0.0 | the caps push the tubes shorter; the rod drops to the shelf |
| `squeeze`: held level between the walls, unlocked, let go | 0.0 | drops to the shelf |
| `hold`: the full demo, but the right hand never lets go | 0.67 | locked and level, gripper touching the rod |

**Dream-1 run** (the harness's own policy, 2,400 steps, one trial, one session, RTX 3090 Ti): 0. The policy did not lay the rod between the walls in the 120 s run (no tier reached). The film is `model_run.mp4` (top camera).

## Runtime and seeds

Nothing is random (`object_jitter=0`). The full demo takes about 6 minutes of wall time on an RTX 3090 Ti with video, about 2 without.

## Real trial

Bench setup, from the bought items in Real items. All positions are in mm from the bay's front edge along the shelf's depth (x) and from its centre line across (y).

1. Build the bay from 3/4 in (18.5 mm) birch plywood: a shelf 37 cm deep and about 83.7 cm wide, two side walls 32 cm tall and 37 cm deep screwed and glued to the shelf's ends so their inner faces are 800 mm apart (check with a tape at the front, middle and back: 800 +/- 2 mm), and a back panel 800 mm wide and 320 mm tall. Check the shelf with a level. Leave the inner faces bare or painted, not waxed.
2. Stand the bay on a table with its front edge about 33 cm from the robot's base line.
3. Place three sealed 15.25 oz cans (67 mm diameter, 111 mm tall) upright at the back, their centres 230 mm behind the shelf's front edge, at y = -150, 0 and +150 mm. They mark the space the rod must clear.
4. Mark the guard line with two pencil ticks on each side wall: 90 mm in from the shelf's front edge and 120 mm above the shelf.
5. Set the KN633NP rod to 75 cm tip to tip (a tape mark on the tubes; the rod collapses to 71 cm), unlocked with the cam turned back to its free position, on the shelf near the front, its right cap by the right wall and its axis along y. Put a tape flag on the inner tube to read the turn.

**Reset between trials.** Take the rod off, turn the cam back until the inner tube slides freely, push the tubes together, and lay the rod in the start pose. Wipe the caps and the wall faces. Re-measure the wall gap if the bay has been moved. Stand the cans upright at their marks.

**Steps of one trial.**
1. Start the robot's run with the instruction. The run lasts 2,400 steps at 20 Hz (120 s).
2. The rod's start pose, the wall gap and the can marks are checked before the run, not during it.
3. At the end of the run, stop the arms and do the hands-off hold below.

**Scoring by hand.** Score the end state, in order:
- Void (0): the rod rests on the shelf or on the cans, or a cap face is off its wall. Look from above and the front; slide a 3 mm feeler between each cap face and its wall: if it enters, the cap is off. Also void: a cap face past the wall's front or back edge (the wall is 37 cm deep) or above its top (32 cm above the shelf), checked by eye and a tape against marks on the wall. A photo from the front and one from above records the end state.
- 0.33: the rod is off the shelf with both caps on the walls. Check by passing a ruler under the rod.
- 0.67: also the cam is locked. Read the turn of the inner tube on the tape flag with a protractor; it counts as locked at the cam's lock angle, measured beforehand (60 to 120 degrees on the real rod). Alternatively pull the tubes by hand: a locked rod keeps its length.
- 1.0: also level and at height, hands clear, and the push held. Put a digital inclinometer on the tube: within 3 degrees. Measure the rod's middle above the shelf with a tape: 120 +/- 15 mm. Check by eye and with a ruler that no part of either gripper (fingertips, pads, palm, housing) is within 30 mm of the rod, held for 5 s with a stopwatch. Then push down at mid-span with a push-pull gauge (a 0 to 50 N spring push-pull gauge, about $15, resolving 1 N) to 10 N, held 2 s, with a ruler or dial indicator under the rod's middle: sag under 5 mm, and the rod still on both walls.

**Forces and fits that make the act hard, and bench checks.** Each is a measured value in the Sizes table, with its accepted range:
- End-cap squeeze: the caps compress about 1 mm each when the rod is carried level (the demo's 0.85 mm at 25 N/mm is about 21 N a cap). Check a cap's stiffness by pressing it on a kitchen scale and reading the compression with calipers over the first 2 mm: accepted 20 to 40 N/mm. Check that the cap holds the rod: with the rod squeezed 1 mm each end, the 10 N push must not drop it.
- Twist-lock torque: the inner tube turns at 0.1 to 0.2 N m. Tape a 10 cm lever to the inner tube and pull its end with a luggage scale: 1 to 2 N. The outer tube must be held still against this torque.
- Cam hold: with the rod locked and the outer tube clamped, hang a luggage scale on the inner tube's end and pull along the axis until it slips: accepted 80 to 200 N. The turn to lock is read with the protractor: 60 to 120 degrees.
- Unlocked slide: 1 to 3 N on a luggage scale. Cap friction: tilt a painted board with a cap on it until it slides; accepted 0.5 to 1.0 (tan of the angle).
- Wall gap: 800 mm, tape. The rod's length when locked at the tilt is about 2 mm longer than the gap straight across; measure the locked length with a tape (the gap plus 1 to 3 mm) to check the setting.

## Real items

- Tension rod: Kenney Twist & Fit No Tools camlock tension rod, 28 to 48 in, 5/8 in, white, model KN633NP, about $8 (https://www.walmart.com/ip/750274656). https://www.amazon.com/dp/B07HBD352R
- Cupboard bay (bench-built, under about $60): 3/4 in (actual 0.728 in, 18.5 mm) birch plywood project panels, 2 ft x 4 ft, for the shelf, the two side walls and the back panel (for example Home Depot https://www.homedepot.com/p/206120918), cut to a 37 cm deep shelf, two 32 cm tall side walls set 80 cm apart (inner faces) and a back panel, and joined with wood screws and wood glue. The inner faces are sanded and left as painted or bare board. Any cabinet or RV pantry shelf with an 80 cm clear span would serve the same way.
- Cans: three sealed 15 oz food cans, for example Del Monte Golden Sweet Whole Kernel Corn, 15.25 oz (https://www.walmart.com/ip/10295217), set at the back of the shelf as bought and reused as they are; they only mark the space the rod must clear.

## Sources

- Kenney Twist & Fit rod listing (Amazon B07HBD352R): "Featuring a strong 5/8" diameter rod with a camlock tension mechanism"; "just extend the rod against both walls at an angle, twist to lock, and level into place"; 28 to 48 in; 8.64 oz. https://www.amazon.com/dp/B07HBD352R
- Kenney curtain rod buying guide: "Camlock Tension uses a locking mechanism to keep the rod at a static length." https://pdf.lowes.com/productdocuments/43e9f035-2ba8-4190-919f-81ca25564838/66314690.pdf
- RV Industry Association: "RV ownership is at a record high, with 11.2 million households owning an RV in 2021." https://www.rvia.org/news-insights/pandemic-pushed-people-outdoors-record-number-them-rvs
- Go RVing 2025 RV owner demographic profile (RV Industry Association): owners take about nine RV trips a year. https://www.rvia.org/2025-go-rving-rv-owner-demographic-profile
- RVing USA, tension rods in an RV: "By placing a short tension rod across the front of a shelf, you can help keep things contained." https://www.rvingusa.com/blog/clever-ways-to-use-tension-rods-in-your-rv

Kenney is named for identification only; the name is its owner's trademark.

## Licence

MIT
