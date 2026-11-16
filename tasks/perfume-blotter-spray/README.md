# Spray a perfume tester on a blotter strip held in its line, then lay the strip on the tray touching nothing else

A shop's fragrance counter: a glass counter top with a white acrylic display block, and on the block a 100 mL eau de parfum spray tester (Chanel N°5, cap off), its nozzle facing away from the robot. Five white blotter strips stand in a slotted acrylic stand, and a white melamine tray waits for the sprayed strip. One hand takes a strip and holds it upright, tip up, in front of the nozzle; the other turns the tester on its block until the nozzle points at the strip and presses the actuator from above. The sprayed strip is then laid on the tray. Only the effect is scored: the perfume's mark across the strip near its tip, landed while a gripper held the strip up in the air, the strip on the tray having touched nothing but the grippers and the tray since it left the stand, the other strips still standing, and the tester upright on its block.

In the demo the right hand pinches the strip nearest the robot 80 mm below its tip, its fingers pointing forward and 30 degrees down, and lifts it out of its slot. At the same time the left hand grips the tester across its 7.6 cm glass and turns it 90 degrees on the block, so the nozzle faces the strip's side. The right hand brings the strip's tip 15 cm in front of the nozzle, 1 cm above the spray's axis. The left hand closes its fingers over the actuator and presses straight down until it fires. The right hand then carries the strip to the tray, lowers its foot to just above the tray's floor, leans it 30 degrees and lets go so that it falls to lie along the tray, and both hands draw back.

**Why it matters.** Most people have tried a perfume at a shop's tester by spraying it on a paper blotter strip, and fragrance-counter staff do it for customers. The act is done on light, free objects: a thin card held still in a spray's line while the other hand turns and presses a bottle that stands loose on a small block, then a wet strip carried and put down without brushing anything.

**Ability tested:** holding a 0.5 mm card still in a spray's line with one hand while the other aims and presses a free-standing tester, then carrying the strip to a tray and laying it down so that it touches nothing but the grippers and the tray on the way.

**Nearest tasks.** Watering plants with a trigger spray bottle also aims and fires a spray, but one hand holds the bottle while the other squeezes its trigger, and the target is a fixed plant: no hand holds the target. Tasks that present work to a heat gun dose a sleeve along its length; here the press is one aimed shot at a thin card held in the air, and the card must then travel without contact.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Try the perfume on a blotter strip. Take one strip from the stand and hold it upright, tip up, about 15 cm in front of the tester's nozzle. With the other hand, turn the tester on its display block to aim the nozzle at the strip's tip and press the spray from above, so the perfume wets the strip across its full width within 4 cm of its tip. Then lay the strip on the tray and move both hands clear. The strip may touch only the grippers and the tray; leave the other strips standing in the stand and the tester on its block."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                   # full score, with a video
python demokit.py task.py OUT --no-record --no-cameras --mode one_arm,noaim  # the check runs below
python check_score.py                                                       # the scorer on hand-built states, no simulator
```

The demo reads the poses of the strip and the tester to plan its grasps and to put the strip's tip in the spray's line. It reads no contact force and never moves an object except through the grippers.

`check_score.py` feeds 41 hand-built runs through the scorer itself: each rung, each rule with a case that fails it, the zeros, and the routes that are allowed because only the effect is scored. Among them: a strip sprayed while it stands in the stand, lies on the tray, rests across the tray's rims or lies on the display block, then laid on the tray (0, gripped or not: the mark counts only from spray that landed while a gripper held the strip up in the air and the tester had been turned at least 45 degrees on its block from its start (nozzle facing +x)), and the spray reaching 30 cm and wetting the standing strips behind the held one (1.0: not scored).

## The scene and what is simplified

- **Counter:** a 40 x 60 cm glass top, 1 cm thick, on the table, centred 35 cm in front of the midpoint between the arm bases.
- **Display block:** white acrylic, 10 x 10 cm and 5 cm high, 8 cm left of centre.
- **Tester:** a Chanel N°5 Eau de Parfum 100 mL spray bottle at its retail size: glass 7.6 x 3.8 cm and 7.4 cm high, 281 g. The listing gives 10.16 cm overall with the cap on; with the cap off the model is 9.8 cm to the top of its 2 cm actuator (an estimate; the task does not depend on it). It stands loose on the block, so a press off its axis or a knock tips it. Its nozzle is 14 cm above the counter and starts facing away from the robot.
- **Actuator:** a sprung slide with a 4 mm stroke and 10 N at full stroke. It fires one spray each time it is pressed past 90% of its stroke and re-arms when it springs back. Stroke and force are simulation settings; a real trial uses the real tester.
- **Spray:** a 30 degree cone of 2,000 straight droplet rays from the nozzle, reaching 22 cm, stopped by the first strip or fingertip they meet. After the turn the nozzle points toward the stand, 24 cm away; a real spray carries further and may wet the standing strips behind the held one, which is not scored (a check case raises the reach to 30 cm: still 1.0). Where a strip is hit, its card darkens in cells 4 mm wide and 6.5 mm long (3 across the strip, 20 along it); a cell is dark once 2 droplets land on it. The perfume soaks through, so the mark shows on both faces.
- **Blotter strips:** five white card strips, 130 x 12 mm and 0.5 mm thick, 0.5 g each, standing tip up in a clear acrylic stand (18 x 3 cm, 2.5 cm high) in slots 2 cm deep and 3.5 cm apart. Each strip is a rigid card that does not bend.
- **Tray:** white melamine, 12 x 18 cm, with a 4 mm floor and a 1 cm rim, at the robot's right.
- **Contact:** read from geometry. Points every 5 mm over the strip's face are checked against the counter outside the tray, the stand, the block, the tester's glass and actuator, and the other strips; a point within 2 mm of one of them is a touch. Touches count from the moment the strip is 1 cm clear of the stand. The strip is held up in the air when a gripper's jaws are shut on it (fingertip points of both fingers on its two faces, or on its two long edges) and all of it is 1 cm clear of the stand, with its lowest point more than 1.5 cm above the counter (above the tray's 1 cm rims); only spray that lands on it then counts toward the mark. The hands' 3 cm clearance is measured to lines along the strip and up the tester's corners and the middle of each glass face.
- Physics: rigid bodies at 240 Hz on the GPU. Friction (static/dynamic): card 0.6/0.5, glass 0.5/0.4, counter, block, stand and tray 0.5/0.4.

## Building it for a real trial

Lay a glass counter top (or any smooth white surface) on the table, and stand a 10 x 10 x 5 cm white acrylic block on it. Stand a Chanel N°5 Eau de Parfum 100 mL tester on the block with its cap off. Cut a stand from clear acrylic, 18 x 3 x 2.5 cm, with five slots 16 x 1.6 mm and 2 cm deep, 3.5 cm apart, and stand five fresh 130 x 12 mm perfumer's blotter strips in it. Put a 12 x 18 cm melamine tray with a 1 cm rim where the setup says. Film with the station's top and wrist cameras and one more camera at counter height, looking along the counter past the stand, the block and the tray, to see any brush against them. A light box or a lamp behind the strips makes the wet mark easy to read: the perfume turns the card darker and translucent where it lands, for some seconds. Perfume is alcohol-based and flammable: no flame or spark near the station, a ventilated room, and nobody in front of the nozzle.

## Scoring

Scored by effect only: how the strip is held, its distance from the nozzle, the number of presses and the order of steps are not scored. Spray that wets the other strips or the tray is not scored. 0, latched, if the tester falls over or comes down off its block onto the counter or the floor (lifting it and setting it back is allowed), or any strip reaches the floor. Otherwise, at the end of the trial:

| Score | Condition |
|---|---|
| 0.5 | one strip carries the spray's mark across its full 12 mm width within 40 mm of its tip, from spray that landed while a gripper held the strip up in the air and the tester had been turned at least 45 degrees on its block from its start (nozzle facing +x) (jaws shut on it, 1 cm clear of the stand, its lowest point more than 1.5 cm above the counter) |
| 0.75 | and that strip lies on the tray floor, inside its rim and out of the grippers |
| 1.0 | and from the moment it left the stand the strip touched nothing but the grippers and the tray (not the counter, the stand, the block, the tester or its nozzle, or another strip); the other four strips never left their slots (or tipped past 20 degrees); the tester stands upright on its block (tilted under 10 degrees); both grippers are at least 3 cm from the strip and the tester's faces; and the strips and the tester are still (under 1 mm over 1 s, held 1 s) |

Real trial: the mark with a ruler from the tip, on a photo taken against a light within 5 s of the press; the strip in the gripper and where it was at the press (in the air, out of the stand), on the wrist, top and side cameras; the strip on the tray, the other strips in their slots and the tester on its block on the top camera's end frame and a photo, with a square against the tester's side for its tilt and a ruler for the 3 cm clearance; any touch, frame by frame on the top camera and the counter-height side camera; stillness on two photos 1 s apart.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps | 0 (no spray fired, no strip lifted) |
| `full` | The whole task (the film) | 1.0 |
| `one_arm` | The left hand alone turns the tester and presses it; the right never takes a strip, so the spray goes into the air | 0.0 |
| `noaim` | The strip is held where the nozzle would point after the turn, but the tester is never turned, so the spray misses; the unmarked strip is then laid on the tray | 0.0 |

## Runtime and seeds

The demo takes 456 steps, 22.8 s of simulated time at 20 Hz; it ends when the 1 s end hold is met. There is no randomness: `object_jitter=0`, so no seed is needed.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Tester: Chanel N°5 Eau de Parfum 100 mL spray, item dimensions 10.16 x 3.81 x 7.62 cm and 280.66 g (retail listing, https://www.desertcart.com.ar/products/48578403).
- Blotter strips: 130 x 12 mm perfume testing strips (retail listing, https://www.desertcart.lk/products/92132263).
- Chanel and N°5 are named here for identification only; the names are their owner's trademarks.

## Licence

MIT
