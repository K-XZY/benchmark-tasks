# Close a wall belt barrier: draw the belt, pass its end hand to hand against the spring, and click it into the receiver

Belt barriers close off escalators, aisles and queues in stations, malls and airports. A Tensabarrier wall unit's belt is drawn out against its retractor spring and its metal end pushed into a receiver on the facing wall. A person usually walks the end across; a worker who closes it from one spot, as a fixed robot must, changes hands mid-pull: one hand draws the belt from the unit, the other takes the end and seats it. The spring never stops pulling, so the second hand must close on the end before the first lets go, or the belt runs back into the case. Here the unit and receiver are 90 cm apart: the receiver is beyond the reach of the arm at the unit, and the unit beyond the other arm. The instruction does not say to change hands, how far the belt may run back, or to square the end to the slot, so the task checks whether the robot keeps the end held through the pass and lines it up against the pull itself. Tensabarrier is a Tensator trademark, named for identification only.

**Instruction.** Close the belt barrier: pull the belt out and click its end into the wall receiver.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder.

## Run a trial

From this folder. One trial is at most 2,400 steps (2 minutes of simulated time at 20 Hz; the scripted demo ends at about 754 steps). Nothing is random: seed 1.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --seed 1 \
  --instruction "Close the belt barrier: pull the belt out and click its end into the wall receiver."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                     # full score, with a video
python demokit.py task.py OUT --no-record --mode none,right,letgo,nolatch     # the lazy routes below
```

The right hand comes to the front of the end housing where it rests against the unit, fingers level and turned 20 degrees toward the unit, jaws open 70 mm, and closes on its top and bottom edges (50 mm across) on the half away from the case. It draws the belt out along the wall to y = +5 cm (0.46 m out, about 10 N). The left hand, waiting above, comes down on the housing's other half and closes on its broad faces. Holding the housing where it is, the left takes the pull as the right opens and backs away. The left carries the housing along the wall to 2.5 cm short of the receiver, steering by the housing's pose at every step, turns the hand about the vertical to square the housing to the slot, leans its fingers 25 degrees toward the slot about the jaws' axis, and pushes the housing in until the tongue drops behind its ramp. It opens, lifts 8 cm and moves clear; the latch holds the belt's 15 N pull. The demo reads the housing's pose to steer; it never moves an object except through the grippers.

## The scene and what is simplified

Station: two YAM arms with parallel grippers at the near edge of the table. Tensabarrier 896-type wall unit: yellow case 100 mm deep, 115 mm long, 70 mm tall, 2 in (50 mm) belt, 13 ft. Belt end: a metal end housing 60 mm long, 50 mm tall, 12 mm thick, 0.12 kg, its front top edge ramped (3.5 mm over 6 mm) with a 6 mm notch behind the ramp, 3 mm deep. Metal wall receiver (Tensabarrier 898 type): a slot 14 mm wide, its mouth chamfered 4 mm; the housing enters 20 mm to a stop face, and a sprung latch tongue riding its top edge is lifted by the ramp and drops into the notch (8 N at its 3 mm lift; metal on metal friction 0.25, so the push past the tongue is about 8 N). Layout in the line frame (cm, table top z = 0; x forward from the arm bases, y to the robot's left): arm bases at y = -30.5 and +30.5; the wall is a vertical board, its face along x = 45, 80 cm wide and 50 cm tall; the belt runs along the wall in the plane x = 37, its centre at z = 18; the unit's mouth is at y = -44 and the receiver's mouth at y = +46, 90 cm apart. At the start the housing rests against the unit's mouth, the belt fully in. Spring: the maker does not publish it; the model pulls the end back along the belt with 4.8 N plus 12 N per metre out (6 N at 0.1 m, 12 N at 0.6 m), through a brake of 40 N s/m. A lab sets the same pull with a spring scale on the belt end at 0.1 m and 0.6 m out. Belt modelled as a taut line from the mouth to the housing, the housing free to move 10 mm across the belt and up or down and to turn 10 degrees about the vertical against the belt's pull; sag and twist of the webbing are not modelled. Both grippers start at the arms' ready pose. Start state, the same every trial: the housing against the unit's mouth, the belt fully in, the tongue down.

- Belt: a taut line of varying length from the mouth to the end housing; the housing rides on slide joints along it (10 mm play across and up or down, 10 degrees of turn), so it cannot fall. Retractor: a 4.8 N + 12 N/m spring with a 40 N s/m brake, assumed (the maker publishes none). Latch: a sprung tongue lifted by the housing's ramp (8 N), standing for the clip. Sag, twist and webbing slip are not modelled. The demo reads the housing's pose.
- Sources: Tensator, Tensabarrier 896 heavy-duty wall-mounted barrier: 2 in woven polyester belt, 7 ft 6 in or 13 ft, metal case, sold with one wall clip (https://www.tensator.com/shop/us/product/tensabarrier-heavy-duty-retractable-wall-mounted-barrier/). Tensator, Tensabarrier wall receiver belt clip, ref. 898: metal, fits the standard anti-tamper webbing end (https://www.tensator.com/shop/us/product/tensabarrier-wall-receiver-belt-clip). Tensator, Tensa Midi wall unit sheet: braking cassette, 50 mm webbing (https://d4sr6trxn2jcb.cloudfront.net/Tensa_Midi_Wall_Mounted_Belt_Barrier_Technical_Specification_XL_Displays.pdf). Lavi Industries Beltrac install sheet 87-065: pull the belt by its end across the aisle and fix the end. The maker publishes neither the retractor force nor the clip's latch force; both are assumptions, stated above. The case's depth and the end housing's size are not on the maker's pages and are taken as stated. All meshes are built in code; nothing is downloaded. Tensabarrier is a Tensator trademark, named for identification only.

## Building it for a real trial

Mount a 13 ft wall belt unit and its wall receiver on a board 90 cm apart, the belt at the height the arms reach. Check the retractor with a spring scale on the belt end: about 6 N at 0.1 m out and 12 N at 0.6 m (a hanging weight over a pulley can stand in). Check the receiver: the end must go in square and click; the tongue should need about 8 N to pass. Put tape on the wall every 10 cm from the unit, to read any run-back from the video.

## Scoring

Read from video with a tape on the wall, a spring scale and the latch's click. Score 1.0: the end housing latched in the receiver (in at least 18.5 mm of its 20 mm, the tongue down in the notch behind its ramp, the housing within 4 mm of the slot's line across and 5 mm up or down), held there by the latch alone against the belt's own pull with every fingertip of both hands at least 5 cm from the housing and the housing still, for 1 s; the belt out at least 0.5 m; and the belt's end never ran back toward the case by more than 0.10 m after it first passed 0.3 m out. 0.6: latched and held, but the belt ran back more than 0.10 m at some point. 0.4: pushed in 18.5 mm, square to the slot, not latched. 0.2: the belt held out 0.5 m by a hand (jaws shut on the housing), not seated. 0 otherwise. In the simulation the run-back, depth, tongue and clearances are read from the joints and fingertips.

## Results

Scripted demo: 1.0 on GPU physics in 4 of 4 runs (latched, belt out 0.86 m, run-back at the hand-over 3 mm). Dream-1: 0.0 (one valid run of 2,400 steps; the arms stayed near their bases and never reached the belt end).

## Failure table

Each lazy route, run in the simulator (`--mode`), must score no more than its cap:

| Route | Mode | Cap | Result |
|---|---|---|---|
| Do nothing | `none` | 0 | 0 |
| Right hand only: draws the belt to the middle and holds it there | `right` | 0.2 (cannot reach the receiver) | 0: held at 0.46 m out, short of 0.5 m |
| The left lets go of the end for half a second at the receiver | `letgo` | 0.6 (run back) | 0.2: the belt ran back 0.23 m |
| Pushed in 6 mm short of the latch, then let go | `nolatch` | 0.4 (not latched) | 0.2: the end pulled back out |

Nothing is random (object_jitter=0). Between real trials let the belt run fully back into the case.

## Licence

MIT
