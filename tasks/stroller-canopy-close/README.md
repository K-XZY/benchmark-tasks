# Draw a baby carrier's cloth canopy forward to its stop over the doll's face, steadying the rocking carrier by its handle

A Graco SnugRide 35 Lite LX infant car-seat carrier stands on the bench with a baby doll lying in it, its open front facing the robot. Its cloth canopy, sewn over three hinged bows, is folded back over the doll's head. A carer shades the baby by drawing the canopy's front edge forward until the canopy stops; Graco lists the canopy as multi-position (https://www.gracobaby.com/shop/car-seats/infant-car-seats/snugride-35-lite-lx-infant-car-seat/SAP_2110187.html) and Target the carrier at 27.4 x 15.5 x 18.07 in (https://www.target.com/p/graco-snugride-35-lite-lx-infant-car-seat/-/A-79436026). The carrier's base is curved, so the pull rocks it unless the other hand holds the carry handle, which is turned back behind the head end. The hard part is the draw: the front bow is swung through its arc while the fabric unfolds and drags the other bows after it, and the carrier must not rock while it does. Graco and SnugRide are named for identification only.

Unlike the accepted laptop-lid lift or RoboCasa's CloseSingleDoor, the base here is not fixed: the carrier is free to rock on its curved base and must be held level throughout the draw, and the work is done over a doll's face.

**Instruction.** Shade the baby: with your left hand hold the carrier steady by its carry handle, and with your right hand take the canopy's front bow and draw it forward until it stops, its front edge over the doll's face, without the fabric touching the face.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. The canopy is an Isaac surface deformable, so run on a GPU.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
  --instruction "Shade the baby: with your left hand hold the carrier steady by its carry handle, and with your right hand take the canopy's front bow and draw it forward until it stops, its front edge over the doll's face, without the fabric touching the face."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                              # full score, with a video
python demokit.py task.py OUT --no-record --mode noleft                # a lazy route below; run each (none, noleft, noleft_release) in its own session
```

The left hand comes down on the handle's left side arm near its grip bar, tilted 45 degrees and leaning out from the shell, closes on it at 35 N, then reads the carrier's rock and raises or lowers the handle until the carrier is level. The right hand comes level from the open front onto the canopy's front bow, 25 degrees round the arch from its top, and closes on the tube. It then swings the bow forward about the canopy's hinge in one smooth draw (about 7 degrees a second) until the bow clicks into its stop, turning the hand through 0.3 of the bow's swing while the round tube rolls in the pads. Throughout the draw the left hand keeps reading the rock and lifting or lowering the handle to hold the carrier level against the pull. The demo reads only the carrier's rock and the bows' angles; it never moves an object except through the grippers.

## The scene and what is simplified

Station: two YAM arms with parallel grippers, bases 61 cm apart at the near edge of a 120 x 90 cm wood table, 75 cm high. Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left; heights above the table top. Carrier: a Graco SnugRide 35 Lite LX infant car-seat carrier at its listed 69.6 x 39.4 x 45.9 cm (Target's 27.4 x 15.5 x 18.07 in, read as length, width and height to the canopy's top): shell 69.6 cm long and 39.4 cm wide, its rim 28 cm high, on a curved rocker base of 42 cm radius; carrier 3.6 kg and doll 1 kg. It stands with its middle at x 16, y 0, its long axis along x, its head end away from the robot (x 51) and its open front toward it. It rocks fore and aft on its base. Carry handle: turned back behind the head end, its side arms (2 cm square) at y 24.7 and y -24.7, 20 cm high, from x 34 to x 60, joined by a grip bar at x 60. Canopy: three bows of 16 mm tube on one hinge axis across the shell at x 38, 28 cm high; each bow is an arch 39.4 cm wide and 17.9 cm high over the axis. At the start the rear, middle and front bows lean 60, 30 and 0 degrees back from upright (the front bow upright, its top at x 38, 45.9 cm high); the front bow stops at 45 degrees forward of upright, where it clicks into place; this fully drawn position, the last the canopy reaches, is what counts as its stop (assumed at 45 degrees and unsourced: Graco lists a multi-position canopy but gives no angle; a real trial uses the carrier's own fully open click). The canopy pulls its front bow back toward closed, about 1.2 N m at the stop, so the draw takes a firm, steady pull. Fabric: polyester canopy cloth, pleated between the bows and sewn to them. Doll: a soft-bodied vinyl baby doll, 45 cm long and 1 kg, lying on its back, head toward the canopy, head 14 cm across centred at x 30, 22.5 cm high; the face zone is a 12 x 14 cm patch over the face, from x 23 to x 37 and y -6 to y 6, about 27 cm high. The line for the canopy's front edge is set from the doll's face: x 26.5, 3.5 cm past the centre of the face toward the doll's feet (11.5 cm nearer the robot than the hinge). Both grippers start at the arms' ready pose. Start state, the same every trial: canopy folded back, carrier level and still.

- Carrier at Graco's listed size; doll 1 kg. The canopy is cloth: a sewn surface-FEM sheet pleated between three rigid hinged bows (0.6 mm, 160 g/m2, 2 MPa assumed). The pull back on the front bow is a hinge torque in code (1.5 N m/rad), heavier than a real canopy's fabric so the carrier needs steadying; the stop's click is a latch. Rocker: a hinge at its bench contact, m g (R - h) = 11.3 N m/rad. The right hand holds the front bow by friction alone, 25 N per finger.
- Sources: Graco SnugRide 35 Lite LX product page (https://www.gracobaby.com/shop/car-seats/infant-car-seats/snugride-35-lite-lx-infant-car-seat/SAP_2110187.html: multi-position canopy, machine-washable canopy fabric); Target listing (https://www.target.com/p/graco-snugride-35-lite-lx-infant-car-seat/-/A-79436026: overall 27.4 x 15.5 x 18.07 in); neither page gives the canopy's positions or angles. Carrier mass 3.6 kg lies within the listings' 3.3 to 6.5 kg. The canopy fabric's thickness, weight and stiffness and the 45 degree stop are assumed, with the ranges given above. All meshes are built in code; nothing is downloaded.

## Scoring

Read from video, with a tape for the front edge's line and a phone level on the shell for the rock: a level app that logs its maximum, or the level's screen kept in the trial video. Score 0 if the canopy's fabric touches the doll's face or a hand touches the doll (in the simulation: any fabric within 1 cm of the face zone, or any fingertip within 1 cm of the doll, at any time), or if the carrier rocks more than 6 degrees. The carrier counts as unsteadied if it rocks 3 degrees or more at any time, or is 1 degree or more off level at the end: an unsteadied run gets no credit for the canopy and scores at most 0.25. Otherwise: 1.0 when the front bow is at its stop (within 2 degrees) with the front edge past the line and the carrier under 1 degree, held for 1 s; 0.5 when the bow reached its stop past the line but was not held there steady for 1 s; 0.25 when the front edge was drawn at least 5 cm forward; 0 otherwise. With no steadying hand the same draw scores 0.25 in the simulator: the carrier tilts 2.5 degrees and stays tilted, and the bow stops short of its stop.

## Results

Scripted demo (`--mode full`): 1.0. The right hand holds the bow by friction alone. The front bow reaches its 45 degree stop (43.8 degrees logged when the run ends on success), the front edge is drawn 12.6 cm, the fabric stays 108 mm from the face zone, and the carrier's rock peaks at 1.7 degrees and ends at 0.1. Dream-1, one run: 0; it did not draw the canopy (the front bow stayed near closed) and the carrier's rock peaked at 1.9 degrees.

## Failure table

Each lazy route, run in the simulator (`--mode`), must score no more than its cap:

| Route | Mode | Cap | Result |
|---|---|---|---|
| Do nothing | `none` | 0 | 0 |
| Draw the canopy with the right hand only, no steadying hand | `noleft` | 0.25 | 0.25: the carrier tilted 2.5 degrees and stayed there, and the bow stopped at 38 degrees, short of its stop |
| Draw with the right hand only to the click, then let go, no steadying hand | `noleft_release` | 0.25 | 0.25: one hand could not reach the click (the carrier tilted 2.6 degrees and the bow stopped at 38); once let go, the canopy sprang back to 3 degrees and the carrier was left 1.3 degrees off level |

Nothing is random (object_jitter=0). Between real trials fold the canopy back with the front bow upright and level the carrier.

## Licence

MIT
