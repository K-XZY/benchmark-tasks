# Turn a loose Asimov humanoid leg over on the table, then seat its knee side plate flush in the pocket

Menlo's Asimov 1 is an open-source humanoid built from published parts (CERN-OHL-S-2.0). Left Leg Step 8 says to turn the leg so the thigh actuator's bearing faces up, then to fit the knee's outer side plate 600_08A on housing 600_10C so its protrusion enters the bearing and its holes meet the inserts. Here the leg, off the body, lies loose on its side, plate side away. At 3.03 kg it is twice the arm's 1.5 kg payload, so it is tipped toward the robot over its lower edge onto its back, not lifted; its knee bends freely against the knee actuator's own friction. Then three close fits meet on one axis: a 5 mm plate in a 5 mm pocket (0.3 mm a side), a 34.8 mm pad in a 35.0 mm bore, an 11.85 mm boss in a 12.0 mm bearing. The fingers can't follow the plate into the pocket, so it is let go just above and pressed home; tilted, it cocks. On its back the leg rests on three points: pressed over the boss it stays put, pressed near the plate's edge it rocks. The nine M3 bolts are left out.

**Instruction.** Turn the leg over on the table until the knee's bearing faces up, then fit the knee side plate: seat it flat in its pocket on the knee housing with its boss in the thigh's bearing and all nine holes over their inserts, leave the leg on its back, and move both hands clear.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `knee_plate.npz` (the maker's geometry as meshes and colliders) are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --seed 1 \
  --instruction "Turn the leg over on the table until the knee's bearing faces up, then fit the knee side plate: seat it flat in its pocket on the knee housing with its boss in the thigh's bearing and all nine holes over their inserts, leave the leg on its back, and move both hands clear."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full   # full score, with a video
python check_score.py                       # the scorer on hand-built states, no simulator
```

The demo reads the leg's and the plate's poses to plan its grasps and its moves; it reads no contact force and moves no object except through the grippers. Both hands grip the leg's ends from above (the calf's end and the thigh's top end, across their thickness) and swing together about the edge the leg rests on, toward the robot, to 66 of its 86 degrees: past its balance point at 38, as far as the wrists follow. They let go and it settles onto its back. Then the right hand pinches the plate across its neck, lifts it out of the post, squares it to the face as the leg lies, lets go 1.5 mm above the face with its axis on the bearing's, presses it home over the boss if it has not slid home, and both hands clear. `check_score.py` feeds hand-built runs through the scorer itself: each rung, the leg's poses (on its side, half turned, held up, face down, on its end, off the table, sliding or rocking under the plate), the zeros, the hands and stillness gates, and the lazy routes.

## The scene and what is simplified

Station: two YAM arms with parallel grippers, bases 61 cm apart at the near edge of the table. Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left; heights above the table top. All parts are from Menlo's Asimov 1 repository (github.com/menloresearch/asimov-1, CERN-OHL-S-2.0): shapes from the maker's STEP files (the thigh from the maker's URDF mesh). Leg, as after the guide's instruction 3, off the body and loose on the table, nothing holding it: the thigh assembly (URDF left_hip_yaw_link: 1.949 kg, with its centre of mass and inertia) slotted into the knee housing 600_10C (MJF PA12, 1,069.5 cm3 at 1.01 g/cm3: 1.080 kg; the URDF's 2.48 kg knee link includes the calf's later parts), 3.03 kg in all and 54 cm long. Its knee joint is free: the housing turns on the thigh about the bearing's axis, from straight to 1.50 rad bent (URDF), against the unpowered knee actuator's published friction, 0.70 N m to start it and 0.02 N m once it moves, with 0.034 kg m2 reflected inertia (Menlo's actuator table, EC-A4315-P2-36, docs.menlo.ai/asimov/1/reference/joint-design-and-actuation). It starts straight, lying on its side along y with the shank toward -y, the housing's outer face (the plate side) upright and facing away from the robot (+x), the pocket's centre at (46, -7), 6.3 cm up. On its back, the task's turned pose, it rests on three points with that face up, about 4 degrees off level, the pocket's centre at about (31, -6), 10.3 cm up: from its side it tips 86 degrees toward the robot over its lower edge, its centre of mass rising from 6.2 to 7.9 cm (over the edge at 38 degrees) and falling to 5.3. In the outer face, the pocket for the plate: 5.0 mm deep, the plate's own outline with 0.3 mm clearance (80.0 mm across its round end, 40.0 mm across its neck, its narrow end toward the shank), with a 35.0 mm bore at its centre and nine M3 threaded inserts. Its walls stand full depth round the round end and the neck's root and fall away along the neck, where the housing's surface curves down. The thigh's flanged bearing 600_11X (12.0 mm bore, 5.0 mm wide) is on the pocket's axis, its face 11 mm below the housing's face. Plate 600_08A: aluminium 7075, a 5.0 mm sheet 149.4 mm long, 79.4 mm across its round end and 39.4 mm across its neck, 1 mm chamfers on its edges; on one face a 34.8 mm pad (5 mm), a 15 mm step (1 mm) and an 11.85 mm boss (5 mm, 0.9 mm tip chamfer); nine 3.2 mm countersunk holes; 120 g (42.6 cm3 at 2.81 g/cm3). It starts boss down in the socket (36 mm across, 12 mm deep) of an acetal kitting post 50 mm across and 30 mm tall at (20, -17), its sheet on the post's rim, its narrow end toward -y. Friction 0.3 (PA12 and aluminium on the table, on acetal and on each other). The plate rests on a holder on the bench (the post). Both arms start at their ready pose. Low-cost stand-in for a real trial (costly: the 7075 plate and the actuated leg; nothing here needs the real robot): print the thigh and the knee housing 600_10C from the published STEP files (PA12 if you have a service, PLA otherwise; solid prints are heavier than MJF PA12 at 1.01 g/cm3, PLA being about 1.24 g/cm3, so weigh them). Ballast: the leg must weigh 3.03 kg with its centre of mass where the URDF puts it. Weigh the printed pair, then add the missing mass as steel or lead-free shot or bar glued into two pockets, one in the thigh and one in the housing, sized so the sum of mass times distance matches the URDF thigh (1.949 kg) and housing (1.080 kg) centres. Check by balancing the leg on a knife edge and by the tip test: tipped over its lower edge it passes its balance point at about 38 degrees. Knee: keep the thigh's bearing seat to the STEP size (12.0 mm bore, 23.0 mm flange, 5.0 mm wide) and fit the real flanged bearing 600_11X or a plain 12 mm bearing in it, with a 12 mm steel pin as the hinge, 0 to 1.50 rad. Set the friction by tightening a friction screw or strap on the pin until a spring scale at 0.30 m from the axis reads 2.3 N to start it (0.70 N m) and read what it moves at (the target is 0.02 N m, 0.067 N at 0.30 m; the moving value is the hard one: report what you measure). Side plate: cut 5 mm aluminium sheet to the STEP outline (laser or waterjet from the STEP's drawing), with the 34.8 mm pad and 11.85 mm boss as a turned part fixed to it, or print the one-piece plate in PA12 or PLA with the same outline (a printed plate is lighter, about 53 g solid in PLA against 120 g). Print the pad, the boss and the pocket's outline undersize or oversize as the fit needs, then sand, ream or turn to the STEP sizes and check with the real bearing, a caliper and gauge pins: the fits are 0.3, 0.1 and 0.075 mm a side. Keep the pocket 5.0 mm deep, the 35.0 mm bore and the nine M3 threaded inserts (brass heat-set) at the STEP positions. Plate holder: a printed block (PETG or PLA) 50 mm across and 30 mm tall with the 36 mm, 12 mm deep socket, in place of the acetal post. No cost is given here: no published price was found to cite.

- Physics: Rigid bodies at 240 Hz. Plate (45 convex colliders), pocket (217) and bearing (48) are the maker's STEP geometry; no collider widened: rim 0.30 to 0.34 mm a side, pad 0.10 to 0.12, boss 0.075 to 0.09. The leg's outside is coarse convex slabs of its meshes. Friction 0.3. The knee's dry friction (0.70 start, 0.02 moving N m) is a coded drive; inertia 0.034 kg m2. The thigh sits in the housing as in the CAD. The arm presses through soft joints, about 10 N.
- The leg on its back, pressed through the plate (gate runs): 10 N and 20 N over the boss for 2 s move it 0.00 mm; 10 N at the plate's edge 35 mm off its axis on the far side moves it 0.00 mm, and on the near side rocks it about 2 degrees onto its other three points (the face 4 mm over), until the press stops. Left alone on its side for 5 s it does not move.

## Scoring

Measured at the end with an inclinometer and a ruler on the housing's face, a depth gauge on the plate's top against the face at ten points round its outline (round end, fillets, neck, narrow end), a 2.4 mm pin gauge and a tape. Score 0 if the plate or the leg falls off the table. Rungs are cumulative: 0.25 with the leg on its back on the table (the face within 10 degrees of level, at its resting height within 12 mm: lying there, not held up); 0.5 with that and the plate in the pocket's mouth, its boss's axis within 1.5 mm of the bearing's and no point of its top more than 4.0 mm above the face (the rim, pad and boss each at least 1 mm in); 0.75 with the plate seated flat, every point within 0.5 mm of flush and level with the face (tilt under 1 degree, boss down); 1.0 with that, the pin gauge passing through each of the nine 3.2 mm holes into its insert's thread (axis within 0.45 mm: 0.4 through the hole, 0.05 into the 2.5 mm minor diameter), no gripper within 10 cm of the plate and the face round it or within 8 cm of the leg's centre line (thigh's end, knee, calf's end), and the plate and leg still (plate under 1 mm in 1 s; leg under 2 mm/s and 0.6 deg/s), held for 1 s and still true at the end. A plate set in the pocket before the leg was on its back, pressed home with the leg moved 15 mm, or its face turned 3 degrees, from how it lay when the plate first reached the mouth, or never carried by a gripper (10 steps in a row 1 cm up with a fingertip within 3 cm), scores at most 0.5. No hold is needed during the press: the leg stays put on its own, so no hand should hold it. Real trial: the same gauges, tape marks round the leg, and a photo.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps | 0.0: the leg never moved |
| `full` | The whole task (the film) | 1.0 |

## Runtime and seeds

The demo takes about 1,115 steps (56 s at 20 Hz): the turn, then the plate. The 2,400-step limit leaves room for a slower policy. Nothing is random (`object_jitter=0`).

## Sources

Menlo Asimov 1 assembly guide, Left Leg Step 8, instructions 3 to 5 (read 11-16): https://docs.menlo.ai/asimov/1/build/assembly-steps/left-leg-module/knee-and-calf-assembly/step-8. Menlo Asimov 1 joint design and actuation, the full actuator table (left_knee_joint, EC-A4315-P2-36: range, armature, static and dynamic friction; read 11-16): https://docs.menlo.ai/asimov/1/reference/joint-design-and-actuation. Part STEP files and URDF, Menlo Research, CERN-OHL-S-2.0: https://github.com/menloresearch/asimov-1 (mechanical/ASV1/600/FABRICATION/ALU_7075/ASV1_600_08A.STEP, MJF_PA12/ASV1_600_10C.STEP, OFF_THE_SHELF/ASV1_600_11X.STEP; sim-model/asimov_1_25dof/urdf/asimov_1_25dof.urdf). Densities: aluminium 7075 2.81 g/cm3; MJF PA12 1.01 g/cm3 (HP's datasheet for PA12 parts).

The Asimov 1 part files (plate 600_08A, housing 600_10C, bearing 600_11X, the thigh's URDF mesh) are Menlo Research's, under CERN-OHL-S-2.0; `knee_plate.npz` is derived from them.

Menlo Research and Asimov are named for identification only; the names are their owner's trademarks.

## Licence

MIT
