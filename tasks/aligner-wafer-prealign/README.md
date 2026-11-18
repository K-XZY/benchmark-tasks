# Seat a 100 mm wafer on a mask aligner's chuck: flat on two pins, rim on the third, then slide it in

Contact mask aligners (SUSS MA6, MA8, MJB4) print chip patterns in university nanofabs and packaging fabs. Every exposure starts by hand: 'Place the flat of the wafer against the two pins at the bottom and move the wafer against the last pin on the left' (UCSB MA6 SOP); 'Be sure not to load on top of the pins!' (EPFL MJB4 manual). Then the vacuum goes on and the slide is pushed in. Here the wafer lies loose on the chuck, slightly off and turned, and is pushed by its rim alone, with a closed fingertip, until it sits on three raised pins: an off-centre push turns it flat onto both close pins, then it slides along them to the third. This differs from carrying a wafer with tweezers and orienting it by its flat on a mounter: nothing is carried here; the rim is pushed by fingertips against two flat pins and an edge pin, then the chuck is slid in.

**Instruction.** The wafer has been set down on the mask aligner's chuck. Seat it against the pre-alignment pins: push it only by its edge until its flat rests against the two close pins at the front and its rim against the pin on the left, without pushing it up onto a pin and without touching its top face more than 3 mm in from the rim. Then press the vacuum key, push the transport slide in by its handle to its end stop, and move both hands clear.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
  --instruction "The wafer has been set down on the mask aligner's chuck. Seat it against the pre-alignment pins: push it only by its edge until its flat rests against the two close pins at the front and its rim against the pin on the left, without pushing it up onto a pin and without touching its top face more than 3 mm in from the rim. Then press the vacuum key, push the transport slide in by its handle to its end stop, and move both hands clear."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                      # full score, with a video
python demokit.py task.py OUT --no-record --mode slidefirst,nokey,centre,face  # the check runs below
python check_score.py                                                          # the scorer on hand-built states, no simulator
```

The left hand's closed fingertips ride on the chuck's face. A first push toward the robot, from the far side and well left of the wafer's centre, turns the wafer onto both close pins; a second push, along a line turned 15 degrees toward the robot, slides it left along the flat to the third pin; a short third push (repeated if needed) brings the flat back flat on both pins. The hand then presses the vacuum key, pushes the slide in by its handle to the end stop and moves clear.

## Setup

Station: two YAM arms with parallel grippers, bases 61 cm apart at the near edge of a 120 x 90 cm table. Positions in cm from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table. Aligner base: a fixed block 30 x 25 x 5 cm centred at (50, 0). The transport slide, a 16 x 14 x 1 cm aluminium plate (top 6.0 cm up, 1 kg with its chuck), rides on two rails along x and is pulled out toward the robot, spanning x 22 to 38; it runs 15 cm in (+x) to an end stop with 3 N of sliding friction. A 8 x 1.2 x 1.2 cm handle bar stands at its near edge at x 21. On it a 120 mm round aluminium vacuum chuck, 10 mm thick, centred at (30, 0), face 7.0 cm up. Three 2 mm steel pins stand 0.4 mm proud of its face (screw-set on real chucks; UCSB MA-6 SOP Rev D): the two close pins' centres at (-48.3, -10) and (-48.3, +10) mm from the chuck's centre, so their faces lie on x = -47.3 where a seated wafer's flat lies; the left pin's centre at (0, +51.0), its face at +50.0. Vacuum key: a push key on the base at (40, +10), 4 mm travel, 3 N. Wafer: silicon, 100.0 mm, 0.525 mm thick, primary flat 32.5 mm (SEMI M1), 9.6 g, lying loose on the chuck (friction 0.3), flat toward the robot, its centre 4 mm farther from the robot and 3 mm to the right of seated and turned 8 degrees clockwise seen from above: its flat is 2.5 mm or more from the close pins and its rim 3 mm from the left pin, so it starts on no pin. Real trial at low cost, no aligner needed: a 300 mm ball-bearing drawer-slide pair with an end stop under a 160 x 140 x 10 mm aluminium plate, a printed handle, on plywood; the chuck is a 120 mm 6061 aluminium disc, 10 mm thick, with three 2 mm ISO 2338 h8 dowel pins in reamed holes set 0.4 mm proud with a feeler gauge (drawing: standin/chuck.svg), a 1 mm ring groove and a 3 mm hole to a hose barb on a small diaphragm pump, switched by a push switch. Wafers: test-grade (dummy) 100 mm silicon wafers, SEMI M1, 525 um, one primary flat. Plate, chuck and handle are machined or printed to the sizes above (no catalogue part needed; any 100 mm, 525 um test-grade wafers). A printed gauge sets the start pose. Lift and tilt in the real trial need no camera and no optics (about $30 of tools): a feeler-gauge set (0.05 to 1 mm, about $15) and a vacuum gauge or pressure sensor on the pump line (about $10). A person reads them after the key is pressed and again with the slide in; the simulator checks lift and tilt at every step. The ride-up is not filmed: a wafer that was pushed up onto a pin is still on it when the vacuum comes on, which the gauge and feeler show. Real-trial safety: the gripper fingers wear protective fingertip covers (a nitrile or silicone sleeve 0.3 mm thick, its tip height over the chuck face set and checked with a feeler before each trial, so the silicon's edge is touched by a soft tip and a chipped rim is not scratched further), and a shallow containment tray with a raised lip, under and around the chuck and slide, catches any wafer fragments; a cracked or chipped wafer scores 0, the trial stops, and the shards are cleared with the tray before the next one. Keep 10 spare dummy wafers (cheap, test-grade): two chipped or cracked wafers in one session stop the session. A bare gripper fingertip is used on the rim because the aligner's pins are 0.4 mm proud and tweezers cannot push a loose wafer square; the sleeve makes it a soft tip.

## Scoring

Read by a person with simple tools: a 0.3 mm feeler gauge at each pin at the end, a 0.1 mm feeler under the wafer's rim at the three pins and a vacuum gauge, a photo from above, a ruler on the slide, and one time-stamped overhead video kept for the whole trial (fingertip over the wafer's face, slide, hands clear). Lift and tilt are not read optically: with the vacuum on, a wafer that is lifted or tilted on a pin leaves the groove open to air, so the gauge does not reach its steady reading (it must hold within a quarter of its value while the slide moves) and the 0.1 mm feeler passes under the rim; either means the wafer rode up (score 0). A ride-up that settled flat before the vacuum is not seen in the real trial; the simulator, which checks the wafer's pose at every step through the push and the slide (0.25 mm lift or 1 degree tilt), covers it. Vacuum on is read from the pump's gauge (or the wafer resisting a light tap). Score 0, and it stays 0 (video), if at any time a fingertip touches the wafer's top face more than 3 mm in from its rim (above video), the wafer rides up onto a pin (its centre 0.25 mm or more above the face, or tilted over 1 degree, by the gauge and 0.1 mm feeler in the real trial), leaves the chuck or is flicked faster than 0.1 m/s, the wafer cracks or chips, or the slide moves more than 5 mm before the vacuum is on. Otherwise at the end: 0.25 if the flat touches one close pin (photo from above; the feeler does not pass); 0.5 if it touches both close pins, lying flat on the chuck; 0.75 if its rim also touches the left pin and the vacuum is on (the key counts only once the wafer is seated, and must be released); 1.0 if then the slide, pushed by a fingertip at its handle, is at its end stop (within 1 mm), the feeler still does not pass at any pin with the slide in, every fingertip is 5 cm from the wafer and the handle (video), and all is still for 1 s (video).

## Fidelity

PhysX, 240 Hz. The wafer is a rigid disc of 1.5 mm box strips. The vacuum is a 30 N downward force on the wafer once the key is pressed; no air flow. Pins are 2 mm cylinders 0.4 mm proud. Lift (0.25 mm) and tilt (1 degree) are read from the wafer's pose; the real trial reads them with a gauge and feelers. The slide's 3 N friction and the key's 3 N are settings. The face-touch check reads the finger ends' centre against the wafer's outline. The aligner body, mask and optics are not modelled.

## Dream-1

One run on 2026-11-17 (1,800 steps, seed 1, no session loss): score 0. The model did not seat the wafer. The scripted demo scores 1.0.

## Failure table

| Route | Result |
|---|---|
| `centre`: first push at the wafer's centre line | 0 (GPU run: never reaches the seated rungs) |
| `face`: a fingertip pressed on the wafer's face and dragged | 0 (GPU run) |
| `slidefirst`: slide pushed in before the key | 0 (GPU run) |
| `nokey`: slide pushed in, key never pressed | 0 (GPU run) |
| Arms do nothing | 0 |
| Flat turned away, round rim on the close pins | 0.25 or less (the flat must face the pins within 1 degree; check_score.py) |
| Rim on a pin, edge 0.15 mm up, or tilted 0.4 degree | below 0.5 (check_score.py) |
| One blade point on the top face | 0 (check_score.py) |
| Key pressed before the wafer is seated | does not count (check_score.py) |
| Slide moved without a fingertip at the handle | at most 0.75 (check_score.py) |
| Key held down at the end | hands not clear, at most 0.75 (check_score.py) |
| Wafer flicked over 0.1 m/s | 0 |

## Sources

UCSB nanofab, Suss MA6 SOP 3 (https://wiki.nanofab.ucsb.edu/w/images/5/5f/Suss_MA6_SOP3.pdf); NIST, Suss MA8 manual (https://www.nist.gov/document/manualma8pdf); EPFL CMi MJB4 manual 2024 (https://epfl.ch/research/facilities/cmi/wp-content/uploads/2024/11/MJB4_manual_2024.pdf); EPFL CMi MA6 Gen3 manual 2024 (https://epfl.ch/research/facilities/cmi/wp-content/uploads/2024/07/MA6Gen3_manual_2024.pdf); UCSB MA-6 SOP Rev D, pins raised and lowered by screws (https://wiki.nanofab.ucsb.edu/w/images/archive/4/41/20230802163346%21MA-6_SOP_Rev_D.pdf); wafer sizes, SEMI M1 (100 mm: 525 um, primary flat 32.5 mm), Topline wafer drawing (https://topline.tv/drawings/pdf/wafers/wafer_flats_thickness.pdf). SUSS MicroTec and the other makers' names are for identification only. The geometry here is written for this task; no maker CAD is used. `standin/chuck.svg` is the drawing of the low-cost chuck.

## Licence

MIT
