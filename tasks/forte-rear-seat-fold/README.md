# Fold a sedan's rear seatback from the trunk: one hand holds the spring-return knob, the other tips 9 kg past upright

**What is built.** The trunk at full scale, from fixed colliders (positions in setup; Sizes and Trunk below): a floor 1.20 m long, two side walls 60 cm tall and 1.35 m apart, a wheel arch on each, a 12 cm sill, two opening flanges 12 to 60 cm up, a parcel shelf 3 cm thick, wall to wall, its underside 60 cm up, and the 60% seatback, upright beside the 40% one. The shelf collides whole and is drawn as its two edges; the real trial uses an opaque board. The arms work from inside the trunk; their bases stand on its floor, 65 cm in from the sill; in a scan by geometry of bases on the trunk floor, 55 cm in is the nearest to the sill from which one hand holds the knob while the other pushes 33 cm up the seatback (SCANS.md, Base place). No hand passes the opening or the sill; they and the flanges touch nothing in the act. Of 55, 60 and 65 cm, 65 is the base place with the whole PULL tab inside the overhead image.

Tables, methods and scans are in [SCANS.md](SCANS.md).

**The act.** The 40% rear seatback of a 2019-2021 Kia Forte sedan reclines 25 degrees toward the robot, held by a striker latch at its top outer corner. Its lock knob is on the trim beside its top, 97 cm in from the sill. One hand pulls the knob 20 mm and holds it, the other pushes the seatback past upright, and gravity folds it. Kia's manual, after lowering the headrests (done here at the start): "Pull out the seatback locking knob(1) in the trunk, then fold the seat toward the front of the vehicle" ([2024][m24]; [2019][m19], PDF page 33). The manual gives the pull and the fold; the hold, the knob's return, the re-catch, the recline, the push from the trunk and the forces are this task's model, not the manual's.

**Latch model.** In this task's latch model, the knob springs back, and let go before the seatback leaves its striker, the latch catches again, so one hand holds it while the other pushes. The latch is free while the knob is pulled 15 mm or more; if the knob returns with the seatback under 5 degrees, the latch catches again and brings the seatback back at 2 degrees a second.

**What makes it hard.** The hold lasts into the push, until 5 degrees off the striker. The knob is reached across the trunk floor, under the shelf and over the arch; the push is on the same side, 60 cm out. 9 kg centred 0.30 m up at 25 degrees takes about 34 N square to the seatback 33 cm up, falling to zero at upright. The push is 34 N static (51.7 N peak in the demo); the bench check tests the arm's push before a run counts.

**Who does it.** A sedan's driver, for a load too long for the trunk: "to allow you to carry longer objects than could not otherwise be accommodated" ([Kia][m42], "if equipped"). Hyundai's [2010 Elantra manual](https://www.manualsdir.com/manuals/772779/hyundai-2010-elantra.html?page=27) has a "seatback locking knob in the trunk" too.

## Scoring

The seatback's hinge angle (0 latched) at the end; stop when folded and at rest for 1.0 s, or at 60 s (the run ends at the first step with the full score). Off the striker: 5 degrees or more, which needs the latch released (1/3). Past upright: more than the 25-degree recline (2/3). Folded: every one of the last 20 steps (1.0 s) within 10 degrees of the folded stop, the angle moving 1.0 degree or less in all (1.0). The stop is the recline plus 88, 113 at 25 (folded line 103). The score is the tier met at the end of the run.

**Both hands.** The score reads the hinge angle. In the full run's last second the right gripper is 276 mm from the seatback, the left 443. By geometry, at 100 to 108 degrees, in every pose found the gripper point nearest the seatback is at its foot, 0 to 14 cm up. Of 1,223 poses with a gripper point within 10 mm of it at 103 to 112 degrees, 242 were run in the simulator (288 runs): 0 held it; in the other 981 a force calculation finds no hold (SCANS.md, Both hands).

## Results

From this folder, in `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). Scores: `OUT/<mode>/result.json`.

```bash
python demokit.py task.py OUT --mode full
python demokit.py task.py OUT --no-record --mode none,pushonly,pullonly,letgo,shove
```

| Mode | What the robot does | Score (GPU) |
|---|---|---|
| full | pull and hold, push past upright, let go | 1.0 at step 216 (10.8 s), on an RTX 3090 Ti and an RTX 4070 Ti SUPER; off the striker (8.3 degrees) at 151, past 103 degrees at 196, at rest at 113.0 from 197 |
| none, pushonly, pullonly, letgo, shove | nothing; push, knob untouched; pull and hold, no push; pull, let go, then push (caught again); no knob, hand 8 cm into the latched seatback | 0.0 each |
| Dream-1 (dream-1-beta) | the left arm reaches the tab's side and does not grip it; the right arm goes up and away from the seat | Dream-1: 0.0 (1,200 steps); valid, one session, 0 errored trials |

Probes (SCANS.md, Probe runs): stopped at 109.9 degrees before rest 0.667; held at 10.3 degrees by both hands 0.333; knob let go at 4.4 degrees at most then pushed, jaws closed round the tab without a pull at four places (tab travel 0.0 mm; the pads load the tab 31 to 52 N), and a push on the 60% seatback, 0.0 each; pulled and pushed again 1.0 at step 492; `full` then `none` in one session 1.0 then 0.0.

**How the demo does it.** The demo follows the seatback's angle, read from the simulator, and keeps the push at most 3 degrees ahead. The left hand grips the tab from below, fingers 51 degrees up, jaws across its two flat faces at 40 N a finger for a pull of about 7 N; it is commanded 36 mm back (the tab stops at 20), and at 8 degrees opens to 33 mm and backs off. The right hand is commanded to 31 degrees and leaves the seatback at about upright (contact from step 124 to 179, gone at 180, 24.9 degrees).

**What the cameras show** (film). Left wrist: the tab at rest (steps 43, 70), gripped and drawn (87 to 151), back after release (from 168). Right wrist: the seatback's ribs, dim, from step 156, the ribbed face and side edge at 180 to 186, the folded top end and headrest (196 to 215). Overhead: the PULL tab at the upper left edge; the seatback as a dark outline, latched and to 10 degrees of swing; most of its trunk face, folded (196 to 215). SCANS.md has the projections and a render with the shelf drawn whole.

## Range check

Mode `full`, on both GPU classes, the seat and latch values together at two ends, the rest one at a time. The ranges are what was run, nothing beyond. Every point scores 1.0, at the same step on both:

- hard / easy end: 11 / 8 kg, recline 28 / 23, spring 500 / 250 N/m (10 / 5 N), release 19 / 10 mm, hinge damping 4 / 0, friction 1.5 / 0, striker 7 / 3 degrees, floor 1.30 / 1.10 m, tab 15 / 20 mm, stem 10 / 20 mm: step 240 / 200, rest 116.0 / 111.0; hard pullonly 0.0; easy letgo and pushonly 0.0
- at step 216, rest 113.0: shelf 50, 55, 59, 59.5 and 61 cm; left wall 3, 4 and 20 cm from the knob; knob 2 cm nearer, farther, higher and lower; contact friction 0.5

With the shelf at 50 cm a left fingertip touches the shelf's front lower edge for 59 steps, largest single point force 14.5 N, score unaffected; at the default shelf it stays 82.8 mm away. The demo takes the knob's place from the scene; the knob and wall points show reach and clearance (knob 2 cm lower / farther: the approach point is 23.3 / 9.8 mm off, the grip is reached). The shelf runs move the walls' and flanges' tops too; the floor length moves the sill and the flanges.

## Sizes

Positions in setup; the ranges run, this task's own, in Range check; real windows in Bench checks; row by row in SCANS.md, Sizes and Trunk.

| Part | Sim | From |
|---|---|---|
| Floor, between the arches, height | 1.20 m, 1.20 m, 60 cm | a review's rough words (Trunk) |
| Other trunk parts | in setup | estimates |
| Knob's place | face at x 32, 52 cm up, 5 cm from the left wall | the knob's place is this task's choice |
| Seatback | 55 of the two sections' 118.5 cm, 46 percent (60:40 is the trade name); rigid, 9 kg, inertia 0.45, 0.28, 0.30 kg m^2 at every mass | estimates |
| Hinge | damping 2 N m s/rad, friction 0.5 N m; its limit stops the folded seatback 0 to 19 mm above its cushion plate (self_collisions off); the cushion beside is drawn 11 cm tall | estimates |
| Knob | tab 25 mm out of the trim, 0.03 kg, travel 20 mm, spring 350 N/m, damping 2 N s/m, friction 0.2 N | shape: the manual's [figure OBDM031068L](https://ownersmanual.kia.com/full_webhelp/BDm/2024/en_US/topics/t00043.html); the rest estimates |
| Latch | lock 50,000 N m/rad; release at 15 mm, striker 5 degrees | estimates (Latch model) |
| Contact friction | 0.9 static, 0.8 dynamic | estimates |

## Trunk

Rough words of a 2022 Cerato (the Forte's name in Australia) sedan review ([BestFamilyCars](https://bestfamilycars.com.au/blog/2022-Kia-Cerato-Review-and-Buying-Guide-big-boot-Small-car)) give the floor ("at least 1.2m"), the 1.20 m between the arches ("about 1.2 metres") and the height ("between 500-600mm": 60 is its top). The walls stand 7.5 cm of arch depth outside the 1.20 m on each side.

## Real items

- Kia Back Assembly-Rr Seat 89300M7330B4U, 2019-2021 Forte 4-door sedan: [kiapartsnow](https://www.kiapartsnow.com/genuine/kia-back-assy-rr-seat~89300m7330b4u.html), $542.09 new. The buyer checks that it is the 40% section. Kia's [2020 brochure](https://www.kia.com/content/dam/kia/us/brochures/brochure_forte_2020.pdf) (pages 3 and 4) lists 60/40 split seats for LXS, EX, GT-Line and GT. The trial uses a used one (budget $50 to $150).
- Its latch, knob, cable and striker: bought with it, from the same car.

## Real trial

**Bench setup** (positions as in setup). The used 40% seatback with its hinge brackets, latch, knob with cable, and striker, from a salvage yard. The trunk is 19 mm plywood with steel corner braces: a floor 1.40 x 1.95 m (x -71 to 124, two sheets) that the arm bases bolt to; walls from x -65 to 57; the sill, a plywood strip 12 cm tall across x -71 to -65; each arch a 28 cm tall half-round (a 50 cm disc, its centre 3 cm above the floor, at x 32.5) faced with a 7.5 cm strip of 3 mm hardboard; flanges 15 x 48 cm; the shelf an opaque board screwed across the wall tops; the 60% seatback a closed box 64 x 60 x 10 cm, same recline. Hinge brackets bolt to the floor, hinge 6 cm up at x 60. The striker bolts to a braced upright (15 x 70 cm) behind the top outer corner, set for a 25-degree recline; accept 23 to 28 (angle gauge). The knob's housing screws to a trim block 6 x 13.5 x 22 cm on a 2 x 4 post, its cable run as in the car. Cushion: 54 x 55 cm of 1 cm EVA foam floor tile, 1 to 2 cm will do (check: folded within 10 degrees of the angle measured at setup). Budget: about $300 beyond the arms, with a digital angle gauge at about $25.

Tape checks (1 mm), the windows the runs support: shelf 50 to 61 cm; left wall 3 to 20 cm from the knob; knob depth within 2 cm, height 50 to 54; floor 1.10 to 1.30 m. The rest: to 2 mm (a workshop tolerance); write down the reading.

**Arm bases.** Each foot stands on the plywood: 70 x 200 x 10 mm, four 6 mm slots 90 mm to each side of the axis (y) and 12 to 23 mm fore and aft (read from the maker's [base.stl](https://github.com/i2rt-robotics/i2rt/blob/main/i2rt/robot_models/arm/yam/v1/assets/base.stl)). Axes at x 0, y 30.5 and -30.5, +/- 2 mm. Four M5 x 45 bolts through 5.5 mm holes 17.5 mm fore and aft, washers and lock nuts, the floor on 2 x 4 battens along both long edges and across at x -20 and 20. Load check: pull each arm's upper column 50 N forward and sideways (spring scale); accept no movement on a 1 mm ruler, else tighten or add a steel plate.

**Bench checks before trials.**

- Seatback (luggage scale, 0.1 kg): 8 to 11 kg, else buy another. Latch open, its top pulled square (0 to 50 N scale) starts at about 19 N at 9 kg: accept 15 (8 kg, 23 degrees, no friction) to 28 (11 kg, 28, 1.5 N m).
- Knob (0 to 20 N scale), measured with the seatback latched: 5 to 10 N at full pull; frees at 10 to 19 mm (ruler, 1 mm). Calipers (0.1 mm): tab 23 to 27 mm tall, 10 to 14 thick, 15 to 20 long; stem 10 to 20; full pull 18 to 22 and past the release. Tab height, tab thickness and full pull: the 2 mm workshop tolerance.
- Grip: at friction 0.5 on two pads 7 N needs about 7 N a finger; close as the policy commands and pull the arm back 20 mm: the pads must keep their place on the tab (left wrist camera).
- Push, before a run counts (the first measurement of the real arm's push): latched, a 0 to 100 N scale between right fingertips and push point, command 2.5 cm past contact: runs count once it reads at least the starting force there (about 34 N; 28 at the easy end, 51 at the hard; the top reading times 60/33).
- Latch: knob held, swing slowly; it stops catching at 3 to 7 degrees (digital angle gauge, 0.1 degree). Latched, pull the knob and let go: it must go back by itself and the seatback must stay put when pushed. A latch that does not catch again is not used.
- A part outside any window is replaced before trials. Push face: the seat's own fabric or carpet (friction 0.9 in the sim, 0.5 also run).

**Reset.** About 30 s, nothing consumed: push the seatback back by its top, holding it, until the latch clicks; check that the knob is back and the seatback stays put when pushed.

**One trial.** Both arms at their ready pose, seatback latched; run the policy; stop when folded and at rest for 1.0 s, or at 60 s.

**Scoring by hand.** A digital angle gauge (0.1 degree) on the trunk face's outer edge, clear of the push point, zeroed when latched; a 30 fps camera on the cabin side, above hinge height, films the cabin face and edge from latched to folded (a 10 cm scale on it), then the gauge, face up; the camera stands past the walls' end at x 57. From the last second: within 10 degrees of the folded pose measured at setup, changing by 1 degree or less (1.0); else more than the recline measured at setup (0.67); else 5 degrees or more (0.33). These three tiers are the whole score.

**Safety.** During a run nobody is within reach of the folding seatback; the operator is behind the arms with an emergency stop. Kia warns that a seatback returned without holding it "could spring forward": hold it at each reset. Bare grippers: plastic knob, fabric or carpet push face.

## Licence

MIT. No third-party asset is in this folder.

[m24]: https://ownersmanual.kia.com/full_webhelp/BDm/2024/en_US/topics/t00043.html
[m42]: https://ownersmanual.kia.com/full_webhelp/BDm/2024/en_US/topics/t00042.html
[m19]: https://ownersmanual.kia.com/full_webhelp/BD/2019/en_US/manual/2019%20Forte%20OM.pdf
