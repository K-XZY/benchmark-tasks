# Close a 300 mm wafer shipping box in the maker's order: left edge in and latched, right side pressed flush and latched

A 300 mm wafer shipping box (a front-opening shipping box, FOSB) is closed the way the box maker's manual gives: the door is carried by its two D-ring handles, its pivot edge (the robot's left) is set against the opening first and latched with a quarter turn, the far side is pressed flush against the wafer cushion's spring and latched, and both D-rings are folded down.

**What this task adds.** Hold-and-turn tasks use one hand to hold a part still while the other hand turns a latch. Here the order is set by the manual, the door is a free 1 kg body carried by two handles, and the far side springs back out until it is held flush, so the second hand has to press against a spring while its own latch turns. Latching the far side first, one latch only, a door left cocked or rings left up all score less.

**How the demo does it.** Both hands pinch the two legs of a lifted D-ring (jaws across the ring's plane), lift the door off its rest and carry it, yawed 10 degrees, so that its left edge meets the opening's rim. The door's own pose steers both hands every step. The left edge goes to the rim, the door is swung about it until the cushion pads meet the wafers, the left hand turns its ring a quarter turn (the right hand holds the far side), the right hand presses the far side in 5 mm and turns its ring, both hands let go and back off, and each hand folds its ring down with closed fingertips.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder, the scripted demo (the README command):

```bash
python demokit.py task.py OUT --mode full                                      # the full-score reference, filmed (640 x 360)
python demokit.py task.py OUT --no-record --mode none,rightfirst,oneside,crooked,ringsup   # failure checks, below
python demokit.py task.py OUT --mode full --jitter 0.01 --seed 1               # free parts shifted by up to 1 cm
```

`FOSB_TRACE=1` prints the gaps, pad compressions, ring angles and latch state every 10 steps. The demo reads the door's and the box's poses from the simulator (as `demokit` allows); a policy gets the cameras only.

**Result** (scripted, GPU physics on three lab PCs, 20 Hz): see the table. 560 steps (28 s of simulated time) to 1.0, against a limit of 3,000.

| Run | Setting | Score |
|---|---|---|
| README command, 3 runs | no jitter, 640 x 360, three lab PCs | 1.0, 1.0, 1.0 (560 steps each) |
| Jittered, 4 runs | `--jitter 0.01 --seed 1..4` (free parts shifted up to 1 cm) | 1.0 on all 4 |
| RTX 5090 | README command, GPU to itself | 1.0 |

The slowest 30 steps took 9 s at 640 x 360 on a lab PC (about 0.3 s a step): with Dream-1's 8 s idle limit, send 12 or fewer steps per request. At 320 x 176 (Dream-1's size, `--store-frames` on a fast GPU alone) the slowest 30 steps took 5.3 s (0.18 s a step): `rows_per_request=24` takes 4.3 s plus 0.5 s of round trip, under 5 s.

| Failure check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 |
| `rightfirst` | presses the far side and latches the right ring first, then the left | 0 (the order rule) |
| `oneside` | latches the left ring only, then folds both rings | 0.25 |
| `crooked` | swings the door in only to 4 degrees out of square, latches the left, and turns the right ring without pressing the far side in | 0.25 (the right latch never engages: its side is not flush) |
| `ringsup` | does everything but leave the D-rings up | 0.75 |

## Building it for a real trial

- **Box:** a used Entegris SB300 FOSB (385 x 333 x 337 mm), or any 300 mm FOSB, on the printed plate (the maker's three-pin coupling; the simulation holds the box by friction only). 25 reclaim 300 mm wafers (any 775 um wafer: only the cushion's push matters; dummy or test wafers are cheaper than prime ones).
- **Door:** the box's own door (363.7 x 258.0 mm, the manual). The simulation's 25 mm thickness, 1.0 kg and the D-ring bar are estimates.
- **Plate and rest:** printed; the rest is a 75 mm block with two 35 mm rails 28 mm apart. A pencil outline of the box on the plate reads the 5 mm movement limit.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Cushion's push at flush | 14 N (two 875 N/m springs, 8 mm) | A spring scale on the pressed door's edge |
| Cam torque | 1 N m (the manual's maximum is 4.4 N m) | A torque wrench on the D-ring |
| Door mass | 1.0 kg | A kitchen scale |
| Does the cam draw the door in by itself | assumed not (the latch only holds) | Turn the cam with the door held 5 mm proud |

## Size table and size check

Every dimension a hand or the scorer touches, the scene value, and its source. Manual: Entegris SB300 FOSB user manual, https://www.entegris.com/content/dam/product-assets/sb300fullpitchfrontopeningshippingboxesfosbs/manual-sb300-fosb-for-300mm-wafers-7429.pdf. Wafer: SEMI M1 / SEMI M31 (300 mm wafers and FOSB), https://store-us.semi.org.

| Dimension | Scene | Source |
|---|---|---|
| Box outside, W x D x H | 385 x 333 x 337 mm | Manual |
| Wafer diameter and thickness | 300 mm, 0.775 mm | SEMI M1 (the 300 mm wafer) |
| Wafer count, pitch, first slot above the datum | 25, 10 mm, 44 mm | Manual |
| Door, W x H | 363.7 x 258.0 mm | Manual |
| Door thickness, mass | 25 mm, 1.0 kg | Estimate |
| Box mass | 7 kg | Estimate |
| Opening, W x H, centre above the datum | 340 x 250 mm, 184 mm | Estimate (the manual gives no opening size; the door overlaps it by 11.9 mm a side and 4 mm top and bottom) |
| Rim depth; wafers' front edge behind the rim | 12 mm; 18 mm | Estimate |
| D-ring: bar, legs, height proud | 12 mm square x 52 mm, legs 40 mm apart, 36 mm | Estimate; the manual shows the D-ring handles without dimensions |
| D-ring spacing on the door | 300 mm (y = +/-15 cm) | Estimate |
| Cam torque (latch) | 1 N m | Estimate, within the manual's 4.4 N m maximum |
| Wafer cushion | two pads, 875 N/m each, 14 N together at flush (10 to 18 N) | Estimate; the manual gives no force |
| Plate under the box | 20 mm | Estimate (a stand for the real kinematic coupling, which is not modelled) |

Size check, printed from the constants in `task.py` (`POD_W`, `POD_D`, `POD_H`, `WIN_W`, `WIN_H`, `WIN_Z`, `DOOR_W`, `DOOR_H`, `WAFER_R`, `WAFER_T`, `WAFER_PITCH`, `WAFER_Z0`, `NWAF`) against the published figures; no simulator run is needed:

```
box      385 x 333 x 337 mm   published 385 x 333 x 337    ok
door     363.7 x 258.0 mm     published 363.7 x 258.0      ok
wafer    300 mm x 0.775 mm    published 300 x 0.775        ok
pitch    10 mm, first 44 mm, 25 wafers, last at 284 mm (top of box 337 mm)   published 10 / 44 / 25   ok
opening  340 x 250 mm         estimate; door overlaps it 11.85 mm a side, 4.0 mm top and bottom
```

## Simplifications, stated

- A latch is a rule, not a cam and hook: it engages when its ring is turned 80 degrees or more while its side is within 1 mm of flush, then holds that side to the rim with 12 N. The robot's contact turns the ring and presses the door.
- The wafers are fixed in the box's colliders (25 discs, 0.775 mm). The cushion is two spring pads that press on them; "the door strikes the wafers" is scored as the pads meeting them with the door more than 5 degrees out of square.
- The box and door are hand-built from published sizes. No vendor CAD was used; none is distributed here.

## Safety for a real trial

The door is light but its edges are sharp enough to cut and the wafers are fragile. Keep people clear of the arms' reach while the robot runs and keep an emergency stop in reach. Wear gloves when handling the wafers.

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. The scorer reads the door's four inner corners against the box's rim, the D-ring angles, the pads' compression and the box's position.

## Trademarks and sources

Entegris and SB300 are trademarks of Entegris, Inc. They are used here only to name the product whose published dimensions the box follows; this task is not affiliated with or endorsed by Entegris. Sizes and the procedure quoted in the task text come from the SB300 FOSB user manual (entegris.com, manual-sb300-fosb-for-300mm-wafers-7429.pdf).

## Licence

MIT. Copyright (c) 2026 the task's authors.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: the above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
