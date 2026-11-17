# Screen a mannequin's arm with a detector wand: sweep over, then under it at 5 cm, slow at the watch until it alarms

Security guards (O*NET 33-9032.00; about 1.3 million US jobs, BLS 2024) have the Core task 'Operate detecting devices to screen individuals and prevent passage of prohibited articles into restricted areas.' Guards at venues, schools and courts screen with a hand wand. Garrett's SuperWand guide (1531500.B) says the wand detects metal only while it is in motion and is to be scanned within about two inches of the person. This task is one arm: the 48 cm wand is a lever held at its grip end, the arm tapers from 10 cm to 5.5 cm across, and the scan surface must follow it at that distance over the top and then under it without touching, the wand rolled round the arm between the passes. Too far or held still, the wand misses the watch; too close, it touches. O*NET is a trademark of USDOL/ETA; task text used under CC BY 4.0. Garrett and Casio are named for identification only.

**Instruction.** Screen the mannequin's arm with the metal-detector wand: take the wand from its holder by its grip and sweep it over the top of the arm from shoulder to fingertips, then under the arm back to the shoulder, keeping the coil about 5 cm (two inches) off the arm and never touching it. Keep the wand moving, and slow down at the wristwatch until the wand's red lamp lights. Then lay the wand back flat in its holder, let go and move your hand clear.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Screen the mannequin's arm with the metal-detector wand: take the wand from its holder by its grip and sweep it over the top of the arm from shoulder to fingertips, then under the arm back to the shoulder, keeping the coil about 5 cm (two inches) off the arm and never touching it. Keep the wand moving, and slow down at the wristwatch until the wand's red lamp lights. Then lay the wand back flat in its holder, let go and move your hand clear."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                              # full score, with a video
python demokit.py task.py OUT --no-record --mode none,hover,far,touch,top,side   # the lazy routes below
python gate_ik.py                                                      # the IK dry run, no simulator
```

The right hand takes the wand by its handle, fingers level and jaws closing on the flat handle, lifts it 4 cm and turns the hand until the wand lies flat. It draws the wand toward the robot, rises past the arm and brings the scan surface to 4.5 cm over the shoulder end, the wand tilted 2.5 degrees to lie parallel to the arm's tapering top. It sweeps to the fingertips at 10 cm/s, slowing to 4 cm/s over the watch, where the lamp lights. It slides back to 50 cm from the shoulder, circles the wand round the near side of the arm to 4.5 cm under it, sweeps out to the fingertips and back under the arm to the shoulder, then lowers the wand straight onto its holder, lets go and draws the hand back 10 cm. Along each pass the demo reads the wand's pose, steers the scan surface onto its path and turns the hand a little to keep the wand level; the left arm stays at rest. It never moves an object except through the gripper.

## The scene and what is simplified

Station: two YAM arms with parallel grippers, bases 61 cm apart at the near edge of a 120 x 90 cm wood table, 75 cm high. Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left; heights above the table top. Mannequin arm: 70 cm from shoulder to fingertip; a round forearm 52 cm long, 10 cm across at the shoulder end tapering to 5.5 cm at the wrist, then a flat hand 18 cm long, 8 cm wide and 4 cm thick. It is held out sideways along y, palm down, its axis 32 cm above the table, its shoulder end at x 32, y -45 and its fingertips at y 25. Its stand: a bracket from the shoulder end to a 4 cm square mast at x 46, y -47, on a 14 cm square foot. A Casio MTP-1302D steel watch (case 44.2 x 38.5 x 9.2 mm, 105 g with band) sits on top of the wrist, its centre 6 cm short of the wrist end of the forearm (y 1). Wand: a Garrett SuperWand 1165800, 48.3 x 8.3 x 3.2 cm, 0.45 kg, with a 12 cm handle 3.2 cm square. The fingers take it 10 cm from the handle's end; its mass is spread through its shape, so its centre of mass is 17 cm beyond that point. The scan surface is the far 22 cm of the wand. It lies flat on its holder under the arm's shoulder end, its axis along y, its handle end at y -72 and its tip at y -24, the handle's centre line 12 cm above the table. The holder stands at x 32: two 3 cm square posts 10.3 cm tall, at y -68 under the handle, with a low stop either side of the handle, and at y -42 under the body. The red lamp near the handle lights while the wand moves at 2 cm/s or more with its scan surface within 5 cm of the watch case; it is dark when the wand is still or farther away. Both grippers start at the arms' ready pose. The left arm stays at rest; the score does not check which hand holds the wand. Start state, the same every trial: the wand at rest on the holder, the lamp dark.

- Rigid bodies at the maker's sizes; the wand's mass is spread evenly through its shape; the fingers take the handle near the body. Lamp: lit by code when the scan surface is within 5 cm of the watch case and the wand moves at 2 cm/s or more. Garrett's manual says only 'scan within approximately two inches' and 'detects metal only while in motion'; it gives no watch range, and the 2 cm/s cut-off is ours. Touch is read from the wand's shapes and the fingertips. The switch is not modelled.
- Sources: Garrett SuperWand user guide 1531500.B (https://cdn.shopify.com/s/files/1/2084/4155/files/Garrett_Superwand_Manual.pdf: 'detects metal only while in motion', 'scan within approximately two inches') and spec sheet for 1165800 (https://www.garrettaustralia.com.au/assets/files/GMD-1165800%20SuperWand%20Spec%20Sheet.pdf: size and mass); Casio MTP-1302D case 44.2 x 38.5 x 9.2 mm, 105 g (retailer listing, https://www.skywatches.com.sg/products/casio-white-dial-mens-dress-watch-mtp-1302d-7bvdf-mtp1302d); the wand's 0.45 kg is the manual's 16 oz (spec sheets list 18.6 oz); O*NET 33-9032.00 (https://www.onetonline.org/link/details/33-9032.00); BLS Occupational Outlook Handbook, Security Guards and Gambling Surveillance Officers. All meshes are built in code; nothing is downloaded.

## Scoring

Read from video of the wand's red lamp, with a foam gauge marked at 5 and 7 cm held to the arm for the stand-off, a tape for the holder and a photo for the wand lying flat. Score 0, and it stays 0, if the wand touches the arm, hand or watch at any time (in the simulation: any point of the wand within 2 mm, or any fingertip within 4 mm, of their surface). Otherwise cumulative, and nothing without the lamp: 0.25 once the lamp has lit (the scan surface within 5 cm of the watch case while the wand moves at 2 cm/s or more); 0.5 once a top pass is complete as well: every 5 cm stretch of the arm's 70 cm length has had the moving scan surface over it: the part over that stretch centred across the arm (its middle within the arm's half width of the arm's centre line), wholly higher than the arm's top there and within 7 cm of the arm; 0.75 once an underside pass is complete after that, the same but wholly lower than the arm's underside (a sweep along the arm's side counts for neither); 1.0 with the wand then back on the holder within 3 cm of its start place and 1 cm of its start height, lying flat (within 8 degrees), no fingertip within 3 cm of it and the wand still, held for 2 s; a wand that was let go more than 1.5 cm above the holder (dropped) does not reach 1.0.

## Failure table

Each lazy route, run in the simulator (`--mode`), must score no more than its cap:

| Route | Mode | Cap | Result |
|---|---|---|---|
| Do nothing | `none` | 0 | 0 |
| Hold the coil still over the watch | `hover` | 0 (no lamp: the wand is not moving) | 0: the lamp never lit |
| Both passes 11 cm off the arm | `far` | 0 (no lamp: past two inches) | 0: the lamp never lit |
| Press the coil into the arm | `touch` | 0 (touch) | 0: touch over the shoulder end |
| Top pass only, wand kept in hand | `top` | 0.5 | 0.5 |
| Top pass, then the second pass along the arm's near side just below its centre line instead of under it, wand back on the holder | `side` | 0.5 | 0: no underside stretch counted along the side (0.5 through the side sweep); the wand's tip then met the arm's side on the way back, which ended the run at 0 |

Nothing is random (object_jitter=0). Between real trials lay the wand flat on its holder and switch it on.

## Licence

MIT
