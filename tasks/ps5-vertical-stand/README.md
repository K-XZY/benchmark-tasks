# Hold a PS5's vertical stand square while hand-turning its screw snug, then roll the 2.6 kg console up onto it

A PS5 Slim lies flat on a soft cloth on a media unit, its rear edge toward the arms. Sony's vertical stand sits on the console's end on its started mounting screw. One hand holds the stand's chrome ring square while the other turns the screw by its head, in strokes, three turns until it seats. Then both hands pinch the console's rear edge, roll it a quarter turn up onto the stand, set the ring down on the cloth and let go with the console standing.

**Ability tested:** two hands doing different jobs on one axis, then sharing a load neither can carry. The screw's head drags the stand round with it, so a stand left unheld ends the turning skewed. Seating the screw takes a rising torque through finger friction alone. The 2.6 kg console is more than one arm's 1.5 kg payload, and held at one end its centre of mass is 179 mm away (about 4.6 N m), so the roll needs both hands and must end square on the ring without a push that tips it.

**Why it matters:** more than 100 million PS5 consoles have been sold (Sony Interactive Entertainment business data, as of 10 Nov 2026, https://sonyinteractive.com/en/our-company/business-data-sales/) and PlayStation counts 127 million monthly active users (same page, 30 Sep 2026). Sony's support page gives every owner who wants the console upright the same steps: "Place your console on a soft cloth spread over a flat surface before attaching the stand" and "secure with the mounting screw using your hand" (https://www.playstation.com/en-gb/support/hardware/ps5-vertical-stand/). Owners do it at set-up and again whenever the console is moved, cleaned or switched between flat and upright.

**Hands:** one hand holds the stand while the other turns the screw; both hands roll the console. The score does not check which hand does what.

## Real items

| Item | Maker, model | Link | Sizes used |
|---|---|---|---|
| Console (about 400 USD certified refurbished) | Sony PlayStation 5 Slim, Digital Edition, model group CFI-2000 | https://www.playstation.com/en-gb/ps5/ | 358 x 80 x 216 mm, 2.6 kg (CFI-20XX Safety Guide, https://www.playstation.com/content/dam/global_pdc/en-gb/corporate/support/manuals/ps5-docs/2000ab/CFI-20XX_PS5_Safety_Guide_Web$en-gb.pdf) |
| Vertical stand (about 30 USD) | Sony Vertical Stand for PS5 consoles, with its mounting screw | https://www.playstation.com/en-gb/accessories/vertical-stand-for-ps5/ | ring 190 mm across (W190 x D190 x H42 mm, Sony), 42 mm tall; plate 216 x 80 mm across the console's end; screw head 18 mm across, 6 mm proud (estimates; see Fidelity) |

Price sources: Best Buy, Geek Squad Certified Refurbished PS5 Slim Digital Edition, 399.99 USD (https://www.bestbuy.com/site/-/6605878.p); PlayStation Direct, Vertical Stand (https://direct.playstation.com/en-us/buy-accessories/vertical-stand-for-ps5-consoles). A lower-cost trial (about 40 USD with the stand) can use a stand-in for the console: a 358 x 80 x 216 mm plywood box weighted to 2.6 kg, with a brass knife-thread insert for hardwood at the console's screw hole (E-Z LOK 400-M4, M4-0.7, 0.375 in long, https://www.ezlok.com/ezknife-insert-400-M4; replacement-screw listings give the stand's screw as M4, some give M3, so check the screw with a thread gauge and buy the insert in the matching size); the act and the score are the same, since the score reads only the console's pose, the screw and the stand.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `gatekit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Fit the stand to the PS5: hold it square while you turn its screw snug by hand, then roll the console upright onto it and let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                   # full score, with a video
python demokit.py task.py OUT --no-record --mode noscrew,nohold,unseated    # lazy routes below
python demokit.py task.py OUT --no-record --mode noroll,onehand,drop
python demokit.py task.py OUT --no-record --mode slowseat
```

The demo is a scripted reference. It reads the console's pose and the stand's angle, which the cameras see; a policy gets only the cameras. The right hand pinches the near side of the stand's ring. The left hand pinches the screw head level, turns it 120 degrees, lets go, turns back and pinches again, until the screw seats; between strokes the right hand turns the stand back if it has crept more than 1 degree. The left hand goes round the stand; both hands pinch the console's rear edge 50 mm in from it, 120 mm apart, across its 80 mm thickness, with 35 N each, and roll it a quarter turn about its long side while lifting it up to 250 mm, so the ring clears the unit. They stand the console's long axis up by its own lean, lower the ring onto the cloth, open, and back away.

## Scoring

See `scoring` in `task.json`, and `Trial.step`, `Trial.score`, `clearance` and `gripper_points` in `task.py`. The score is read from the state at each step; only the void latches. The screw's seating and the stand's lock are physical states: once seated, both joints hold.

- **1/3:** the screw turned 3 turns from started, through its seating torque (0.15 N m running, rising to 0.25 N m over the last quarter turn), with the stand within 5 degrees of where it started, counting its whole turn (a stand spun a full turn round is 360 degrees off, not square).
- **2/3:** also the console's long axis within 10 degrees of vertical.
- **1.0:** also the ring's bottom within 10 mm of the cloth, the long axis within 5 degrees of vertical, the console at rest (under 0.02 m/s), and every point of both grippers (fingers, pads, palm and housing) more than 30 mm from the console and the stand, all held for the last 3 s (60 steps).
- **Void (0):** the console or its ring moves down faster than 0.35 m/s (a free fall of about 6 mm), the console's centre drops 20 mm below the unit's top, or the console tips past 45 degrees after it stood.
- **Success:** 1.0.

In a real trial each tier is scored by hand; see Real trial.

## Real trial

**Bought items:** a Sony PS5 Slim (CFI-2000, Digital Edition, 358 x 80 x 216 mm, 2.6 kg) or the weighted plywood stand-in under Real items; the Sony Vertical Stand for PS5 consoles with its mounting screw (in the box); a media unit or box with a flat top 160 mm high and at least 330 x 495 mm (a 3/4 in plywood box, 330 x 495 x 160 mm, works); an adjustable click-type torque screwdriver, 0.1 to 0.6 N m (Wiha TorqueVario-S, RS stock no. 2436291, https://au.rs-online.com/web/p/torque-screwdrivers/2436291) with a wide flat bit; a digital angle gauge with level (Wixey WR365, https://www.woodcraft.com/products/wixey-wr365-digital-angle-gauge-with-level); digital calipers; a tape measure; a pen; an IKEA VITMOSSA fleece throw, 47 x 63 in, folded to about 290 x 465 mm, for the cloth (https://www.ikea.com/us/en/p/vitmossa-throw-gray-90304889/; tilt-tested as in the Fidelity table: a cloth whose tilt test reads under 0.5 contact friction is swapped for one that passes, or the run is reported out of range); two phone cameras on stands (see Cameras).

**Bench setup:** the unit in front of the two arms, its front edge 310 mm out from the station origin (`UNIT_X0` in `task.py`), the cloth spread over its top. The console lies flat on the cloth, its rear edge toward the arms and 60 mm proud of the unit's front edge, its stand end 5 mm past the unit's side. The stand sits on that end face with its marks square to the console's, on its screw turned in by hand until it just holds (started): 3 turns short of seated.

**Cameras:** one at 60 fps on a stand 1 m from the unit's front, looking side-on at the console's height over the whole trial, with a 300 mm ruler lying on the unit in view (at 60 fps a 0.35 m/s fall is 6 mm a frame, a 6 mm step against the ruler); one looking down the stand's end face for the stand's marks. Both run from before the first touch to 3 s after the last release.

**Reset:** back the screw out to started (3 turns from seated, counted by the pen mark), square the stand by its marks, and lay the console flat in its start pose (tape against marks on the cloth).

**One trial:** 2400 control steps (120 s at 20 Hz) under the instruction above. The arms hold the stand, turn the screw until it seats, roll the console upright onto the ring, and let go.

**Scoring by hand:**
- 1/3: the screw seated: after the trial the torque screwdriver, set to the seating torque measured on the bench (bound 0.15 to 0.35 N m, see below), clicks without turning the screw further, and the pen mark shows 3 turns from started; the stand's marks within 5 degrees of the console's (angle gauge on a photo of the end face, counting any full turn).
- 2/3: also the console's long side within 10 degrees of vertical (phone level on its side).
- 1.0: also the ring's bottom on the cloth (tape: within 10 mm), the long side within 5 degrees of vertical, the console still, and both grippers at least 30 mm from the console and stand for the last 3 s (side camera; the 3 s read from its timestamps and the ruler).
- Void (0): the console or its ring dropped (side camera: more than about 6 mm of fall between consecutive frames at 60 fps, seen against the ruler), the console off the unit (its centre 20 mm below the top), or the console tipped past 45 degrees after it stood.

**Screw torques on the bench:** before the first trial, set the torque screwdriver to 0.2 N m and turn the screw from started: it must not click before the last quarter turn (running torque under 0.2 N m; the task uses 0.15; bound 0.05 to 0.2). Then find the lowest setting, in 0.05 N m steps from 0.15 to 0.35, at which the screw seats without a click: that is the seating torque (the task uses 0.25 N m; bound 0.15 to 0.35 N m, from T = 2 mu F r with a 15 to 35 N fingertip pinch, mu 0.6, 9 mm head radius). Before trial 1 also measure the head with calipers: the trial is valid at 5 to 8 mm proud and 15 to 20 mm across; a flatter head is reported as out of range, not scored. Count the turns from started to seated (the task uses 3; bound 3 to 5). The demo scores 1.0 at the bounds' hard ends (see Results); a reading outside a bound is reported with the trial.

## Results

| Run | Score | Note |
|---|---|---|
| `--mode full` | 1.0 | success at step 1142 (57.1 s), RTX 3090 Ti; the same step 1142 (57.1 s) on an RTX 4070 Ti SUPER (see the 5 mm head row, same code and scene) |
| full at the torque and turn bounds (`PS5_SEAT=0.35 PS5_DRAG=0.2 PS5_TURNS=5`) | 1.0 | 5 turns, 0.35 N m reached; step 1670 (83.5 s), RTX 4070 Ti SUPER |
| full with a 5 mm proud screw head (`PS5_HEAD=0.005`) | 1.0 | step 1142 (57.1 s), RTX 4070 Ti SUPER |
| full with a 4 mm proud screw head (`PS5_HEAD=0.004`) | 0 | below the stated bound: the fingers lose the head and the stand turns about 50 degrees |
| full with a 3.5 mm wide ring (`PS5_RINGW=0.0035`) | 1.0 | step 1142 (57.1 s), RTX 3090 Ti |
| full with a 6 mm thick ring (`PS5_RINGT=0.006`) | 1.0 | step 1141 (57.05 s), RTX 3090 Ti |
| full with the cloth at 0.4 static (`PS5_MU=0.4`, the cloth only; averaged with the console's 0.6 shell the contact is 0.5 static) | 1.0 | step 1170 (58.5 s), RTX 3090 Ti |
| full with a 30 N grip (`PS5_GRIP=30`), 5 mm head | 0.67 | the console slid about 50 mm as it was released and the hands stayed within 30 mm: 30 N is too low; the default is 35 N |
| (lazy rows below run at the defaults: 4 mm ring, 35 N grip) | | |
| `slowseat`: last 0.3 turn at 3 degrees per second, then stop | 1/3 | peak resisting torque 0.25 N m, the seating torque |
| `noscrew`: stand held, screw not turned, console rolled | 0 | |
| `nohold`: screw turned with the stand unheld | 0 | the stand spun round with the screw, about 1080 degrees |
| `unseated`: screw stopped 0.4 turn short of seated | 0 | |
| `noroll`: screw seated, console left flat | 1/3 | |
| `onehand`: one hand only | 1/3 | |
| `drop`: console let go 100 mm up | 0 | void |
| Dream-1 (one run, seed 1, 2400 steps) | 0 | the model's arms did not move to the console or stand; no tier reached; RTX 4070 Ti SUPER |

No seed is needed: nothing in the scene is random.

## Fidelity

The console is one rigid 358 x 80 x 216 mm box, drawn as a black centre body between two white covers whose edges curve in; the stand is a 16-segment chrome ring under a plate across the console's end, on a hinge about the screw's axis, as a stand on a started screw turns. The screw is a hinge on the same axis. Its friction is a drive whose target stays put while the hand's torque is under the screw's torque and slides to hold exactly that torque when it is over, so the torque to turn it is the same at any speed (checked by the `slowseat` route, which turns the last 0.3 turn at 3 degrees per second).

| Quantity | Sim (task.py) | Real | Source | Real-trial check | Effect of the difference |
|---|---|---|---|---|---|
| Console size, mass | 358 x 80 x 216 mm, 2.6 kg | same | Sony CFI-20XX Safety Guide | tape, kitchen scale | none |
| Gripper pinch force | 35 N per gripper (one per hand), on the ring, the screw head and each console end | the YAM gripper's limit is about 40 N (harness limit, estimate) | estimate: set 5 N below the limit; at 30 N with a 5 mm head the console slid about 50 mm as it was released and the hands stayed within 30 mm (0.67); 35 and 40 N score 1.0 | none: the real arms are driven the same way | none: the console's weight is 12.7 N per end and two pads at mu 0.5 give 2 mu F = 35 N of friction per end |
| Stand ring | 190 mm across, 42 mm tall | 190 x 191 x 42 mm | Sony product page; retailer listing (Currys, 190 x 191 x 42 mm) | tape | none |
| Ring section | 4 mm wide, 10 mm thick, pinched across the thickness | width about 3.5 to 5 mm, thickness about 6 to 10 mm | Sony's photo: the ring's outer width spans about 1325 px for 190 mm (7.0 px per mm) and its rim is 25 to 35 px (3.5 to 5 mm) wide at the left and right sides; the thickness is not measurable in the foreshortened view (estimate) | calipers on the rim | the sim rim matches the photo; demo 1.0 at 4 mm wide, 1.0 at 6 mm thick |
| Plate | 216 x 12 x 80 mm | about 200 to 220 x 10 x 70 to 85 mm | estimate from Sony's photo | tape | none: not grasped |
| Stand mass | 0.15 kg | 100 to 200 g | estimate | kitchen scale | small: the holding hand carries it |
| Screw head | 18 mm across, 6 mm proud | 15 to 20 mm across, 5 to 8 mm proud, coin slot | estimate: Sony's photo shows the stand's socket, not the screw, so the head cannot be scaled from it; Sony says the screw is turned by hand or with a coin, which needs a head a fingertip pinch can hold | calipers | a shorter head is harder to pinch; no lip or grip added; demo 1.0 at 5 mm proud; at 4 mm the stand is not held square (score 0), so the bound starts at 5; a real head under 5 mm proud is out of range and reported |
| Running torque (DRAG_NM) | 0.15 N m | 0.05 to 0.2 N m | estimate: a hand screw that turns freely by fingertip but does not unwind under the stand's weight | click torque screwdriver, 0.1 to 0.6 N m, set to 0.2 N m: no click | demo 1.0 at 0.2 |
| Seating torque (SEAT_NM), over the last quarter turn | 0.25 N m | 0.15 to 0.35 N m | calculation: fingertips slip at T = 2 mu F r; F 15 to 35 N, mu 0.6, r 9 mm give 0.16 to 0.38 N m | click torque screwdriver, 0.15 to 0.35 N m settings | demo 1.0 at 0.35 |
| Share of the screw torque that turns the stand | 0.5 | about 0.5 under the head | bolted-joint torque split, https://en.wikipedia.org/wiki/Bolted_joint | an unheld stand turns with the screw, seen | none |
| Turns from started to seated | 3 | 3 to 5 | estimate | pen mark on the head | demo 1.0 at 5 |
| Screw drive stiffness (SCREW_K) | 20 N m/rad | none: model device | the screw gives at most 1 degree (0.35 / 20 rad) before it slips | none | none |
| Stand drag spring | 2.0 N m/rad, capped at half the screw torque | none: model device | carries the head's drag to the stand; capped, so a held stand stays put | none | none |
| Console shell, stand and screw-head friction | 0.6 static, 0.5 dynamic | about 0.4 to 0.6 | estimate | tilt test: tan of the slide angle | see the next row |
| Cloth friction | 0.9 static, 0.8 dynamic; with the console, averaged: 0.75 static | console on cloth about 0.5 to 0.8 | estimate | tilt test of the console on the cloth | demo 1.0 at cloth 0.4 (contact 0.5 static, run in Results); the window starts there |

The soft cloth is drawn 2 mm thick on the unit's top; it is rigid, and its friction is that of the unit's top. The media unit is 160 mm tall so the arms reach the console; the act is the same on any flat top.

## Assets

All meshes are made by the task's own code from plain shapes. No third-party assets, no textures.

## Licence

MIT
