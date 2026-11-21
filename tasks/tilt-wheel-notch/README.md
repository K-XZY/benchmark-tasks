# Raise a tilt steering wheel two notches: pull the lock lever down, lift the drooping wheel, push the lever up to lock it

A car's tilt steering column stands on the bench: a dashboard block with the column coming out of it at 23 degrees, a 370 mm wheel on its end, and the tilt lock lever under the column on the driver's left, as in the owner's manuals of a 2022 Kia Sonet (QY) and a Mazda CX-3. The wheel starts locked in its lowest notch. The robot does what the manuals give: pull the lock-release lever down, raise the wheel to the wanted height (here two notches up, notch 3 of 5), push the lever back up so the column locks there, and let go.

**What this task adds.** With the lever open the column is free, and it droops under its own weight against a counterbalance spring, so one hand has to carry the wheel at the target height while the other closes the lever. In the simulator, with the lever open the column never springs up: it falls to its lowest notch. The lock only engages when the column sits on a notch, within 0.3 degrees: a lever closed between notches leaves the column free until it drifts onto the next notch, and a lever closed on the wrong notch locks it there. The lever stays where it is left, the target is one notch out of five, and the lever must be pushed closed, against its own friction, while the other hand holds the wheel there.

**Who does it, and how often.** Every driver who shares a car resets the wheel before driving, and fleet, bus and truck drivers do it at the start of a shift: about 2.2 million US heavy and tractor-trailer truck drivers ([BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/transportation-and-material-moving/heavy-and-tractor-trailer-truck-drivers.htm); many heavy-truck columns tilt by an air foot valve instead, so read it as an upper bound) and 233 million licensed US drivers ([FHWA Highway Statistics 2023](https://www.fhwa.dot.gov/policyinformation/statistics.cfm)). Kia's owner's manual ([Tilt steering wheel, 2022 Kia Sonet (QY)](https://ownersmanual.kia.com/full_webhelp/QY/2022/en_IN/topics/chapter4_9_2.html)) gives the steps: pull the lock-release lever down, set the wheel, pull the lever up to lock it, then push the wheel up and down to check it is locked. Mazda's CX-3 manual ([Steering Wheel](https://owners-manual.mazda.com/gen/en/cx-3/cx-3_8gt4ee18d/contents/04050101.html)) gives the same: pull the lever under the column down, tilt the wheel, push the lever up to lock it.

**How the demo does it.** The right hand grips the rim 15 degrees above its right-hand spoke, pointing into the wheel with its jaws across the rim. The left hand comes in from the side between the wheel and the dashboard, pinches the lever's paddle across its faces and swings it down 25 degrees about the lever's pivot, turning the grip with the paddle; the lever stays open and the hand lets go. The right hand then carries the rim up the column's arc in small steps until the column reads 6 degrees, and holds it there while the left hand grips the paddle again and swings it back up to its stop. The lock engages, both hands open and back off, and the scorer's 5 N push at the rim checks the lock.

## Real items

- Kia Sonet (QY, 2022) tilt steering column, lock-release lever under the column: [Kia owner's manual, Tilt steering wheel](https://ownersmanual.kia.com/full_webhelp/QY/2022/en_IN/topics/chapter4_9_2.html).
- Mazda CX-3 tilt column and lock-release lever: [Mazda owner's manual, Steering Wheel](https://owners-manual.mazda.com/gen/en/cx-3/cx-3_8gt4ee18d/contents/04050101.html).

Both are shape references only (lever position and travel, wheel on a tilting column); neither manual gives a wheel size, spring or lever force, so those are the estimates in the Sizes table. Prices: each manual is free online (USD 0); neither car is bought. The real trial uses a used column (typically under USD 150, estimate).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder, the scripted demo (the README command):

```bash
python demokit.py task.py OUT --mode full                                                                # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode none,liftonly,leveronly,overshoot,leveropen,restwheel,lightlever,regrab,drop,held_fast   # failure checks, below
```

`TRIAL_TRACE=1` prints the column angle, lever angle, lock state, hands-clear distance and score every 10 steps. The demo reads the column and lever angles from the simulator, as `demokit` allows; a policy gets the cameras only.

**Result** (scripted, GPU physics, 20 Hz, RTX 3090 Ti): 1.0 at step 585 (29.25 s of simulated time), against a limit of 1,200 steps.

**Dream-1** (the Dream-1 model on this task, one trial, 1,200 steps, no lost session): 0.

| Failure check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 |
| `liftonly` | the rim hand lifts; the lever is never touched | 0 (the column stays locked at its lowest notch) |
| `leveronly` | the lever is pulled down and pushed back up; the wheel is not lifted | 0 (it relocks at the lowest notch) |
| `overshoot` | as `full`, but the wheel is lifted to 9 degrees (notch 4) before the lever closes | 0.33 (raised, locked at the wrong notch) |
| `leveropen` | the lever is pulled down, the wheel lifted to 6 degrees and let go with the lever open | 0 (the wheel droops back to its lowest stop) |
| `drop` | the wheel is lifted to 12 degrees and let go, the rim hand clears, and the lever is shut while the wheel falls past a notch (about 9 degrees per second) | 0 (void: locked while moving with no gripper within 3 cm of the wheel; the run prints `lock ... held=False`) |
| `held_fast` | the rim hand lifts the wheel at about 6 degrees per second and keeps hold while the lever shuts onto a notch | 0.67 (locked, a hand on the wheel; no void because the wheel is held; the run prints `lock ... held=True`) |
| `restwheel` | as `full`, but the rim hand stays on the rim at the end | 0.67 (locked at the target, a hand on the wheel) |
| `lightlever` | as `full`, but the lever hand sweeps the paddle with open fingers, no pinch | 0 (the 15 N lever does not move, the column stays locked) |
| `regrab` | as `full`, but the rim hand opens, then closes on the rim again and ends holding it | 0.67 (locked at the target, a hand on the wheel) |

## Sizes

| Size | Scene | Source |
|---|---|---|
| Wheel diameter | 370 mm | estimate: neither manual nor Kia's Sonet spec page gives a wheel diameter; passenger-car wheels are 36 to 38 cm across; Kia lists a D-cut wheel for some Sonet trims, modelled round |
| Rim section | 25 mm | estimate, typical |
| Column angle | 23 degrees from horizontal | estimate from the manuals' figures |
| Tilt pivot to wheel centre | 260 mm | estimate |
| Tilt range | 0 to 12 degrees, five notches 3 degrees apart; target notch 3 (6 degrees) | estimate: the manuals give no figures |
| Moving mass | 3 kg (wheel and upper column) | estimate |
| Counterbalance spring | 3.4 N m/rad, leaving about 1.2 N m of droop at 6 degrees (gravity alone 6.2 N m) | estimate: a droop-type tilt column (Kia and Mazda manual figures) carries a counterbalance spring |
| Lever | 70 mm arm, 45 x 45 x 10 mm paddle, 25 degrees travel, unlocks past 10 degrees, 1.0 N m friction (about 15 N at the paddle) | estimates from the manual figures; no spring scale reading of a real lever was taken, the force is an estimate; the pinch holds 40 N |

**Forces in the demo.** Pinch 40 N a finger (the YAM's figure), lever push about 15 N (under the arm's push of about 20 N), rim lift about 5 N at the rim, because the counterbalance spring carries most of the 3 kg moving mass. A real trial needs no extra protection beyond pad covers on the gripper and a guard over the wheel's lowest swing for fingers.

## Real trial

A used tilt steering column with its wheel, from a salvage yard or an online parts seller (the Mazda CX-3 column, bought by the car's VIN from a parts seller, is the easier one to find in the US), bolted to a bench frame at 23 degrees (typically under USD 150 used; an estimate). Score it with a digital angle gauge (0.1 degree, about USD 30) on the column shroud and a 5 N spring scale at the rim. Notch 3 is the second position above the lowest (on a column with finer teeth, its gauge reading is measured at setup): read it as the angle gauge showing 6 +/- 0.3 degrees above the start reading, and in a side photo the rim top level with the yellow tape mark on the dashboard face. Choose a column whose wheel droops when the lever is open (as the Kia and Mazda manual figures show) and check the droop with the 5 N scale at the rim: the wheel must sink from 6 degrees when released with the lever open. Measure the lever's closing force at the paddle with a spring scale when the column is bought and set the 15 N estimate to it; above 20 N the real trial is not feasible. Hands-clear: no part of either gripper within 3 cm of the wheel, column or lever, judged from the end photo or video. The simulator's lattice of points 2 to 3 cm apart over each gripper is used in the simulator only. The void is judged from video: the column still moving when it locks onto a notch, with no gripper within about 3 cm of the rim, spokes or hub.
