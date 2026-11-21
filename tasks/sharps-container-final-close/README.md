# Press a full sharps container's cap through a light click to its final lock, then carry it by the handle into a box

A full 1.5 qt phlebotomy sharps container (BD 305487, 127 x 145 x 229 mm, 0.5 kg) stands open on a clinic counter, its clear hinged cap leaning toward the robot, with an open cardboard pickup carton beside it. The robot closes the cap, presses it through a first light click to the final lock, lifts the container by its handle and sets it upright in the carton. Closing a full container before it is moved is required by OSHA 29 CFR 1910.1030(d)(4)(iii)(A), and the people who do it are some 1.4 million US clinic staff.

**What this task adds.** The container is full of loose needles, so it must stay upright and still while the cap is pressed; a press on an unsteadied 9 in tall tub pushes it along the counter. The cap has two stages: a light press (about 3 N at the cap) gives a temporary click, and the cap can still be opened; only a firm press (about 9.4 N at the cap) on both sides engages the final locks, after which the cap is held shut. A light press therefore stops at the first tier, and the hand has to judge a push that is firm yet does not shove the container. The same container is then lifted by its handle, not the cap, and lowered into the carton and let go without a drop.

**Who does it, and how often.** About 0.8 million medical assistants (O*NET 31-9092.00) and about 0.6 million licensed practical nurses (O*NET 29-2061.00) (BLS Occupational Employment and Wage Statistics, May 2025: https://www.bls.gov/oes/current/oes_nat.htm), plus registered nurses and phlebotomists, close and swap sharps containers whenever one reaches its fill line, in every clinic, ward and blood-draw room. The O*NET Core task list for medical assistants (https://www.onetonline.org/link/summary/31-9092.00) includes "Clean and sterilize instruments and dispose of contaminated supplies." The requirement is OSHA 29 CFR 1910.1030(d)(4)(iii)(A): containers are closed before removal so contents cannot spill in handling or transport (https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.1030). The maker's product page for the BD 305487 describes a 'secure final closure feature' with 'visual, audible, tactile (VAT) perimeter locks' and 'handles designed for safe transport and disposal' (https://www.bd.com/en-us/products-and-solutions/products/product-page.305487); the maker's instructions say to press the closure down firmly until it clicks on each side, after which the cap cannot be reopened.

**How the demo does it.** The left hand grips the moulded handle bar on the container's side. The right hand, fingers closed, pushes the open cap over from its far side, following it down to rest on the two catches (cap shut, no click yet). It then lifts, moves over the catches' line, and presses straight down at about 16 N commanded (about 11 N reached, by the arm's own compliance) until both catches are 6 mm down: the final lock snaps. The left hand lifts the container 18 cm by the handle, carries it over the carton wall, lowers it to the carton floor, opens, backs off the handle and moves clear of the container.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder, the scripted demo (the README command):

```bash
python demokit.py task.py OUT --mode full                                                          # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode none,light_press,no_press,free_press,drop_high,drop_low,drop_2cm,cap_fall,held_fast_lower   # failure checks, below
python check_score.py                                                                              # scoring table and clearance test, no simulator
```

`SC_TRACE=1` prints the cap angle, catch depths, lock state, tilt, carton test, hands-clear distance and score every 20 steps. The demo reads the container's pose and the catch depths from the simulator, as `demokit` allows; a policy gets the cameras only.

**Result** (scripted, GPU physics, 20 Hz, RTX 3090 Ti): 1.0 at step 575 (28.75 s of simulated time), against a limit of 1,600 steps.

**Dream-1** (the Dream-1 model on this task, one trial, 1,600 steps, session held): 0 (the cap never reached the temporary click).

| Failure check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 (the cap stays open) |
| `no_press` | the cap is swung shut and the container carried into the carton, no press | 0 (cap shut, catches 0.2 mm down: no click) |
| `light_press` | as `full`, but the press stops at 4 N | 0.33 (temporary click only; the final lock needs about 9.4 N at the cap) |
| `free_press` | the right hand swings the cap and presses with the container not held, then the left carries it | 0.67 (final-locked, but the container was tilted 8 degrees by the press and the left hand then missed the handle, so it was never lifted into the carton) |
| `drop_high` | as `full`, but the left hand lets go with the container base about 10 cm above the carton floor (measured at release) | 0.67 (final-locked; dropped, so capped below 1.0) |
| `drop_low` | as `drop_high`, but with the base about 3.7 cm above the carton floor (measured at release) | 0.67 (final-locked; dropped) |
| `drop_2cm` | as `drop_low`, but with the base about 2 cm above the carton floor (it slips a little in the opening fingers, so it falls about 1.4 cm free) | 0.67 (final-locked; dropped) |
| `held_fast_lower` (a check mode) | as `full`, but the left hand lowers the gripped container into the carton in 0.7 s (about 0.24 m/s on average, more than 4 mm a step) | 1.0 in 3 of 4 runs (a descent in hand is not a drop: the fingertips stay within 3 mm and the gripper falls with the container; in 1 run the container was flung and scored 0) |
| `cap_fall` | the cap is lifted to 60 degrees and let fall shut under its own weight, no press | 0 (the cap's weight is about 0.4 N; no click) |

One catch alone cannot be pressed in this model: the cap is a single rigid hinged body, so both catches go down together.

## Sizes

| Size | Scene | Source |
|---|---|---|
| Container, cap shut | 127 x 145 x 229 mm | BD 305487 product page and catalogue (5.0 x 5.7 x 9.0 in) |
| Red body / clear collar | 150 mm / to 214.5 mm | estimate from the catalogue photo |
| Handle bar | 80 mm long, 14 mm wide, 18 mm tall, 32 mm off the wall | estimate from the catalogue photo |
| Mass | 0.5 kg | estimate: a container filled to its fill line |
| Cap | rigid hinged body, 100 degrees open at rest, 0.04 kg | estimate |
| Catches | two, 40 mm either side of centre, 700 N/m, 8 mm travel; temporary click 2 mm (3 N at the cap), final lock 6 mm (9.4 N at the cap) | estimates: no real container was measured |
| Pickup carton | 300 x 280 x 120 mm, 5 mm walls | estimate: a medical-waste pickup box |

**Forces in the demo.** Pinch 35 N a finger on the handle; press about 11 N reached at the cap. The final lock's force and the click depths are estimates; a real trial should measure them on a real container (a luggage scale pressed on the cap).

## Real trial

A BD 305487 (or any 1.5 qt hinged-cap sharps container) filled to its fill line with new, needle-free syringes (or an equal weight of plastic) sealed in a bag, never discarded ones, on a counter beside an open box (container about $10-15 (USD, estimate from distributor listings); BD 305487: https://www.bd.com/en-us/products-and-solutions/products/product-page.305487). Score it by eye and by ear: the first click (cap shut), then the second, firmer one; after the trial, pull the cap by hand and check it stays locked with both tabs engaged; a photo from a front and an overhead camera shows the container upright in the box, both grippers off it and still for 1 s, with no free fall after release; a luggage scale on the cap gives the final-lock force.
