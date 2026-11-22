# Hold a spring release button in through a 60 mm slide down a pillar rail past three lock notches, then let go so the lock catches the lowest one

A front-seat occupant lowers the shoulder-belt anchor on the car's B-pillar so the belt crosses the shoulder, not the neck. The anchor moves only while the white release button stays pressed: let it go part way and the lock drops into the next notch. Here the bought adjuster, a Toyota RAV4's 73200-12070, is bolted through its rail's two end holes to a bench board tilted 10 degrees like the pillar, locked at its top notch. One arm presses the button with its fingertips, carries the anchor down by the raised block above the button to the lowest notch, then backs off so the button springs out and the lock catches there.

## Importance

The United States has 237,655,885 licensed drivers (FHWA Highway Statistics 2023, table DL-22), and Toyota sold 475,193 RAV4s there in 2024 (Toyota Motor North America, 2024 U.S. sales results); each has this height-adjustable anchor at both front seats. Americans make 2.1 trips a day on average (FHWA Mobility Trends briefs, 2022 National Household Travel Survey), and a new occupant, or the same one after a change of driver, sets the anchor for the trip. The RAV4 owner's manual gives the act as "To lower: Push in the lock release button and slide the anchor down" and adds "After adjustment, make sure the anchor is locked in position." Sources are listed at the end.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,no_button,tap,half,short,hold
python check_score.py                              # the scorer's tiers on hand-built records, no simulator
```

The harness's instruction (Dream-1 run): `--instruction "Lower the seat belt's shoulder anchor to its lowest notch: hold its release button in while sliding it down, let go so it locks, and clear the hands."`

## Scoring, and the routes that score less

Scores are the harness's task_score on an RTX 3090 Ti.

| Mode | What it does | Score |
|---|---|---|
| full | fingertips press the button in, carry the anchor down 60 mm to the lowest notch, the hand backs off, the button springs out, the lock catches | 1.0 (344 steps, 17.2 s) |
| none | both arms wait | 0.0 |
| no_button | the anchor is pushed down from on top of the slider, the button never pressed | 0.0 (the locked anchor does not move) |
| tap | the button pressed and let go, no slide | 0.0 |
| half | pressed, slid two notches, let go | 0.33 |
| short | pressed, slid to 52 mm, let go 8 mm above the lowest notch | 0.33 (the lock catches only within 1.5 mm of a notch; let go at 52 mm, the anchor stays free between notches) |
| hold | pressed and slid to the bottom, the button never let go | 0.67 |
| Dream-1 (model, the harness's instruction) | the anchor and button never move: no press | 0.0 (600 steps, 30 s; one session, no loss) |

The score is the state now, never a best so far. 0.33: the anchor at least 10 mm below its top notch. 0.67: the anchor within 3 mm of the lowest notch. 1.0: the button back out (within 1 mm of its rest), the lock caught in the lowest notch, and both whole grippers (fingers, palm and housing, measured against the anchor's real outline) at least 2 cm from the anchor, held 3 s. Void (0): a locked anchor forced more than 3 mm off its notch (checked from 0.5 s after the lock catches). Real trial: a ruler or side photo for the 10 mm and 3 mm marks.

## Real trial

Bolt a Toyota 73200-12070 adjuster through its rail's two end holes to a 3/4 in birch plywood board (Home Depot internet #206120918, about 18 mm, cut to 30 x 42 cm) on two 1-1/2 in aluminium round tube posts (Metals Depot 6061 round tube, 38 mm OD, about 40 mm modelled), the rig under 100 USD in all, tilted 10 degrees from upright with its top towards the robot, the stud about 1.16 m above the floor; fit the D-ring guide (the slotted loop of a Toyota front seat belt assembly, listed for the 2019 RAV4 as 73220-06390-C3 at https://www.toyotapartsdeal.com/oem-2019-toyota-rav4-seat_belt.html; not part of the 73200-12070 adjuster) on the stud. The anchor starts at the top notch. Success is judged from video and a side photo: after the robot's hands are 2 cm clear for 3 s, the anchor sits at the lowest notch (the video shows it there; a ruler or side photo gives 3 mm), and a light upward tug of about 5 N on the anchor does not move it. Reset is lifting the anchor back up with the button held, about 10 seconds. Nothing is consumed, and nothing in the scene is sharp, hot, wet or fragile enough to harm the rig. The button needs about 10 N and the slide under 20 N (estimates), well inside the YAM's roughly 40 N grip and 20 N push.

## Real items

- Toyota front shoulder belt anchor adjuster assembly, part 73200-12070 (steel rail with two bolt holes, slider carrying the anchor stud, white lock release button at the slider's lower edge): https://www.nunawadingtoyotaparts.com.au/toyota-front-shoulder-belt-anchor-adjuster-assembl . Price about 75 USD (AUD 113.86, list AUD 133.47) at that seller. It is listed as suiting the Toyota RAV4 (2019 to 2025, XA50); the scene is built to its photo, https://www.nunawadingtoyotaparts.com.au/assets/full/TO7320012070.jpg.
- The D-ring guide hung on the stud is not in that photo or part: it is the slotted loop of the belt assembly, Toyota 73220-06390-C3 (listed for the 2019 RAV4 at https://www.toyotapartsdeal.com/oem-2019-toyota-rav4-seat_belt.html).

## Modelling

- The board is a fixed articulation tilted 10 degrees about the across-rail axis (top towards the arms): 18 mm plywood, 30 x 42 cm, its face 30 cm in front of the arm bases (61 cm apart) at the stud, on two 40 mm aluminium posts. The rail (31 mm wide, 25 cm long over its two end tabs, 8 mm deep) is fixed to it by two bolts through its 15 mm end holes.
- The anchor (0.22 kg; its weight is left out, gravity off, so the press alone moves it and the act is harder than real) rides a prismatic joint along the rail with 60 mm of travel, five notches 15 mm apart, and a joint friction of 0.5 N (estimate; range below). It carries the slider (41 x 85 mm, 14 mm deep, riding 0.5 mm off the rail), the raised block above the button (21 x 23 mm, 10 mm proud), the stud's collar and washer (24 mm across, 12 mm), the belt's D-ring guide (60 mm wide, 54 mm long, a 50 x 10 mm webbing slot, 5 mm thick, rigid, hanging 55 degrees from straight down towards the seat side, where the webbing runs to the shoulder) and its 17 mm nut. The webbing is left out.
- The release button (white, 21 x 14 mm, 4 mm proud, at the slider's lower edge under the stud) is a prismatic joint on the slider with a 2500 N/m spring, 10 N pressed in full.
- The lock pawl is code. Button in at least 2 mm (5 N) lifts it and the anchor slides free. With the button back within 1 mm of rest, the pawl catches a notch only when the anchor is within 1.5 mm of it, holding with a 50 kN/m spring capped at 2000 N.
- A 1.5 mm catch window is a fit tolerance for the pawl in its notch; the real value is not published.

## Sizes

All sizes are scaled from the maker's part photo by the anchor stud (7/16 in, 11 mm); none is published.

| Part | In the scene |
|---|---|
| rail | 31 mm wide, 25 cm long over its end tabs, 15 mm bolt holes |
| slider | 41 x 85 mm |
| raised block above the button | 21 x 23 mm, 10 mm proud |
| release button | 21 x 14 mm, 4 mm travel, 10 N (force an estimate) |
| collar and washer, nut | 24 mm across; 17 mm across flats |
| D-ring guide (the loop of the belt assembly, not part of the 73200-12070 adjuster) | 60 mm wide, 50 x 10 mm slot (estimate for a 48 mm webbing) |
| notch pitch, travel | 15 mm, 60 mm (estimate; range below) |
| anchor mass | 0.22 kg (calculation below) |
| board tilt, stud height | 10 degrees, 1.16 m at the top notch (bench choices) |

## Ranges for the estimates

The demo scores 1.0 at both ends of each range below, run together: low end (rail friction 0.2 N, button 625 N/m, pitch 15 mm, catch 0.5 mm, mass 0.15 kg) and high end (rail friction 2 N, button 2500 N/m, pitch 20 mm with 80 mm of travel, catch 1.5 mm, mass 0.35 kg). Each environment variable named sets one value: `ANCHOR_RAILF`, `ANCHOR_BTNK`, `ANCHOR_PITCH`, `ANCHOR_CATCH`, `ANCHOR_MASS`.

| Value | In the scene | Range run at 1.0 | Bound | Real-trial measurement |
|---|---|---|---|---|
| button force, full press | 10 N (2500 N/m over 4 mm) | 2.5 to 10 N | push-button guidance: 2.5 to 5 N finger-operated, 10 N hand-operated (RoyMech, machine controls table); travel 3.3 +/- 0.76 mm on a comparable military push-button switch (MIL-PRF-22885/101) | push-pull gauge, 0 to 50 N, pressed square to the button; ruler for its travel |
| notch pitch, travel | 15 mm, 60 mm | 15 to 20 mm, 60 to 80 mm | 3 to 5 positions in owner's manuals (Kia: 4, Ford: 5); the rail's clear length above the slider in the maker's photo, about 105 mm, caps the travel | steel ruler or caliper, 0 to 150 mm, from the stud's centre at each notch |
| pawl catch window | 1.5 mm | 0.5 to 1.5 mm | shown calculation: a pawl tooth in a stamped notch with 0.25 to 0.75 mm of clearance a side | with the button out, rock the anchor by hand and read the play on a caliper, 0 to 150 mm |
| rail friction | joint friction 0.5 N | 0.2 to 2 N | shown calculation: a plastic liner on steel (friction coefficient 0.1 to 0.4, Engineering ToolBox) under at most 5 N of sideways load from the press gives up to 2 N | spring scale, 0 to 10 N, pulling the anchor down the rail with the button held |
| anchor mass (weight left out) | 0.22 kg, inertia only | 0.15 to 0.35 kg | shown calculation: slider folded from 2 mm steel (41 x 85 mm face, two 14 mm sides) 92 g; stud, collar and nut 45 g; D-ring 3 mm steel plate less its slot 64 g with a 10 g cover; button and block 10 g; total 221 g | kitchen scale, 0 to 5 kg, the adjuster weighed whole less the rail's share (or the spring scale above with the board upright; the sim leaves the weight out, so the real anchor's 0.4 N of weight along the rail only helps) |

## Sources

- Toyota RAV4 Owner's Manual, shoulder belt anchor: https://www.manualslib.com/manual/1155021/Toyota-Rav4.html?page=56
- 2024 RAV4 Quick Reference Guide: https://assets.sia.toyota.com/publications/en/om-s/OM24V1QRG/pdf/OM24V1QRG.pdf
- FHWA Highway Statistics 2023, table DL-22: https://www.fhwa.dot.gov/policyinformation/statistics/2023/dl22.cfm
- Toyota Motor North America, 2024 U.S. sales results: https://pressroom.toyota.com/toyota-motor-north-america-reports-2024-u-s-sales-results/
- Toyota part 73200-12070: link under Real items.
- RoyMech, machine interface controls and ergonomic design data: https://www.roymech.co.uk/Useful_Tables/Ergonomics/Human_controls.html
- MIL-PRF-22885/101 switch listing (actuation travel 0.130 +/- 0.030 in): https://switches.emilspec.com/MIL-PRF-22885-101/page6.html
- Kia owner's manual, shoulder belt height (4 positions): https://ownersmanual.kia.com/full_webhelp/MQ4/2023/en_GB/topics/t00045.html
- Ford Villager owner's guide, shoulder belt height adjustment (5 positions): https://www.manualslib.com/manual/575345/Ford-Villager.html?page=19
- FHWA Mobility Trends briefs (2.1 daily trips per person, 2022): https://www.fhwa.dot.gov/policy/otps/transportation_travel_trends_briefs.cfm
- Birch plywood, 3/4 in: https://www.homedepot.com/p/206120918
- Aluminium round tube, 6061: https://www.metalsdepot.com/aluminum-products/aluminum-round-tube
- Engineering ToolBox, friction coefficients: https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
