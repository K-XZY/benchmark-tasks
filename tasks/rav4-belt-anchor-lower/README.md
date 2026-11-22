# Push a seat belt's shoulder anchor 45 mm up its pillar rail against the belt's pull, clicking past two notches into the marked top one

A taller front-seat occupant raises the shoulder-belt anchor on the car's B-pillar so the belt crosses the middle of the shoulder. The anchor ratchets up with no button, but the belt's retractor pulls it back down the whole way: a hand that stops between notches lets it drop to the last notch passed, and pressing the white release button sends it to the bottom. Here the bought adjuster, a Toyota RAV4's 73200-12070, is bolted to a section of the left B-pillar tilted 10 degrees like the pillar, behind the RAV4's own upper pillar trim, with the belt's webbing routed from the lower trim up through the D-ring and down beside the front seat, and the retractor's pull on it. The anchor starts locked at its second-lowest notch; a strip of tape on the trim marks the top notch. One arm reaches in from the cabin past the trim, sets its fingertips under the raised block above the button, without pressing the button, and pushes the anchor 45 mm up against the pull until the pawl clicks into the marked notch, then eases off and backs away.

## Importance

The United States has 237,655,885 licensed drivers (FHWA Highway Statistics 2023, table DL-22), and Toyota sold 475,193 RAV4s there in 2024 (Toyota Motor North America, 2024 U.S. sales results); each has this height-adjustable anchor at both front seats. Americans make 2.1 trips a day on average (FHWA Mobility Trends briefs, 2022 National Household Travel Survey), and a new occupant, or the same one after a change of driver, sets the anchor for the trip. The RAV4 owner's manual gives the act as "To raise: slide the anchor up" and adds "make sure the anchor is locked in position"; it asks for the belt to cross the centre of the shoulder, so a taller occupant, whose shoulder sits higher, sets the top notch. Sources are listed at the end.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,stay,lower,half,short,hold,lean
```

The harness's instruction (Dream-1 run): `--instruction "Raise the seat belt's shoulder anchor against the belt's pull to the top notch marked on the trim, so it locks there, and clear the hands."`

## Scoring, and the routes that score less

Scores are the harness's task_score on an RTX 3090 Ti.

| Mode | What it does | Score |
|---|---|---|
| full | fingertips under the raised block push the anchor 45 mm up against the belt's pull until the pawl clicks into the marked top notch, the hand eases off and backs away | 1.0 (331 steps, 16.55 s) |
| none | both arms wait | 0.0 |
| stay | the release button pressed and held 5 s with the hand still, then let go | 0.0 (the belt's pull carries the freed anchor down to the lowest notch) |
| lower | the button pressed and the anchor followed down, let go | 0.0 (it locks in the lowest notch) |
| half | pushed up one notch, let go | 0.33 (it stays in the middle notch) |
| lean | the fingers under the block push up with a command about 5.5 N, below the 9.8 N the latch gives way to, held 5 s, let go | 0.0 (the anchor does not move) |
| short | pushed up to 8 mm under the marked notch, let go | 0.33 (the pull takes it back to the notch below the marked one) |
| hold | pushed up to the marked notch, the hand never taken away | 0.67 |
| Dream-1 (model, the harness's instruction) | the anchor and button never move: no press | 0.0 (600 steps, 30 s; one session, no loss) |

The score is the state now, never a best so far. 0.33: the anchor at least 10 mm above its start notch. 0.67: the anchor within 3 mm of the marked top notch. 1.0: the pawl caught in the marked notch, the button out (within 1 mm of its rest), and both whole grippers (fingers, palm and housing, measured against the anchor's real outline) at least 2 cm from the anchor, held 3 s. Void (0): a locked anchor forced more than 3 mm below its notch (checked from 0.5 s after the pawl drops). Real trial: a ruler or side photo for the 10 mm and 3 mm marks.

## Real trial

Bolt a Toyota 73200-12070 adjuster through its rail's two end holes to a 3/4 in birch plywood board (Home Depot internet #206120918, 18 mm, cut to 30 x 59 cm) on two 1-1/2 in aluminium round tube posts (Metals Depot 6061 round tube, 38 mm OD), tilted 10 degrees from upright with its top towards the robot, the stud about 1.16 m above the floor at the top notch. Clip the RAV4's upper centre pillar trim, Toyota 62420-0R060-B1, over it as in the car, the slider in its window, and a lower trim panel (or an 18 mm board edge with a 50 mm slot) below. Put a strip of tape on the trim beside the window at the stud's height in the top notch. Fit a RAV4 front belt assembly, Toyota 73220-06390-C3: its retractor bolted to the board's back near the foot, the webbing up through the slot, through the D-ring bolted on the stud, and down to its lower anchor beside the seat, so the retractor's own spring loads the anchor as in the car. Stand a front seat back's outboard edge and headrest (a car seat, or foam blocks of 16 x 10 x 32 cm and 13 x 8 x 15 cm) forward of the pillar; the act does not touch them. Rig cost, under 300 USD: the adjuster 62 USD new (https://toyotapartsnow.com/oem/toyota~adjuster_assy_fr_sh~73200-12070), a used belt assembly 73220-06390-C3 at 165 USD (a used front driver belt with that part number, https://ahparts.com/buy-used/2021-toyota-camry-front-driver-seat-belt-black-73220-06390-c3-7322006390c3/385216-1), the board and two posts about 40 USD (an estimate), and the upper trim as a 3 mm ABS or PETG sheet cut to the 11 to 17 cm outline with its 50 x 157 mm window, about 15 USD: about 282 USD. The scene uses the real trim 62420-0R060-B1; the sheet gives the same window and the same collision. The anchor starts locked at the second-lowest notch. Success is judged from video and a side photo: after the robot's hands are 2 cm clear for 3 s, the anchor sits at the marked notch (the video shows it at the tape; a ruler or side photo gives 3 mm), and a light downward tug of about 5 N on the anchor does not move it. Reset is pressing the button and letting the belt draw the anchor down to the start notch, about 10 seconds. Nothing is consumed, and nothing in the scene is sharp, hot, wet or fragile enough to harm the rig. The push up is about 10 N in the scene (4 to 22 N over the ranges below).

## Real items

- Toyota front shoulder belt anchor adjuster assembly, part 73200-12070 (steel rail with two bolt holes, slider carrying the anchor stud, white lock release button at the slider's lower edge): https://www.nunawadingtoyotaparts.com.au/toyota-front-shoulder-belt-anchor-adjuster-assembl . Price about 75 USD (AUD 113.86, list AUD 133.47) at that seller. It is listed as suiting the Toyota RAV4 (2019 to 2025, XA50); the scene is built to its photo, https://www.nunawadingtoyotaparts.com.au/assets/full/TO7320012070.jpg.
- Toyota upper centre pillar trim, left, part 62420-0R060-B1 (light grey, with the window the adjuster's slider runs in), listed for the 2019 to 2024 RAV4 at 97.57 USD: https://www.toyotapartsdeal.com/oem/toyota~garnish~assy~ctr~pi~62420-0r060-b1.html . The scene is built to the photo of its mirror-image right-hand part, 62410-0R060-B1: https://www.toyotapartsdeal.com/oem/toyota~garnish~assy~ctr~pi~62410-0r060-b1.html
- Toyota front seat belt assembly, left, part 73220-06390-C3 (retractor, webbing and the D-ring hung on the stud), listed for the 2019 RAV4 at 220.39 USD: https://www.toyotapartsdeal.com/oem-2019-toyota-rav4-seat_belt.html

## Modelling

- The pillar section is a fixed articulation tilted 10 degrees about the across-rail axis (top towards the arms): an 18 mm board, 30 x 59 cm, in place of the pillar's sheet metal, its face 30 cm in front of the arm bases (61 cm apart) at the stud, on two 40 mm aluminium posts. The rail (31 mm wide, 25 cm long over its two end tabs, 8 mm deep) is fixed to it by two bolts through its 15 mm end holes.
- The upper trim is a fixed 3 mm shell standing 2 mm off the rail, 11 cm wide flaring to 17 cm at its foot and 42 cm tall, round a 50 x 157 mm window that clears the slider's whole travel by 2 to 4 mm a side; the lower trim below it is a fixed block, 50 mm deep, with the webbing's slot. Both collide with the hand.
- The anchor (0.22 kg) rides a prismatic joint along the rail with 60 mm of travel, five notches 15 mm apart, starting at the second-lowest, 45 mm below the top notch, and a joint friction of 0.5 N (estimate; range below). It carries the slider (41 x 85 mm, 14 mm deep, riding 0.5 mm off the rail), the raised block above the button (21 x 23 mm, 10 mm proud), the stud's collar and washer (24 mm across, 12 mm), the belt's D-ring (60 mm wide, 54 mm long, a 50 x 10 mm webbing slot, 5 mm thick, rigid, hanging 55 degrees from straight down towards the seat) and its 17 mm nut.
- The belt's pull: on a free anchor (button pressed), a drive pulls it down the rail with a near-constant 6.8 N: the retractor's 3 N on the webbing's retractor side, straight down (3 N), and on its cabin side, leaving the D-ring 55 degrees from straight down (3 N x cos 55 = 1.7 N), plus the anchor's weight along the 10 degree rail (0.22 kg x 9.81 x cos 10 = 2.1 N). The scene's gravity is off for the anchor; its weight is in this pull.
- The webbing (48 mm wide, black) is a rigid stand-in that rides with the anchor, with no collider: it shows the routing, and its load is the pull above. The act does not depend on its shape.
- The latch gives way to a push of 9.8 N in either direction (one symmetric limit); a real pawl holds a downward push on its flat face. The scored act pushes only up; a downward push of more than 3 mm below the notch voids the run, so leaning down scores nothing, and the lean mode (an upward push under the 9.8 N, held) scores 0.
- The release button (white, 21 x 14 mm, 4 mm proud, at the slider's lower edge under the stud) is a prismatic joint on the slider with a 2500 N/m spring, 10 N pressed in full.
- The lock pawl is code, a ratchet. Locked, a 50 kN/m spring holds the anchor at its notch; pushed up, it gives way at 9.8 N, the belt's 6.8 N pull and 3 N for the pawl riding up its notch's ramp (estimate; range below), so the push up meets the pull the whole way and a hand that lets go between notches sees the anchor drawn back to the notch below. Within 1.5 mm of the notch above, the pawl drops into it. Button in at least 2 mm (5 N) lifts the pawl and the anchor slides free under the pull; with the button back within 1 mm of rest, the pawl drops into the first notch the anchor meets within 1.5 mm.
- A 1.5 mm catch window is a fit tolerance for the pawl in its notch; the real value is not published.
- The front seat's back (its outboard 16 cm, 32 cm tall) and headrest stand forward of the pillar, 9 to 12 cm in from the trim, fixed; the rest of the seat, inboard and below, is left out because the arms stand there. The hand's path does not reach them.

## Sizes

The adjuster's sizes are scaled from the maker's part photo by the anchor stud (7/16 in, 11 mm); none is published.

| Part | In the scene |
|---|---|
| rail | 31 mm wide, 25 cm long over its end tabs, 15 mm bolt holes |
| slider | 41 x 85 mm |
| raised block above the button | 21 x 23 mm, 10 mm proud (sized from the maker's photo, an estimate; the real trial measures it with a caliper on the bought adjuster and accepts a block at least 15 mm wide and tall and 6 mm proud, a fingertip's push surface; only the fingers' fit under it depends on it) |
| release button | 21 x 14 mm, 4 mm travel, 10 N (force an estimate) |
| collar and washer, nut | 24 mm across; 17 mm across flats |
| D-ring (part of the belt assembly 73220-06390-C3, not of the 73200-12070 adjuster) | 60 mm wide, 50 x 10 mm slot (estimate for a 48 mm webbing) |
| upper pillar trim 62420-0R060-B1 | 11 to 17 cm wide, 42 cm tall, 3 mm shell; window 50 x 157 mm (scaled from its photo by the slider's window) |
| notch pitch, travel | 15 mm, 60 mm (estimate; range below); start 45 mm below the top notch |
| anchor mass | 0.22 kg (calculation below) |
| pillar tilt, stud height | 10 degrees, 1.16 m at the top notch (rig choices) |
| belt's pull on the anchor | 6.8 N down the rail (retractor 3 N on both webbing sides, plus weight; range below) |
| push to raise a locked anchor | 9.8 N: the pull and a 3 N ramp (range below) |

## Ranges for the estimates

The demo scores 1.0 at both ends of each range below, run together. The low end gives the weakest pull and ramp against the most friction and the longest push: belt 1 N, mass 0.15 kg, ramp 1 N, rail friction 2 N, button 2500 N/m, pitch 20 mm with 80 mm of travel (60 mm pushed), catch 0.5 mm (full 1.0, half 0.33, short 0.33). The high end gives the strongest pull and ramp, 22.4 N to push: belt 7 N, mass 0.35 kg, ramp 8 N, rail friction 0.2 N, button 625 N/m, pitch 15 mm, catch 1.5 mm (every mode run: full 1.0, half 0.33, short 0.33, hold 0.67, lean 0.0, none 0.0, stay 0.0, lower 0.0). Each environment variable named sets one value: `ANCHOR_BELT`, `ANCHOR_RAMP`, `ANCHOR_RAILF`, `ANCHOR_BTNK`, `ANCHOR_PITCH`, `ANCHOR_CATCH`, `ANCHOR_MASS`.

| Value | In the scene | Range run at 1.0 | Bound | Real-trial measurement |
|---|---|---|---|---|
| button force, full press | 10 N (2500 N/m over 4 mm) | 2.5 to 10 N | push-button guidance: 2.5 to 5 N finger-operated, 10 N hand-operated (RoyMech, machine controls table); travel 3.3 +/- 0.76 mm on a comparable military push-button switch (MIL-PRF-22885/101) | push-pull gauge, 0 to 50 N, pressed square to the button; ruler for its travel |
| notch pitch, travel | 15 mm, 60 mm | 15 to 20 mm, 60 to 80 mm | 3 to 5 positions in owner's manuals (Kia: 4, Ford: 5); the rail's clear length above the slider in the maker's photo, about 105 mm, caps the travel | steel ruler or caliper, 0 to 150 mm, from the stud's centre at each notch |
| pawl catch window | 1.5 mm | 0.5 to 1.5 mm | shown calculation: a pawl tooth in a stamped notch with 0.25 to 0.75 mm of clearance a side | with the button out, rock the anchor by hand and read the play on a caliper, 0 to 150 mm |
| rail friction | joint friction 0.5 N | 0.2 to 2 N | shown calculation: a plastic liner on steel (friction coefficient 0.1 to 0.4, Engineering ToolBox) under at most 5 N of sideways load from the hand gives up to 2 N | spring scale, 0 to 10 N, pulling the anchor down the rail with the button held |
| pawl ramp, pushed up | 3 N over the pull | 1 to 8 N | shown calculation: a pawl spring of 1 to 4 N on a 45 degree notch ramp with a friction coefficient of 0.1 to 0.4 (Engineering ToolBox) takes (tan 45 + mu) / (1 - mu tan 45) = 1.2 to 2.3 times the spring force, 1.2 to 9.3 N | spring scale, 0 to 20 N, pushing the locked anchor up the rail with the webbing unhooked, read as it clicks over a notch |
| anchor mass | 0.22 kg (2.1 N of weight along the rail, in the pull) | 0.15 to 0.35 kg | shown calculation: slider folded from 2 mm steel (41 x 85 mm face, two 14 mm sides) 92 g; stud, collar and nut 45 g; D-ring 3 mm steel plate less its slot 64 g with a 10 g cover; button and block 10 g; total 221 g | kitchen scale, 0 to 5 kg, the adjuster weighed whole less the rail's share |
| retractor pull on the webbing | 3 N (6.8 N down the rail with both webbing sides and the weight) | 1 to 7 N (3.0 to 14.4 N down the rail, with the mass range) | FMVSS 209 S4.3(j)(2)(i)(A): an emergency-locking retractor on a lap and shoulder belt exerts a retractive force of not less than 1 N and not more than 7 N | spring scale, 0 to 10 N, holding the webbing just out of the retractor; then a 0 to 20 N scale pushing the locked anchor up the rail with the webbing on |

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
- FMVSS 209, 49 CFR 571.209, S4.3(j) (retractor force): https://www.law.cornell.edu/cfr/text/49/571.209
- Engineering ToolBox, friction coefficients: https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
