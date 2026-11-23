# Turn a See 'n Say's free arrow onto the goat's 36-degree sector, then pull its sprung lever without moving the 420 g toy

A talking farm toy sits face up on a table. The robot turns its free-spinning arrow from the cat to the goat, a 36-degree picture sector, and pulls the sprung side lever past its trigger so the toy says "goat". The lever pull outweighs the toy's sliding friction, so the toy slides or turns unless one hand holds it down while the other pulls.

## Why it matters

About 18.9 million US children are aged 1 to 5: the sum of ages 1 to 5 in the US Census Bureau's July 2024 national estimates, 18,864,252 ([nc-est2024-agesex-res.csv](https://www2.census.gov/programs-surveys/popest/datasets/2020-2024/national/asrh/nc-est2024-agesex-res.csv)). In a national poll of 710 parents of children aged 1 to 5, "54% [say their child is very likely to engage in] object play such as puzzles or blocks" on a typical day at home ([Mott Poll, Parent Perspectives on Play, October 2025](https://mottpoll.org/sites/default/files/documents/102025_Play.pdf)). Turning an arrow and pulling a lever on a talking toy is one of those object-play acts, done by the child or by the carer beside them.

## Real items

| Item | Maker and model | Link | Size used | Price |
|---|---|---|---|---|
| See 'n Say The Farmer Says | Fisher-Price (Mattel) CBN98 | [shop.mattel.com](https://shop.mattel.com/products/fisher-price-see-n-say-the-farmer-says-cbn98) | barn body 22.9 x 20.3 x 5.1 cm, 420 g (listing) | about US$27 (retail listings) |

## Scene

Two YAM arms, bases at y +30.5 cm and -30.5 cm, at a wooden table. Coordinates are cm from the midpoint between the arm bases, x forward, y left. The toy lies face up centred at (30, 0), roof away from the robot, on tape marks. Its face has ten 36-degree picture sectors, clockwise from the roof: goose, turkey, mouse, goat, music, quiz, chick, owl, bunny, cat. The dial centre sits 1.5 cm toward the robot from the toy's centre. The yellow arrow (6 cm from the centre to its tip, 1 cm tall blade) turns freely with light detents and starts on the cat. The yellow lever hangs on a pivot beside the toy's right edge, 6 cm toward the roof from its centre. Its round smiley head (3.4 cm across) stands 8 cm from the pivot and swings 75 degrees toward the robot against a return spring.

## Scoring

The scorer reads the toy every step (`judge` in `task.py`; `check_score.py` runs 16 hand-made cases through it).

| Tier | Condition |
|---|---|
| 0.33 | Arrow within 15 degrees of the goat's sector centre. |
| 0.67 | The lever passed its trigger (90% of the stroke, 67.5 of 75 degrees) at a moment when the arrow was on the goat and the toy was face up within 3 cm and 10 degrees of its marks (the toy plays the goat), and the arrow is still on the goat. |
| 1.0 | As 0.67, and the lever back within 5 degrees of home, the arrow on the goat, the toy on its marks, and every point of both grippers (fingers, pads, palm and housing) at least 5 cm from the toy, all held for 2 s (40 steps at 20 Hz). |
| 0 (void) | The toy tips more than 25 degrees from face up (the code reads the toy's up axis below 0.9), or leaves the table. Holds for the rest of the run. |

Run limit 1,200 steps (60 s at 20 Hz). Score at the end of the run.

## Fidelity

Sound is not simulated: in the sim the toy plays the goat when the lever passes 90% of its stroke with the arrow on the goat and the toy on its marks. On the real toy the 0.67 gate is that the toy says "goat".

| Quantity | Sim | Real value or bound | Source | Effect on difficulty |
|---|---|---|---|---|
| Body | 229 x 203 x 51 mm, barn outline | same | maker listing | none |
| Mass | 420 g | 420 g | maker listing | none |
| Picture sector | 36 degrees; on-goat window +-15 | 36 degrees (ten sectors) | the toy's face | window 3 degrees narrower than the sector: slightly harder |
| Arrow | 60 mm to tip, 10 mm blade, hub radius 17 mm | similar | estimate from the maker's photo | neutral |
| Arrow detents | joint friction 0.02 N m (tested 0.01 to 0.04), damping 0.01 | light clicks; measured in the real trial | estimate | no snap to a sector centre in sim: harder |
| Lever stroke | 75 degrees (tested 60 to 90) | measured in the real trial | estimate from the maker's photo | mid of the bound |
| Trigger | 90% of the stroke | the toy speaks; measured as an angle in the real trial | estimate | mid |
| Lever spring | 4.4 N at the head at the trigger (tested 3 to 8 N); torsion 0.30 N m/rad, 4.4 = 0.30 x 1.178 rad / 0.080 m | measured in the real trial | estimate | the pull stays above the toy's sliding friction across the whole bound |
| Lever head | round, 34 mm across, 20 mm tall, 80 mm from the pivot | round smiley head, 34 mm across | read off the maker's photo against the 203 mm body width | none |
| Toy on table friction | 0.4 (tested 0.3 to 0.5); sliding force 1.65 N | plastic on wood; measured in the real trial | estimate | mid |
| Marks | 30 mm, 10 degrees | tape marks | rule | - |
| Hands clear | 50 mm, whole gripper | rule | rule | - |
| Lever home | within 5 degrees | rule | rule | - |
| Tip void | up axis below 0.9 (25.8 degrees) | rule | rule | - |

Bounds. The maker publishes no lever force, stroke or detent torque, so each is an estimate with a stated range, measured on the real toy before trials (Real trial, bench checks). The force range is set by a shown calculation: at 420 g (4.12 N weight) and plastic-on-wood friction 0.3 to 0.5, the toy starts to slide at 1.2 to 2.1 N, and the range 3 to 8 N lies above that, so across the whole range the pull moves an unheld toy. The sim's 4.4 N is the head force at the trigger: torsion 0.30 N m/rad times 1.178 rad (67.5 degrees), over the 0.080 m lever. Every scored threshold is an angle, a distance or a time, not a force, so the scorer reads the same across the bound. The reference demo was run at both corners of the bounds (below).

## Real trial

Setup. A wooden table, two YAM arms as above. Put the toy face up, roof away from the robot, its centre 30 cm in front of the arm-base midpoint. Tape its outline (two corners and the roof tip) on the table. Stick a paper protractor under the lever pivot, 0 at the lever's rest. Put a small 10 mm scale bar on the table in the overhead camera's view. Put a second camera 60 cm from the toy at the side, level with the tabletop, also with a 10 mm scale bar in view, for the tip and hands-clear reads.

Bench checks before trials, each with its accepted window:
- Lever force: hook a luggage scale (0.1 N resolution) on the lever head and pull it tangentially; read the force when the toy starts to speak. Accept 3 to 8 N.
- Stroke and trigger: with the protractor, read the full stroke (accept 60 to 90 degrees) and the angle at which the toy speaks (accept 85 to 95% of the stroke).
- Toy friction: pull the toy across the table with a spring scale at its base; read the force when it starts to slide and divide by 4.12 N (its weight). Accept 0.3 to 0.5. A table outside this window needs a different surface.
- Arrow detents: hang a small spring scale on the arrow tip (60 mm from the centre) and read the force that turns it one sector. Accept 0.15 to 0.7 N (0.01 to 0.04 N m).

Reset. Lay the toy on its tape marks, turn the arrow to the cat, let the lever spring home. Protect the grippers with the YAM's standard pads; nothing else is needed.

Steps of one trial. Start the overhead camera and the side camera, both at 30 frames per second, the whole toy and the scale bar in view of each, and a microphone. Give the instruction, run for 60 s, stop.

Scoring by hand, from the recording:
- 0.33: on the final frame, the arrow lies within 15 degrees of the goat's sector centre (read with a printed 10-degree overlay on the dial).
- 0.67: the toy says "goat" while the toy is within 3 cm and 10 degrees of its tape marks (read on the frame of the sound), and the arrow is still on the goat at the end.
- 1.0: as 0.67, and for the last 2 s both grippers stay at least 5 cm from the toy (read against the scale bar), the lever is home and the toy is on its marks.
- 0: the toy tips more than 25 degrees off face up (a side view, read with the protractor) or leaves the table, at any point.

## Lazy routes

Each route was run on the current scene with `--no-record`.

| Mode | What it does | Score |
|---|---|---|
| full | reference demo | 1.0 |
| none | nothing | 0.0 |
| cat | pulls the lever with the arrow on the cat | 0.0 |
| spin | turns the arrow past the goat, then pulls | 0.0 |
| half | pulls the lever halfway | 0.33 |
| no_pin | pulls without holding the toy down | 0.33 (the toy turns 31 degrees and slides 17 mm off its marks before the trigger) |

Range corners of the reference demo: low (3 N, 60 degrees, friction 0.5, detent 0.01 N m) full 1.0, no_pin 0.67 (the light pull plays the goat, but the toy ends 21 degrees off its marks); high (8 N, 90 degrees, friction 0.3, detent 0.04 N m) full 1.0, no_pin 0.33.

## Run

```
python demokit.py task.py OUT --mode full
```

Reference demo: 1.0, success at step 426 (21.3 s), RTX 4070 Ti SUPER. A run with the harness's Dream-1 model (RTX 3090 Ti, 1,200 steps, one session, no session lost) scored 0.0: neither gripper reached the arrow or the lever (closest gripper-to-toy gap 13 mm at step 240, 116 mm at the end). Range knobs: `SS_LEVER_N`, `SS_STROKE`, `SS_MU`, `SS_DETENT`.
