# Turn a See 'n Say's free arrow onto the goat's 36-degree sector, then pull its sprung lever without moving the 420 g toy

A talking farm toy sits face up on a table. The robot turns its free-spinning arrow from the cat to the goat, a 36-degree picture sector, and pulls the sprung side lever past its trigger so the toy says "goat". The lever pull outweighs the toy's sliding friction, so the toy slides or turns unless one hand holds it down while the other pulls.

## Why it matters

About 18.9 million US children are aged 1 to 5: the sum of ages 1 to 5 in the US Census Bureau's July 2024 national estimates, 18,864,252 ([nc-est2024-agesex-res.csv](https://www2.census.gov/programs-surveys/popest/datasets/2020-2024/national/asrh/nc-est2024-agesex-res.csv)). In a national poll of 710 parents of children aged 1 to 5, "54% [say their child is very likely to engage in] object play such as puzzles or blocks" on a typical day at home ([Mott Poll, Parent Perspectives on Play, October 2025](https://mottpoll.org/sites/default/files/documents/102025_Play.pdf)). Turning an arrow and pulling a lever on a talking toy is one of those object-play acts, done by the child or by the carer beside them.

## Real items

| Item | Maker and model | Link | Size used | Price |
|---|---|---|---|---|
| See 'n Say The Farmer Says | Fisher-Price (Mattel) CBN98 | [shop.mattel.com](https://shop.mattel.com/products/fisher-price-see-n-say-the-farmer-says-cbn98) | barn body 22.9 x 20.3 x 5.1 cm, 420 g (listing) | about US$27 (retail listings) |
| Spring scale (lever force and toy friction) | Pesola LightLine 10 N, item 18010 | [datasheet](https://www.itm.com/pdfs/cache/www.itm.com/18010/datasheet/18010-datasheet.pdf) | 10 N range, 0.1 N marks | about US$20 |
| Spring scale (arrow detents) | Pesola LightLine 1 N, item 18001 | [itm.com](https://www.itm.com/product/pesola-18001-light-line-newton-spring-scale-1n) | 1 N range, 0.05 N marks | about US$20 |
| Protractor | Westcott 6 in (15 cm) clear plastic 180 degree protractor, item 11200 | [staples.com](https://www.staples.com/Westcott-Protractor-Clear-6/product_486928) | 5 degree marks (read to about 1 degree) | about US$2 |
| Tape for toy marks | 3M ScotchBlue Original painter's tape 2090, 0.94 in x 60 yd (item 2090-24NC) | [3m.com](https://www.3m.com/3M/en_US/p/d/B00035531/) | 24 mm wide | about US$5 |

## Scene

Two YAM arms, bases at y +30.5 cm and -30.5 cm, at a wooden table. Coordinates are cm from the midpoint between the arm bases, x forward, y left. The toy lies face up centred at (30, 0), roof away from the robot, on tape marks. Its face has ten 36-degree picture tiles, clockwise from the roof: goose, turkey, mouse, goat, music, quiz, chick, owl, bunny, cat. Each tile shows a picture of its animal (a music note and a question mark on the two game tiles) with its name along the rim, as on the toy. The dial centre sits 1.5 cm toward the robot from the toy's centre. The yellow arrow (6 cm from the centre to its tip, 1 cm tall blade) turns freely with light detents and starts on the cat. The yellow lever hangs on a pivot beside the toy's right edge, 6 cm toward the roof from its centre. Its round smiley head (3.4 cm across) stands 8 cm from the pivot and swings 75 degrees toward the robot against a return spring.

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
| Picture tiles | ten pictures with their names, in the toy's order and sector positions; flat icon art (Twemoji, see Assets and licences) | the maker's own drawings of the same animals, same order | maker's photo | the goat reads as a goat in the robot's cameras; the art style differs |
| Picture sector | 36 degrees; on-goat window +-15 | 36 degrees (ten sectors) | the toy's face | window 3 degrees narrower than the sector: slightly harder |
| Arrow | 60 mm to tip, 10 mm blade, hub radius 17 mm | similar | estimate from the maker's photo | neutral |
| Arrow detents | joint friction 0.02 N m (tested 0.01 to 0.04), damping 0.01 | light clicks; measured in the real trial | estimate | no snap to a sector centre in sim: harder |
| Lever stroke | 75 degrees (tested 60 to 90) | measured in the real trial | estimate from the maker's photo | mid of the bound |
| Trigger | 90% of the stroke | the toy speaks; measured as an angle in the real trial | estimate | mid |
| Lever spring | 4.4 N at the head at the trigger (tested 4 to 8 N); torsion 0.30 N m/rad, 4.4 = 0.30 x 1.178 rad / 0.080 m | measured in the real trial | estimate | the pull stays above the toy's sliding friction across the whole bound |
| Lever head | round, 34 mm across, 20 mm tall, 80 mm from the pivot | round smiley head, 34 mm across | read off the maker's photo against the 203 mm body width | none |
| Toy on table friction | 0.4 (tested 0.3 to 0.5); sliding force 1.65 N | plastic on wood; measured in the real trial | estimate | mid |
| Marks | 30 mm, 10 degrees | tape marks | rule | - |
| Hands clear | 50 mm, whole gripper | rule | rule | - |
| Lever home | within 5 degrees | rule | rule | - |
| Tip void | up axis below 0.9 (25.8 degrees) | rule | rule | - |

Bounds. The maker publishes no lever force, stroke or detent torque, so each is an estimate with a stated range, measured on the real toy before trials (Real trial, bench checks). The force range is set by a shown calculation: at 420 g (4.12 N weight) and plastic-on-wood friction 0.3 to 0.5, the toy starts to slide at 1.2 to 2.1 N, and the range 4 to 8 N lies above that, so across the whole range the pull moves an unheld toy. The 4 N floor is the lowest force at which the two-hand hold stays necessary (below it a light pull still plays the goat without the hold; the no-hold route scores 0.33 or less at 4 N and above); most units are expected to lie inside 4 to 8 N, but no unit has been measured, so the bench check decides, and a toy that triggers below 4 N is replaced. The sim's 4.4 N is the head force at the trigger: torsion 0.30 N m/rad times 1.178 rad (67.5 degrees), over the 0.080 m lever. Every scored threshold is an angle, a distance or a time, not a force, so the scorer reads the same across the bound. The reference demo was run at both corners of the bounds (below).

## Real trial

Setup. A wooden table, two YAM arms as above. Put the toy face up, roof away from the robot, its centre 30 cm in front of the arm-base midpoint. Tape its outline (two corners and the roof tip) on the table. Stick a protractor under the lever pivot, 0 at the lever's rest. Put a small 10 mm scale bar on the table in the overhead camera's view. Put a second camera 60 cm from the toy at the side, level with the tabletop, also with a 10 mm scale bar in view, for the tip and hands-clear reads.

Bench checks before trials. Each lever value is checked on the bought toy against the window the simulation was run across; a toy outside any window is rejected and replaced by another unit.

| Check | Simulation | Accepted window | Tool | How |
|---|---|---|---|---|
| Lever stroke | 75 degrees | 60 to 90 degrees | Westcott plastic protractor (5 degree marks, read to about 1 degree), US$2 | pull the head slowly from rest to its stop; read the arm's angle at the stop |
| Trigger point | 67.5 degrees, 90% of the stroke | 85 to 95% of the measured stroke | same protractor and the toy's own sound | pull slowly; read the angle at which the toy starts to speak; divide by the stroke |
| Lever force | 4.4 N at the head at the trigger | 4 to 8 N | Pesola LightLine 10 N spring scale, item 18010 (0.1 N marks, resolves the 4 to 8 N window), US$20 | hook the scale on the head, pull at right angles to the arm slowly (about 10 s for the stroke) while a phone films the scale and protractor at 30 fps; read the force on the frame where the toy speaks (the scale's pointer has no peak hold) |

- Toy friction: pull the toy across the table with the Pesola 10 N spring scale (0.1 N marks) at its base; read the force when it starts to slide and divide by 4.12 N (its weight). Accept 0.3 to 0.5. A table outside this window needs a different surface.
- Arrow detents: hang the Pesola 1 N spring scale (0.05 N marks) on the arrow tip (60 mm from the centre) and read the force that turns it one sector. Accept 0.17 to 0.67 N (0.01 to 0.04 N m).

Reset. Lay the toy on its tape marks, turn the arrow to the cat, let the lever spring home. Protect the grippers with the YAM's standard pads; nothing else is needed.

Steps of one trial. Start the overhead camera and the side camera, both at 30 frames per second, the whole toy and the scale bar in view of each, and a microphone. Give the instruction, run for 60 s, stop.

Scoring by hand, from the recording:
- 0.33: on the final frame, the arrow lies within 15 degrees of the goat's sector centre (read with a printed 10-degree overlay on the dial).
- 0.67: the toy says "goat" while the toy is within 3 cm and 10 degrees of its tape marks (read on the frame of the sound), and the arrow is still on the goat at the end.
- 1.0: as 0.67, and for the last 2 s both grippers stay at least 5 cm from the toy (read against the scale bar), the lever is home and the toy is on its marks.
- 0: the toy tips more than 25 degrees off face up (a side view, read with a printed angle overlay on the side-camera frame) or leaves the table, at any point.

## Assets and licences

The dial's ten picture icons are from Twemoji 15.1 (https://github.com/jdecked/twemoji), copyright Twitter, Inc and other contributors, licensed under CC BY 4.0 (https://creativecommons.org/licenses/by/4.0/). They were scaled and placed on the tiles in `dial.png`, with the tile colours and names added. The label font is the default font bundled with Pillow.

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

Range corners of the reference demo: low (4 N, 60 degrees, friction 0.5, detent 0.01 N m) full 1.0, no_pin 0.33; high (8 N, 90 degrees, friction 0.3, detent 0.04 N m) full 1.0, no_pin 0.33.

## Run

From this folder:

```
python demokit.py task.py OUT --mode full
```

Reference demo: 1.0, success at step 426 (21.3 s), on two RTX 4070 Ti SUPER runs. A run with the harness's Dream-1 model (RTX 3090 Ti, 1,200 steps, one session, no session lost) scored 0.0: no gripper touched the arrow or the lever, and the arrow never reached the goat. Range knobs: `SS_LEVER_N`, `SS_STROKE`, `SS_MU`, `SS_DETENT`.

## Licence

MIT
