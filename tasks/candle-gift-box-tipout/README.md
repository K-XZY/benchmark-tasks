# Tip a glass jar candle out of a gift box too snug to lift it from, then stand it upright on the table

A boxed jar candle is unboxed by hand every time one is bought or given, and the box is cut to the jar: 1.5 mm a side, so no finger fits beside the glass and the candle cannot simply be lifted out; it has to be tipped out. A Diptyque Classic Candle 190 g stands in its open square gift box. The right gripper squeezes the box's two side walls onto the glass, lifts it, turns it over, brings the rim down to the table, loosens the squeeze so the jar slides out onto its rim, lifts the empty box away and sets it down beside it, then takes the jar, turns it upright and stands it on its base.

## Run

From this folder, with the harness's Python on a GPU host:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,no_ease,ease_5cm,ease_2cm,no_upright,upright_drop,slow_release_upright
```

The harness's instruction (Dream-1 run): `--instruction "The candle is too snug in its gift box to lift out: tip it out without dropping it, stand it upright on its base, and set the empty box aside."`

Results on this scene: the scripted demo scores 1.0 (765 steps, 38.25 s); the Dream-1 model scores 0 (1,200 steps, valid run).

## Scoring, and the routes that score less

Both tiers need the box first turned over (tilted past 150 degrees) with the jar still in it and the jar's lowest point at least 3 cm above the table.

- 0.5: the jar is out on the table, on its rim or its base within 10 degrees of vertical and still; every point of the box at least 0.5 cm from it; no part of either gripper (fingers, pads, palm and housing) within 2 cm of it; held for 2 s and still so at the end of the run. A jar left upside down on its rim scores this.
- 1.0: the same with the jar standing on its base, opening up, within 10 degrees of vertical, and the box set down on the table and let go (no gripper within 1 cm of it); held for 2 s and still so at the end of the run.
- Score 0 (drop): the jar comes down more than 3 cm from where it last stood still, both against the table and against the box, with no hand holding the bare glass (both finger pads of one hand within 8 mm of it, the glass riding with the hand: a jar that has slid more than 1 cm down through the closed fingers is no longer held). A box carried with the jar in it moves the jar with it, and a box lifted off the jar leaves it where it is: neither counts. A jar that slides out of a box held high falls against both, so the drop is caught whether the grip was eased or the jar slipped. Score 0 also if the jar leaves the table.

| Route | Score |
|---|---|
| full: squeeze, turn over, rim to the table, ease, lift the box off, set it aside, take the jar, turn it upright, stand it on its base | 1.0 (765 steps, 38.25 s) |
| none: the arm waits | 0 |
| no_ease: the box is turned over and set down still squeezed, so the jar never comes out | 0 |
| ease_5cm: the squeeze is eased with the rim about 5 cm above the table | 0: the jar comes down 4.2 cm (void) |
| ease_2cm: the squeeze is eased with the rim about 2 cm above the table, then the jar is turned upright | 1.0: a 2 cm slide onto the table is not a drop |
| no_upright: the jar is left upside down on its rim | 0.5 |
| upright_drop: the jar is turned upright and let go well above the table | 0: the jar comes down 5.8 cm (void) |
| slow_release_upright: the same height, the fingers opened over 6 s so the jar slides down through them | 0: the jar slides more than 1 cm in the hand, then comes down 3.1 cm (void) |

## Real items

- Diptyque Classic Candle 190 g, Ambre (emea.diptyqueparis.com/en-pf/products/ambre-classic-candle): glass jar 90 mm tall, 77 mm across, 190 g of wax, sold in a square paperboard gift box.

## Simulation

- The jar is one free rigid body, a 77 x 90 mm cylinder collider, 0.50 kg. The box is a free articulation, 0.06 kg: its floor and the two walls the hand does not touch are one rigid body; the two walls the hand squeezes are panels on sliders that move inward up to 3 mm against a spring, so a squeeze presses them onto the glass and an eased squeeze lets them spring back. A real board wall bends rather than slides; the flat panel spreads the hand's force over the whole wall, and the spring takes part of the squeeze before any reaches the glass (at 8 N/mm, 12 N of it at the 1.5 mm gap), so the hold needs at least as much squeeze as the real box. The rigid walls the hand does not touch carry no squeeze in either case.
- The YAM arm is soft at the wrist (10 N m/rad on joints 4 to 6): the turned-over box and jar hang about 2.5 cm below the commanded height, so the demo commands the rim 2 cm below the table top and it comes down onto it, and it commands the upright jar's base 2 cm below the table in the same way. The jar is stood up further out, where the turned hand reaches the table. The demo reads only the arm's own joints; it never reads the jar or the box.
- Physics on the GPU at the harness's rate.
- Range, at each end of the real-trial ranges below: full scores 1.0 at the hard corner (box 81 mm inside, walls 12 N/mm, jar 0.55 kg, glass friction 0.30, board friction 0.35, squeeze 35 N) and at the easy corner (79 mm, 2 N/mm, 0.45 kg, 0.60, 0.60, 40 N), where ease_5cm still scores 0 (the jar comes down 3.1 cm). Outside these ranges the hold can fail: with an 82 mm box, 12 N/mm walls and 40 N the jar slides out mid-turn, so a real box wider than 81 mm is outside the trial. RTX 3090 Ti and RTX 4070 Ti SUPER.

Each estimate has a bound and a check before the trial; the trial runs only with every measured value inside its range.

| Quantity | Sim | Bound or source | Real-trial check and accepted range |
|---|---|---|---|
| Jar size | 77 x 90 mm | maker | none needed |
| Jar mass | 0.50 kg | 190 g wax (maker) + glass by calculation: 4 mm wall, 8 mm base, 120 cm3 at 2.5 g/cm3 = 0.30 kg | kitchen scale, 0.45 to 0.55 kg |
| Box inside | 80 mm square (1.5 mm a side), 100 mm deep | specified: the trial box is 80 +/- 1 mm inside; the maker publishes no box size, so a gift box measuring outside this is replaced by a 1.5 mm paperboard carton folded to 80 mm square inside, 100 mm deep | calliper, 79 to 81 mm |
| Board | 1.5 mm, box 0.06 kg | 0.051 m2 at 0.8 g/cm3 = 0.061 kg | calliper and scale |
| Side wall stiffness at mid-height | 8 N/mm | D = E t^3/12(1-v^2) = 0.9 N m (E 3 GPa, t 1.5 mm); a simply supported 80 x 100 mm plate loaded at its centre gives D/(0.0116 a^2) = 12 N/mm, an upper bound because the open top edge only softens the wall | push gauge on the wall's middle, 2 to 12 N per mm |
| Wall travel | 3 mm | more than the 1 to 2 mm gap to the glass, so the glass ends the squeeze, not the stop | none needed |
| Wall panel solver values | 0.01 kg, damping 5 N s/m, no joint friction | numerical: the panel settles within a control step; the board's friction acts at the contacts | none needed |
| Friction, glass on board | 0.45 | estimate; measured by the TAPPI T 815 inclined-plane method | jar lying on a flat of the same board, tilted until it slides: tan of the angle, 0.30 to 0.60 |
| Friction, board on table and pads | 0.50 | estimate; TAPPI T 815 | same tilt test, 0.35 to 0.60 |
| Squeeze | 40 N commanded | YAM gripper about 40 N (maker figure) | force gauge between the pads, 35 to 40 N |
| Ease | 2.5 N | demo setting | none needed |
| Drop | 3 cm from the last rest | estimate: a thick-walled glass jar survives 3 cm onto its rim on a table | side camera at table height, frame by frame against a 3 cm tape mark |
| Held (no drop while held) | both pads of one hand within 8 mm of the glass, and the glass within 1 cm of its closest approach to the pads' centre | sim only: the pad points read 1 to 6 mm from the glass they touch | video: a hand closed on the glass with the glass moving with it counts; a glass sliding down through the fingers does not |
| Jar grip | 30 N, 3.5 cm above the table on the rim-down jar | demo setting; holds 0.55 kg at glass friction 0.30 with margin | none needed |

## Real trial

Parts: one Diptyque Classic Candle 190 g in its gift box (the boxed candle as sold, about 70 dollars), a table with a hard laminate top, a phone on a stand, a protractor card, masking tape, a calliper, a kitchen scale, a push gauge and a force gauge (each under 30 dollars). The box must measure 79 to 81 mm inside (calliper); if it does not, fold a carton of 1.5 mm paperboard to 80 mm square inside and 100 mm deep, top open, and use that.

Bench (a): the two arms at their usual mounts; positions in cm from the midpoint between the arm bases, x forward, y to the robot's left. Tape a square mark at (40, -14) and stand the box on it, top open, with the jar inside on its base, nothing holding either. The table is clear 25 cm around the mark.

Reset (b, about 1 minute): put the jar back in the box on its base, opening up, and the box back on its mark. Nothing is consumed; the box flexes and springs back.

Bench checks before a session (e): close the pads on the force gauge until the gripper reads 35 to 40 N; jar 0.45 to 0.55 kg on the scale; box 79 to 81 mm inside; each squeezed wall 2 to 12 N per mm with the push gauge at its middle; glass on board 0.30 to 0.60 and board on the table 0.35 to 0.60 by the tilt test in the table above. Run only with each inside its window.

One trial (c): start the camera, run the policy for up to 60 s, stop.

Record (g): the phone on its stand at table height, side-on to the box mark and the area to its left and front, the whole run at 30 fps or more, with a 3 cm tape mark standing beside the box in view.

Score by hand (d): void if any frame shows the jar coming down more than the 3 cm mark from where it last stood still, against the table and against the box, with no hand holding the bare glass (the glass riding with the hand, not sliding down through the fingers), or the jar leaving the table. 0.5 if the box was turned over with the jar in it, at least 3 cm up, and the jar then stands still 2 s on the table and is still so at the end of the run, on its rim or base, the box apart from it (a 5 mm gap; a sheet of 5 mm card passes between) and no gripper within 2 cm. 1.0 if the jar stands on its base, opening up, within 10 degrees of vertical (protractor card held behind it in the end frame), with the box set down on the table and let go, still so at the end of the run.

Care (k): the jar is glass and the void is set below a height it survives; the operator wears safety glasses, and a cracked jar is replaced before the next trial.

## Sources

- Diptyque, Ambre classic candle 190 g (emea.diptyqueparis.com/en-pf/products/ambre-classic-candle): jar sizes and wax mass.
- CivicScience, U.S. adults and fragrances (civicscience.com/u-s-adults-may-seek-fragrances-to-reclaim-their-youth/): more than 4 in 10 U.S. adults regularly buy scented candles.
- National Candle Association, Facts & Figures (candles.org/facts-figures/): about 35% of U.S. candle sales fall in the Christmas/Holiday season; candles are bought as gifts.
- Glass density 2.5 g/cm3 (soda-lime glass, standard value); paperboard modulus about 3 GPa and density about 0.8 g/cm3 (typical folding boxboard; estimates); plate stiffness from Timoshenko and Woinowsky-Krieger, Theory of Plates and Shells, table for a centrally loaded simply supported plate.
- TAPPI T 815, Coefficient of static friction (slide angle) of packaging and packaging materials, inclined plane method (tappi.org): the friction check; the coefficient is the tangent of the slide angle.
- All meshes are built in `task.py`; no third-party meshes are used.

## Licence

MIT
