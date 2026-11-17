# Lock a mailer's roll end: seat both tabs of the rolled inner panel in the base holes while one hand holds the wall up

Shipping, receiving, and inventory clerks (O*NET 43-5071.00; 816,870 US jobs, BLS OEWS May 2025) have the Core task 'Pack, seal, label, or affix postage to prepare materials for shipping, using hand tools, power tools, or postage meter.' Much of it goes in one-piece die-cut mailers that lock without tape or glue (FEFCO 0427). The lock is the roll end: Packlane's guide: 'snap each side panel into the holes in the bottom of the box, continuing the same motion.' This task is only that lock. The tuck flaps are already folded in behind the end wall, and they and the wall's crease spring it back open, so one hand must hold the wall upright while the other rolls the wall's inner panel over a double crease, two creases a wall's thickness apart, and pushes it down inside the box blind, so that its two tabs find their holes along the base crease. Rolled at one end only, the panel skews; pushed too hard, the board crushes. O*NET is a trademark of USDOL/ETA; task text used under CC BY 4.0.

**Instruction.** Lock the near end of the mailer box, both tabs seated in the holes in the base and the wall's top flat: hold the end wall upright, roll its inner panel over the top and down inside until both tabs snap in, then move both hands clear.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Lock the near end of the mailer box, both tabs seated in the holes in the base and the wall's top flat: hold the end wall upright, roll its inner panel over the top and down inside until both tabs snap in, then move both hands clear."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                              # full score, with a video
python demokit.py task.py OUT --no-record --mode unheld,onehand        # the check runs below
python check_score.py                                                  # the scorer on hand-built states, no simulator
```

The left hand comes round the unrolled inner panel and presses the end wall from 75 degrees to upright onto its stop, and holds it. The right hand pushes the panel over the top with its fingers' ends, then down inside, following it; near 157 degrees its straight-down fingers can go no further, so it backs off and rises with its fingers tilting forward, and the back of the hand presses the panel's top flat. Both tabs snap into their holes, the end locks, and the hands leave. The box is free on the table: the left hand's push slides it about 3.5 cm, and sliding the box is not penalised; no hand or stop braces it, and the policy must finish the lock on a box that moves. The demo reads the box's pose and joints and never moves an object except through the grippers. It takes 383 steps (19 s at 20 Hz).

## The scene and what is simplified

Board panels are rigid plates on sprung hinges; crease forces are simulation settings, not measurements. The double crease is one hinge, the rolled end 17.5 mm thick (Uline: outside less inside). A tab's give is a 5 mm sprung slide; pressed 2 mm into its hole it latches, and with both in the end is held locked, as seated tabs and trapped flaps hold a real one. The simulator's non-twisting panel omits cardboard skew: a real inner panel can skew and seat one tab before the other, and this limitation stays. Damage (a torn tab, a new fold) cannot happen in the rigid-panel model, so only a real trial can score it. The box is free: 0.45 kg, friction 0.4 with the table (a setting, 0.38 static for board on board).

- Box: a FEFCO 0427 mailer in B-flute board 3.2 mm thick, outside 34.0 x 31.4 x 15.9 cm, 0.45 kg (Uline S-16934's outside size and weight); its near end is the task, the far end is already locked, the open lid lies to the robot's left.
- Friction: 0.4 between box and table (the table's 0.5 averaged with the box's 0.3).
- Crease settings (simulation settings, no published figure): end wall 3 N at its top edge to hold upright; inner panel 0.2 N at its free edge to hold flat; each tab 0.45 N when pressed back 3 mm.

## Scoring

Measured at the end with photos from the front and from above, a ruler and a square; the tabs checked from underneath. Score 0, and it stays 0 for the rest of the trial, if the box at any time leaves the bench (its middle goes past the bench top's edge or falls below it) or the board is damaged: a tab torn off or bent past 45 degrees from its panel, a new fold or tear in any panel away from the die-cut creases, a dent deeper than 2 mm, or a tuck flap swung out more than 30 degrees. Sliding the empty box, turning it or tipping it does not score 0 by itself; the rungs below are read in the box's own frame. Otherwise the rungs are cumulative: 0.25 with the inner panel rolled over the top (folded past 100 degrees), its free edge inside the box and below the wall's top; 0.5 with the inner panel within 10 degrees of the outer panel and its free edge within 1 cm of the base; 0.75 with at least one tab in its hole; 1.0 with both tabs in their holes, the wall's top flat (both creases of the rolled top within 5 mm of the same height, by ruler: Packlane's check, 'the tops of the side walls are flat, not slanted or peaked'), the wall within 5 degrees of upright with both tuck flaps inside it (within 10 degrees of folded in), both grippers at least 3 cm from the box, and nothing moving more than 1 mm over 1 s. A tab is in its hole when its tip is at least 2 mm down into the base board, inside the hole's outline: in a real trial the box is tipped up after the photos and the seated tab's tip is seen in its hole from underneath.

Failures latch: once the box has left the bench or a flap has been swung out, the score stays 0 (in simulation the scorer reads these two; a torn tab or a new fold cannot happen in the rigid-panel model, so they are checked in a real trial only). A tab latches only with the inner panel rolled to 150 degrees or more, and only while no failure has happened.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps | 0.0 |
| `full` | The whole task (the film) | 1.0 |
| `unheld` | The left hand stands the wall up, holds it, lets go | the wall springs back to 75 degrees |
| `onehand` | The right hand alone rolls the panel, no hand on the wall | 0.0 |

`check_score.py` feeds hand-built runs through the scorer: each rung, each rule with a case that fails it, the latched failures (box off the bench and back, a tuck flap forced out and back), the box slid, turned or tipped with the lock done still scoring 1.0, and a tab poked in with the panel unrolled.

Not scored: a palm or forearm resting on the box (hands clear is read from the fingertips, as everywhere in the kit).

## Runtime and seeds

The demo takes 383 steps, 19 s at 20 Hz. There is no randomness: `object_jitter=0`.

## Building it for a real trial

Use Uline S-16934 as bought, on a laminate or wood bench, its far end already locked and its near end flat and unrolled as in the setup text. Judge with photos from the front and above, a ruler and a square, and tip the box up to see each tab's tip in its hole. Between trials unhook the tabs, roll the panel back out and let the wall spring back; replace the box after 10 trials.

## Sources

O*NET 43-5071.00, Core task 20277 (local O*NET 30 database; USDOL/ETA, CC BY 4.0): https://www.onetonline.org/link/details/43-5071.00. BLS OEWS May 2025 national estimates, 43-5071 816,870 (national_M2025_dl.xlsx): https://www.bls.gov/oes/. Box size, board and mass: Uline S-16934, 'Outside: 13 3/8 x 12 3/8 x 6 1/4" (+/- 1/8")', 'MATERIAL: B flute', 'Unit Weight: 0.99 lbs.' (read 11-16): https://www.uline.com/Product/Detail/S-16934/Indestructo-and-Literature-Mailers/12-x-12-x-6-Kraft-Tab-Locking-Literature-Mailers. Style: FEFCO 0427 in the FEFCO/ESBO international fibreboard case code; the die-line is a template built to it at that outside size (e.g. Pacdora's FEFCO 0427 generator: https://www.pacdora.com/tools/fefco-0427). Folding steps: Packlane, 'How do you assemble a Mailer Box?' (read 11-16): https://packlane.com/support/how-do-you-assemble-a-mailer-box. Board thickness: WebstaurantStore, 'Types of Corrugated Boxes', B flute 'Flute Height: 1/8"' (read 11-16): https://www.webstaurantstore.com/blog/1138/types-and-sizes-of-corrugated-boxes.html.
Friction: Park, Choi, Kim, Jung, J. Korea Soc. Packaging Sci. Tech. 25:131-137 (2019), corrugated board on board, mean static 0.38. O*NET is a trademark of USDOL/ETA. Uline and other maker names are for identification only.

## Licence

MIT
