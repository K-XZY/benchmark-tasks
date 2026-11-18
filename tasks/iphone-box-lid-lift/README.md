# Open a new iPhone's box: lift its close-fitting lid off slowly, so the air in the fit doesn't pull the base up

An iPhone 16 box stands on the bench with its seal already pulled: a lid over a base, a millimetre apart, so that the lid comes off only slowly. The left gripper pinches the lid by its two long walls and raises it straight up at about 3.5 mm/s while the lid overlaps the base, then lifts it clear, carries it to the side and sets it on the bench. A lid raised faster drags the base up with it.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,yank,hold,none
```

The harness's instruction (Dream-1 run): `--instruction "Open the iPhone box: lift the lid off and put it down on the bench beside the box. The bottom of the box, with the phone in it, stays on the bench."`

## Scoring, and the routes that score less

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | the left hand lifts the lid at 3.5 mm/s over its 27 mm of overlap, then sets it down beside the box | 1.0 (571 steps, 28.55 s) |
| yank | the lid lifted at 100 mm/s | 0.0 (the base rises 21 mm) |
| hold | lifted slowly, then held in the air to the end | 0.5 (the lid is lifted clear and never set down) |
| none | both arms wait, the lid left on | 0.0 |
| tilt | the held lid tipped 20 degrees, past the air's 15 degree cutoff, then lifted at 100 mm/s | 0.0 (the base rises 6 mm) |
| right | the right hand does the whole lift instead (its finger wedges on the 2 mm wall as it closes) | 0.5 |

Lift: 0.5 once the lid's rim has risen above the base's top. Set-down: 0.5 more when the lid lies on the bench, either way up, at least 2 mm clear of the base. The score is 0 if the base ever rose more than 3 mm, moved more than 2 cm or tilted more than 5 degrees. The run ends with success once 1.0 has held for 1 s with both hands 3 cm clear and everything still.

## Real trial

Stand a non-working iPhone 16 dummy (same size, 170 g) in its box on the bench. Film from the side against a ruler. Lift: the lid's rim clear of the base's top. Set-down: the lid on the bench, not touching the box. Base down: it never rises 3 mm, moves 2 cm or tilts 5 degrees. Before each session, hold a box by its lid 3 cm up and check that the base slides out in about 4 s; replace the box if it takes under 2 s.

## Modelling

- Lid and base are two free rigid bodies built from boxes (`task.py`): lid outside 164 x 86 x 30 mm, walls and top 2 mm, 55 g, open at the bottom; base outside 158 x 80 x 27 mm, walls and floor 2 mm, 260 g with the phone and cable; 1 mm between the walls on every side and 27 mm of wall overlap. Apple publishes no box size: these are estimates from the phone's size and maker photos, with 1 mm for the gap (real about 0.3 mm) and 2 mm for the walls (real about 1.5 mm).
- The phone is Apple's 147.6 x 71.6 x 7.80 mm, fixed in the base's tray (no separate body); the 260 g includes it at 170 g and a 30 g cable (estimate).
- The air in the fit is code: while the lid and base overlap by more than 2 mm, `Trial.step` pushes the base up with 400 N s/m times the lid's vertical speed relative to the base, and the lid down with the same force limited to what a 55 g lid can take in one control step. Above about 6.4 mm/s (the base's 2.5 N over 400 N s/m) the push exceeds the base's weight and it rises. The 400 N s/m is an estimate, set so that a base left hanging in a held lid slides out in about 4 s. The force is limited to what brings the base level with the lid within one control step, which keeps it stable once the base is off the bench. The rigid lid cannot squeeze the base as flexing paperboard would.
- Frictions: lid 0.35 and base 0.5 (estimates, averaged with the other surface). The demo reads object poses and the fingertips' positions.

## Sources

- Apple, iPhone 16 box information, https://support.apple.com/en-afri/boxinfo/120111: 71.6 x 147.6 x 7.80 mm, 170 g. Checked on 18 Nov 2026.
- BGR, "There's a reason why iPhone boxes are notoriously hard to open", https://bgr.com/2166176/why-iphone-boxes-are-so-hard-to-open/: the lid's close fit and slow lift.
- 9to5Mac, iPhone 13 box, https://9to5mac.com/2021/09/17/iphone-13-box/: the pull-tab seal.
- A user's measure of an iPhone 12 Pro box, about 164 x 89 x 29 mm, https://discussions.apple.com/thread/252040114, as a cross-check on the box size.
- All meshes are built in `task.py` from boxes; no third-party meshes are used.

Apple and iPhone are trademarks of Apple Inc., named for identification only.

## Licence

MIT
