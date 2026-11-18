# Put both AirPods Pro back in their case, each stem-first into its own well, and push the lid shut so its magnet holds it

An Apple AirPods Pro 2 MagSafe Charging Case stands open on the bench, lid at 115 degrees on its far side, and both earbuds lie on their sides beside it. One gripper holds the case low at one end. The other picks each bud, turns it stem down, steers the stem down its own well's bore until the bud is within the well magnet's 7 mm reach, and lets go; the magnet pulls it home. Then it pushes the lid over from behind its far edge with a fingertip until the lid's magnet holds it shut.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,left_only,swap,one_bud
```

The harness's instruction (Dream-1 run): `--instruction "Put both earbuds back in their charging case and close the lid."`

## Scoring, and the routes that score less

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | right hand holds the case; left seats the right bud, then the left bud, and pushes the lid shut | 1.0 (1,163 steps, 58.15 s) |
| left_only | the left hand does everything, the case unheld | 0.0 (both buds seat, but the push on the lid tips the case over, 90 degrees) |
| swap | each bud steered into the other bud's well | 0.0 (each side rib meets the other well's wall; both buds stay 6.3 mm proud and the lid rests 69 degrees open on them) |
| one_bud | the right bud seated, the left bud left on the bench, the lid pushed shut | 0.25 |

Each bud seated in its own well, within 0.5 mm of home, 1 mm of its axis in each direction, 5 degrees of upright and 10 degrees of its turn: 0.25. Lid shut within 1 degree over both seated buds: 0.5 more. The case must end upright within 5 degrees and within 3 cm of where it stood; a case tipped past 20 degrees at any moment, or a bud off the bench, scores 0 (the case may be lifted, as long as it never tips past 20 degrees). Judged once every fingertip of both hands is 3 cm clear of the case and all has been still for 1 s; the scorer holds the end state itself.

## Real trial

Stand a real AirPods Pro 2 case on the bench with its lid fully open, the buds on their marks. Seated: turn the open case upside down and both buds stay in. Shut: turn the closed case upside down and the lid stays shut. Tipped: the case must stand where it stood, within 3 cm.

## Modelling

- Case at Apple's 45.2 x 60.6 x 21.7 mm and 50.8 g; buds at Apple's 30.9 mm length, 21.8 mm width and 5.3 g; bud head 17.3 mm deep: Apple's 24.0 mm includes the silicone ear tip, which squashes in the real case and is not modelled.
- The case is one free mechanism: its body, and the lid on a hinge along the far top edge. Each well is a bore shaped to the stem (5.7 x 7.6 mm, 23.3 mm long, 0.75 to 0.8 mm of play a side) with a chamfered mouth (2.5 mm a side, 4 mm deep) and two key slots over its top 10 mm: one for the rib on the stem's outer side (2.2 mm proud of the stem), one for the bump on its front. The buds are mirror images, so each fits only its own well, stem first, front toward the robot.
- Each well's magnet is code: a 0.4 N pull on a bud toward its seat, only while the bud's seat is within 7 mm of home, with the equal and opposite pull on the case. The lid's magnet is a drive in its own hinge, 1 N at the lid's front edge (21.7 mm from the hinge) over its last 5 degrees, so the case's body and lid pull on each other.
- A seated head stands 7.6 mm proud of the rim and the shut lid clears it by 1.4 mm, so a bud left proud holds the lid open.
- The demo reads object poses and the lid's hinge angle.

## Sources

- Apple, AirPods Pro (2nd generation) tech specs, https://support.apple.com/en-gb/111851: each earbud 30.9 x 21.8 x 24.0 mm, 5.3 g; MagSafe Charging Case 45.2 x 60.6 x 21.7 mm, 50.8 g. Checked on 18 Nov 2026; the page is for the 2022 model with the Lightning MagSafe case, whose sizes and masses the task uses.
- Apple, Accessory Design Guidelines dimensional drawings, "MagSafe Charging Case (USB-C) for AirPods Pro (2nd generation)" and "AirPods Pro (2nd generation)", https://developer.apple.com/accessories/: the lid's full opening (115 degrees) and the stem's section (5.7 mm) and length (23.3 mm). Used for sizes only; the drawings are not included.
- Well, bore, key, magnet, hinge and friction figures are estimates from maker photos, as Apple publishes none.
- All meshes are built in `task.py` from boxes and prisms; no third-party meshes are used.

Apple, AirPods, AirPods Pro and MagSafe are trademarks of Apple Inc., named for identification only.

## Licence

MIT
