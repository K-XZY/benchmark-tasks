# Load four Han-Modular modules into a HARTING hinged frame its spring holds open, then squeeze it shut until it clicks

A HARTING Han hinged frame plus stands open on a bench assembly stand, held open by its own spring. Four Han E connector modules stand in a tray beside it. The robot sets each module on the open frame's support edges at positions A to D, then squeezes the frame shut round all four until it clicks, every module still seated, and moves its hands clear.

**Ability tested:** setting small loose parts on narrow edges at a close pitch, then closing a sprung frame round all of them at once. Each 8 g module rests on the frame's two support edges, 2.5 mm lips along the walls' inner feet, with only about 1.5 mm of overlap while the frame is open; pushed about 1.5 mm sideways it drops through. The finger pads are wider than the 16 mm pitch, so a hand setting one module must pass over its neighbours' tops: a lower grip drags them along their edges. Then one squeeze must take the frame past its spring to the closed stop: the chamfered windows centre each module's lugs as the walls close, and a module left high stops the frame short of the click. The positions are given and the modules are alike, so nothing is counted or chosen; a picture shows where the modules go, not how to set and keep them.

**Why it matters:** electricians make up industrial connectors when they wire machines and control panels (O*NET 47-2111.00 Electricians, Core task: "Assemble, install, test, or maintain electrical or electronic wiring, equipment, appliances, apparatus, or fixtures, using hand tools or power tools."). HARTING's own instructions for Han-Modular modules: "install the completely assembled ... module in the Han-Modular hinged frame. Make sure that the module's notches fit into the recesses of the hinged frame" and "close it by gently pressing on the outer sides of the frame".

**Hands:** in the demo the right hand sets the modules and then parks; the left hand closes the frame. The score does not check which hand does what. The frame is fixed on its stand, so no hand needs to steady it.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Load the four modules into the hinged frame and close it until it clicks, then move both hands clear."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full     # full score, with a video
```

The demo is a scripted reference. It reads the modules' poses, which the cameras see; a policy gets only the cameras. The right hand takes each module from the tray by its 25 mm faces, above its lugs, and sets it on the open frame's support edges at A, B, C and D, then parks about 25 cm away. The left hand comes straight down over the frame's middle with its fingers outside the two walls and closes until the frame clicks, then lets go and moves away.

## Building it for a real trial

- **Frame:** a HARTING Han hinged frame plus, size 16B (09 14 016 0361), on a small bench assembly stand that holds it by its two end flanges, its foot about 15 mm above the bench, at about 30 cm in front of the arms' bases.
- **Modules:** four Han E crimp modules (09 14 006 3001), no contacts fitted, standing in a shallow tray at about 20 to 28 cm forward and 16 cm to the right.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| The frame's opening angle | 15 degrees each wall | **Not measured; approximated** (not published). Protractor on a frame resting open |
| The frame's spring | 0.3 N m/rad each wall (about 8 N at the wall's middle, 4 N at its top, to hold one shut) | **Not measured; approximated.** Push-pull gauge at a wall's middle, frame empty |
| The module's outline | 14.6 x 25 x 34 mm, 2 mm lugs | **Not verified; 14.6 mm modelled** (a distributor's listing gives 15.5 x 25 x 34 mm; the model is 0.9 mm narrower, leaving 1.4 mm between neighbours). Caliper on a module, or HARTING's CAD download |
| The support edges | 2.5 mm lips along the walls' inner feet | **Not measured** (the frame's drawing shows the walls, not the lips). Caliper |

## Safety for a real trial

Empty modules and an unwired frame: no voltage anywhere. Keep an emergency stop in reach.

## Scoring

See `scoring` in `task.json`, and `moment`, `seated` and `Trial` in `task.py`. `python3 check_score.py` runs that scorer on nine hand-built states (the full result, a hand left near, the partial results and each failure) with no simulator. In a real trial, by eye, caliper and video:

- **Seated, frame open:** a module on both support edges at its position, within 1.5 mm along and across, its foot no more than 1.5 mm over the edges, and within 5 degrees of upright.
- **Seated, frame shut:** within 1 mm along, across and in height and 3 degrees, its lugs in their windows.
- **Empty position:** no module within 8 mm along and 6 mm across of it, at any height (in, on or under the frame).
- **1.0:** the frame closed and clicked with all four modules seated shut, every fingertip of both hands at least 3 cm from the frame and the modules, held 1 s.
- **0.6:** closed and clicked with all four seated but a hand still within 3 cm, or with three seated and the fourth position empty, or all four seated with the hands clear for under 1 s.
- **0.3:** all four seated on the open frame, not closed.
- **0** if the frame was forced shut on a module more than 2 mm high (side-on video at the click), or a module fell off the table at any moment. Anything else scores 0.

In the simulator the click is code: once both walls are within 0.5 degrees of shut, they lock, as the real frame's latch does. Hands clear is measured from all 24 fingertip points of both hands to both walls along their length and to each module's faces.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | All four modules set on the open frame's support edges, 0.4 to 1.6 mm along the frame and up to 0.9 mm across from their seats; the frame squeezed shut and clicked, its windows centring all four within 0.6 mm; let go, both hands clear; held 1 s | 1.0, at step 993 (49.7 s) |
| Dream-1 | The right hand takes one module from the tray to the frame; the run ends with it still in the hand, the other three in the tray | 0 |

No seed is needed: nothing in the scene is random. Runtime per trial, on one GPU: the demo's 993 steps take about 4 minutes while recording three cameras; a run to the 2,400-step limit takes about 12 minutes.

## Assets

All meshes are made by the task's own code from plain shapes at the sizes above. No third-party assets, no textures.

The task statement quoted in `task.json` is from the O*NET database (O*NET OnLine, task statements for 47-2111.00, https://www.onetonline.org/link/summary/47-2111.00) by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence. O*NET is a trademark of USDOL/ETA. The assembly steps are quoted from HARTING's instructions for the Han Pneumatic double module (manualslib.de, manual 904709, p. 2); the frame's sizes are from HARTING's drawing of the Han hinged frame plus 16B and the module's mass from HARTING's data sheet for 09 14 006 3001 (https://docs.rs-online.com/e782/A700000015178126.pdf). HARTING, Han and Han-Modular are trademarks of HARTING, named for identification only.

## Licence

MIT
