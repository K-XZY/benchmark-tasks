# Squeegee a wet upright glass pane dry: keep the rubber edge pressed at a working angle through each overlapping stroke

**Where it comes from.** Janitors and cleaners have the task "Clean windows, glass partitions, or mirrors, using soapy water or other cleaners, sponges, or squeegees" (O*NET 37-2011.00, 2,432,600 US employees in O*NET's 2025 figures; USDOL/ETA, CC BY 4.0; O*NET is a trademark of USDOL/ETA). Unger, a maker of window squeegees, teaches the stroke: "place the blade at a 45° angle to the glass, making sure to keep steady pressure on it throughout your movements. If you apply too much pressure, you'll find it difficult to glide the squeegee over the surface, while too little pressure will leave water behind." (https://usa.ungerglobal.com/blog/squeegee-window-cleaning-for-a-streak-free-shine/). Unger is named as a source and for identification only; this task is not affiliated with it. Unger's page teaches a continuous S stroke; this task uses the other common method, overlapping top-to-bottom strokes with the blade wiped between them.

**What this task tests.** A blade's edge kept bearing on an upright surface through whole strokes. The pane stands upright, so the arm must press the rubber into the glass (along +x) while it draws it down (along -z): force and motion in different directions at once, held through each stroke. The water clears only where the rubber touches the glass with at least 2 N, at 30 to 60 degrees to it. A blade held flat, or only touching, clears nothing where it stays so (the `flat` and `light` checks, below). The strokes must overlap. The water a stroke clears stays on the blade, so the blade must be wiped on the towel (held in the other hand, as cleaners do, or left in its holder) before it touches the glass again: an unwiped blade puts the water back where it lands and streaks every cell it passes (the `nowipe` check). A blade lifted off mid-pane leaves its bead on the glass.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Squeegee the wet glass pane dry, wiping the blade on the towel between strokes, then set the squeegee back on its stand, let go of it and take your hands away from the pane."
```

A trial runs up to 2,400 steps (120 s of simulated time at 20 Hz). One fixed layout; no seed changes the scene.

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                         # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode flat,light,nowipe,keep,frame,frametip  # the failure checks
python check_score.py                                                             # the scoring table on hand-built records, no simulator
```

The demo is a scripted reference: it reads the squeegee's, the towel's and the water's state from the simulator, and a policy gets only the cameras, in which the wet glass shows as blue cells. The left hand takes the towel from its holder by its top and holds it up beside the pane, out of the strokes' way. The right hand takes the squeegee by its handle from the -y side, lifts it off its stand, draws it back clear of the frame and turns it to the pane: the channel along y, the rubber toward the glass at 55 degrees, the handle back and up. For each of two strokes, 6 cm apart so they overlap by 19 cm, it brings the rubber's edge to the glass just under the top rail, reads both ends of the edge against the glass and turns the blade about the vertical to square it, presses it in until the rubber gives 3 mm, and draws it down in six steps, commanded to 3 cm above the glass's bottom edge; the arm sags there, so the edge comes down through the bottom row to 1.3 to 2 cm above the bottom rail, in the band left out, and lifts off there, short of the rail, pressing a little deeper while the rubber gives less. The near stroke, on the right arm's own side, presses less deep. After each stroke the left hand brings the towel up and the right wipes the rubber down the towel's face. Then the right hand sets the squeegee back on its stand and lets go, and both hands draw back from the pane.

**Result** (scripted, GPU physics on an NVIDIA GeForce RTX 3090 Ti, 20 Hz, filmed at 640 x 360): 1.0 in 930 steps (46.5 s of simulated time). The first stroke cleared 83 percent of the cells and the second, overlapping it, cleared the rest (144 of 144); the blade was wiped on the towel after each stroke; the squeegee ended within 1 mm of its place on the stand, let go. The same run on an RTX 4070 Ti SUPER gives the same numbers (its result is kept beside the film). The film's first frame is the start state.

**Model run** (Dream-1, once, 12 rows per request, seed 1): 0.0 in 2,400 steps (120 s of simulated time), one session of 198 requests, none refused. The policy reached the squeegee on its stand with the right hand and nudged it 17 mm along the stand, but never lifted it or brought the blade to the glass: no cell cleared. The film is `model_run.mp4`.

| Failure check | What it does | Score |
|---|---|---|
| `flat` | the same strokes with the blade held nearly flat, 10 degrees to the glass (pitched 80 degrees) | 0.053 in 1042 steps: 5.6 percent clear, only at the top of the strokes, where the pressed blade tipped in the hand into the working band; held flat it cleared nothing |
| `light` | the same strokes with the rubber only touching the glass, not pressed | 0.0 in 994 steps: under 2 N the edge cleared no cell |
| `nowipe` | the demo, but the blade is never wiped on the towel | 0.158 in 782 steps: the first stroke cleared 83 percent; the second landed with the first's water on the blade, put it back and streaked every cell it passed, leaving 17 percent clear |
| `keep` | the demo, but the squeegee is left in the hand on its stand | 0.9 in 974 steps: 95 percent of the cells clear (the second episode of a session clears a little less than the first) and the squeegee on its stand, but still held, so the hands are not clear |
| `frame` | two strokes with one end of the blade 1 cm onto a side rail of the frame (5 mm proud of the glass), pressed against the rail, the rest of the edge over the glass | 0.0 in 1018 steps: bearing on the rail, the edge cleared nothing |
| `frametip` | as `frame`, the blade turned 1.5 degrees so its inner end comes toward the glass | 0.427 in 1042 steps: in the second stroke the blade rode the rail with its inner end on the glass at 1.7 to 4 N and cleared nothing; in the first it came off the rail altogether (its outer end 1 cm off the glass), its inner end pressed on the glass at about 3 N and 54 degrees, and that tilted stroke cleared the cells along the part of its edge the pressed rubber reaches, 45 percent |
| `check_score.py` | the scored grid (a point 1 cm from the bottom rail, the top rail or a side rail falls in no cell; 3 cm in, in the outer row or column; every drawn cell's middle scores as that cell) and the scoring table on hand-built records (95 percent clear and tidied is 1.0, 8 of 144 cells wet is 0.895, the squeegee kept in the hand or off its stand is 0.9, the towel touched to the glass or the squeegee dropped is 0) | every case as expected |

## Scoring

The glass is divided into 12 x 12 cells of 2.5 cm, leaving out a band along each rail: 2 cm at the sides and 2.5 cm at the top and bottom, the edge a cleaner details with the cloth afterwards (a real stroke leaves a bead along the bottom rail). Every scored cell lies inside what a stroke sweeps. Every cell starts wet. A cell clears only while the rubber's edge passes over it touching the glass (within 1.5 mm of the glass's face; where some of the edge touches the glass with 2 N or more, the rest of the edge bears up to 4 mm further off, as a pressed rubber flexes; and while any of the edge bears on the frame, within 1.5 mm of a rail's face, the frame holds the blade off the glass and nothing clears that step), pressed with at least 2 N in all along the edge, with the blade at 30 to 60 degrees to the glass. The water a stroke clears stays on the blade. Unless the blade is wiped on the towel (its edge touched to the towel's face: three or more points of it within 3 mm), its next touch on the glass after a lift (0.3 s or more off the glass; a shorter loss of contact is a bounce within the stroke) puts that water back on the cells under it, and its edge then streaks every cell it passes, leaving it wet, until it is wiped. A blade lifted off the glass for 0.3 s or more mid-pane, not off the bottom row, leaves the cells under its edge wet.

| Score | When |
|---|---|
| 1.0 | at least 95 percent of the cells clear, the squeegee back on its stand (its middle within 3 cm of its place), every fingertip of both hands 3 cm or more from points along the squeegee's channel and handle and 5 cm or more in front of (or behind) the plane of the glass's face, held 1 s |
| 0.9 x (share clear) / 0.95 | otherwise, with the share of cells clear counted up to 95 percent; so the most a run that does not tidy up can score is 0.9 |
| 0 | at any moment: the towel touched to the glass (it would dry the glass itself), or the squeegee dropped below the table |

On a real trial: within 10 s of the robot letting go, photograph the glass square-on with a matt black board behind the pane and a lamp at a grazing angle (about 20 degrees) from one side. Map the scorer's grid onto the photo by the frame's four inside corners. A cell is not clear if film, beads or a streak line show in it. The top camera and both wrist cameras film the trial; a side camera level with the pane reads the blade's angle to the glass, which no station camera shows.

## Building it for a real trial

- **Pane:** a pane of 4 mm toughened (tempered) glass, cut to 340 x 350 mm before toughening, in a 25 mm aluminium frame standing 5 mm proud of the glass on the arms' side, held upright by two feet clamped to the table, its glass face at x 35 cm, its middle at y 3 cm and the glass's bottom edge 4 cm above the table. It is sized to the YAM's reach: a real window sash's glass is larger.
- **Squeegee:** an Unger ErgoTec 25 cm squeegee (part ES250: a 25 cm stainless steel S channel with ErgoTec soft rubber on the ErgoTec handle; 0.4 lb, about 0.18 kg, in the maker's table, https://usa.ungerglobal.com/product/ergotec-squeegee-complete/, which lists it for "Cleaning small windows and glass partitions in commercial buildings"). Unger and ErgoTec are named for identification only.
- **Stand:** a dark steel stand: a 10 x 17 cm base plate, a 3 cm square post and two rests, 7 x 5.4 cm under the channel and the handle's first 4 cm and 6 x 4.2 cm under the handle's end on a side arm; the channel's middle at (30, -36) cm, 20 cm up, the rubber toward -y and the handle toward +y.
- **Towel:** a folded microfibre towel, about 12 x 10 x 2 cm, standing in a holder (a 6 x 15 x 3 cm block with 2 cm lips) at (24, 22) cm. A wipe counts wherever the towel is, in the other hand or in its holder, so one arm can do the whole task; the demo holds it up in the other hand, as cleaners do.
- **Wetting:** before each trial, wet the whole glass with window-cleaning solution at its maker's dose from a trigger sprayer: a fixed number of pulls in a fixed grid from 30 cm (calibrate the millilitres per pull by weighing 10 pulls). Start the trial within 30 s, before the film dries. Dry the blade and wring the towel between trials.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Least press that clears | 2 N along the edge | **Not measured.** Draw the squeegee down wet glass on a kitchen scale under the pane's foot at rising presses; note the least that leaves no film |
| Working angle | 30 to 60 degrees between the blade and the glass | **Not measured.** Unger teaches 45 degrees; film strokes at set angles from the side and note where film is left |
| Rubber's give | 1000 N/m, up to 8 mm | **Not measured.** Press the rubber's edge on a scale and measure how far the channel moves with calipers |
| Squeegee's mass | 0.18 kg | The maker's table: 0.4 lb (rounded to 0.1 lb); weigh it |

## What the simulator leaves out

There is no liquid. The water is a grid of wet cells, cleared by the rubber's edge under the conditions above (a code rule). The water a blade carries is a count, put back where an unwiped blade next touches the glass; such a blade then leaves every cell it passes wet, standing in for the streak lines a dirty rubber draws. Water running down the glass is left out, so the top-to-bottom order is not tested, and so is water left where a stroke reverses without lifting. On a real pane the water decides what clears; the 2 N press and the 30 to 60 degree band are the simulator's model of it, to be measured as in the table. The rubber is a rigid strip on a sprung slider in its channel, so its compression is the press; a pressed rubber bears along its edge where the glass is within 4 mm more, standing in for the real rubber's flex. The towel is a rigid pad. The rubber's stiffness is an approximation too.

## Assets

Every shape in the scene (the pane and its frame, the squeegee, the stand, the towel and its holder, the wet cells) is built in code from boxes in `task.py`; no third-party meshes, textures or fonts are used. The YAM arms and the station come from `yam-isaac-lab-arena`.

## Licence

MIT
