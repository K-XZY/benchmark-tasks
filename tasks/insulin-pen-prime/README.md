# Do an insulin pen's 2-unit safety shot with the needle up, then dial a 12-unit dose by its window

A disposable insulin pen with a pen needle on lies in its tray. Its instructions for use start every injection the same way: pull off the outer needle shield, turn the dose knob to 2 units, hold the pen with the needle pointing up, push the knob in until it stops and 0 shows in the dose window, and hold it in while counting to 5. Then the dose is dialled by the window and corrected by turning the knob either way. The robot does this and dials 12 units: one hand picks the pen up, stands it needle up and holds it; the other pulls the shield, dials, pushes the knob home from below and holds it, and dials the dose; the pen goes back in its tray.

**What this task adds.** It is a sequence on one device, each step through the pen's own mechanism: a shield drawn off along the needle, a knob that clicks once a unit and screws out as it is dialled, read through a window that shows even numbers and a line for each odd one, and a push along the pen's axis from below that counts the units back to 0 and must be held there. The dial is turned in short strokes with a regrip between them, so a stroke that comes up a unit short or over must be seen in the window and corrected. The safety shot only counts with the needle up, and the dose only counts after it.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Get the insulin pen ready for a 12-unit dose: take off the needle shield and put it in its tray, do the 2-unit safety shot with the needle up, then set 12 units and leave the pen in its tray."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                                  # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode noshot,short_hold,over --hold 2.5          # failure checks, below
python check_score.py                                                                       # the scoring table on hand-built records, no simulator
```

The demo is a scripted reference: it reads the pen's pose and the window's count from the simulator, and a policy gets only the cameras, in which the window's number shows. The left hand picks the pen at its middle from above, lifts it and turns it a quarter turn so the needle points up and the window faces the right arm, and holds it 15.5 cm up. The right hand, fingers level, pinches the shield and draws it off upward, then drops it in the shield tray. The left hand lifts the pen to 22.5 cm. The right hand turns the knob in strokes of up to 50 degrees about the pen's axis, letting go and coming back between strokes, until the window reads 2. With its fingers closed it comes up under the knob's end face and pushes the knob home, then holds it there 5.5 s; the left hand holds the pen's place against the push. Then the right hand dials 12 by the window, the left hand turns the pen back down and lays it in its tray, and both hands withdraw.

**Result** (scripted, GPU physics on an RTX 5090, 20 Hz): 1.0. The shield in its tray; 2 dialled in one stroke; the knob home with the needle within 3 degrees of up, held 5.5 s; 12 dialled in 6 strokes; the pen in its tray. 1210 steps (60.5 s of simulated time), about 5 minutes of wall time. No seed changes the scene.

**Model run** (Dream-1, once, 12 rows per request, seed 1): no score. After 103 accepted requests in one session, the provider's API refused request 104 at 63.4 s of simulated time: "DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Manipulating exposed needle points up creates a sharp hazard." The trial ended on the refusal, so the scorer gave no score; this is not a 0 the policy earned. The film (`model_run.mp4`) runs to the refusal. The run was not repeated and the instruction was not changed.

| Failure check | What it does | Score |
|---|---|---|
| `noshot` | dials 2, then goes straight on to 12 without the safety shot | 0.2: the shield in its tray; 12 set and the pen in its tray, but the dose counts only after a safety shot |
| `short_hold` | pushes the knob home but holds it 2 s, not 5 | 0.2: the count went to 0 needle up, but held 2 s the shot does not count, so neither does the dose |
| `over` | does the safety shot, then leaves the dose at 13 | 0.6: the shield and the safety shot; the window reads 13, not 12 |

## Building it for a real trial

- **Pen:** a disposable insulin pen of the KwikPen kind (0 to 60 units, 1 unit a click), 17 mm across the body, dose knob 18 mm across, with a 4 mm pen needle and its outer shield on (166 mm long in all), about 20 g. Use a trainer pen only (a spent pen will not dial past the units left, and an expired one still holds insulin), or a pen whose insulin is handled as live.
- **Trays:** a white plastic pen tray 50 x 210 mm with 3 mm rails, centred at (24, 12) cm, the pen along y with the needle toward -y and the window facing down; a blue plastic shield tray 70 x 70 mm with 16 mm walls at (30, -11) cm. Both fixed to the table.

| Value | In the simulator | How to measure it on the real pen |
|---|---|---|
| Knob turn and rise per unit | 18 degrees, 0.33 mm | Dial 20 units: count the turns and measure the knob's rise with calipers |
| Click hold | 0.04 N m | A torque screwdriver bit on the knob |
| Push to deliver | 2 N, held all the way in (a real pen needs about 8 to 12 N: a real trial uses the pen's own force, so the robot must push that hard) | A kitchen scale under the knob's end while pushing it home |
| Shield pull-off | released once a pinch draws it 3 mm | A spring scale on the shield |
| Digits in the window | 3.6 mm tall | Calipers |

## Safety for a real trial

Use a trainer pen only, or a pen whose insulin is handled as live. Fit a fresh pen needle and outer shield for each trial; never put a shield back on a used needle by hand, and drop used needles in a sharps container. During the push the bare needle points up and away from people. Keep people clear of the arms' reach and of the needle while the robot runs, and keep an emergency stop in reach.

## Scoring

See `scoring` in `task.json`, and `Pen`, `_tier`, `score` and `success` in `task.py`. The scorer reads the knob's count, the pen's pose, the shield's place and the fingertips. The safety shot counts once the shield is off and the count, dialled to exactly 2, is pushed to 0 with a fingertip under the knob's end face and the needle within 30 degrees of up, and the knob is held home 5 s. A push is a fingertip under the knob's end face moving the knob in; the count then follows the knob down, as the pen delivers. The 2 s hold applies only to the full 1.0: a run meeting every part scores 0.6 until it has held 2 s, and lower totals count at once.

A real trial is read from video and a photo: the right wrist and top cameras show the window at 2 before the push, the needle within 30 degrees of up, and the knob held home 5 s; a photo at the end shows the window at 12, the pen in its tray and the shield in its tray. Between trials, turn the knob back to 0 without pushing (nothing delivered) and fit a fresh needle with its outer shield.

## Assets

All geometry is built in code from cylinders and boxes; the window's digits are seven-segment bars on the dose knob's sleeve. No third-party assets.
