# Grind a mounted chip package's cross-section flat on a turning abrasive disc, down to the line scribed round the mount

Failure-analysis labs inspect a packaged chip's solder joints, bumps and die by cross-section: the package is sawn, set in an epoxy mount, then ground flat by hand on a grinder-polisher before polishing and microscopy. Buehler's guide to manual preparation asks for "a firm hand pressure", the specimen "continually moved back and forth between the center and the edge of the wheel", starting on P180 silicon-carbide paper "until plane". Here one hand grinds a 32 mm mount holding a strip sawn from a BGA package on P180 paper turning at 150 rpm, down to a line scribed 0.3 mm above its face: the last 0.3 mm of the P180 step, short of the ball row, leaving stock for the finer grits. The paper drags the mount sideways while the hand pushes it down and sweeps it across a disc that moves faster at its edge; any rocking or tilt grinds a facet instead of one plane, and going past the line spoils the section.

**Instruction.** Grind the mounted section on the turning paper down to the line scribed round the mount. Hold the mount face down on the paper and press it steadily, moving it back and forth between the centre and the edge of the disc and keeping its face flat, until the face reaches the line all the way round. Then lift it off the paper, set it back on its rest as it started, and move your hand clear.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Every mesh is generated in code by `task.py`; no third-party assets.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 6000 \
  --instruction "Grind the mounted section on the turning paper down to the line scribed round the mount. Hold the mount face down on the paper and press it steadily, moving it back and forth between the centre and the edge of the disc and keeping its face flat, until the face reaches the line all the way round. Then lift it off the paper, set it back on its rest as it started, and move your hand clear."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                   # full score, with a video
python demokit.py task.py OUT --no-record --mode none,hover,tilted,oneside,light,over,leave,keep   # the failure runs below
GRIND_INDEX_DEG=15 python demokit.py task.py OUT --no-record --mode spot                       # an overgrind between the 0, 90, 180 and 270 degree stops
python -B check_score.py                                                                 # the scorer and the rule on hand-built states, no simulator
python -B check_sizes.py                                                                 # the scene's sizes against the published ones
```

The mount's upper end is clamped in a hand holder: a printed cup over the mount's top, a flange 60 mm across on the cup, and on the flange a knob 26 mm long and 20 mm across two flats. The right hand comes down over the knob with its fingertips just above the flange and closes across the flats, the jaws' axis along the paper's drag; each finger presses at the 40 N cap the task sets (`FINGER_N_MAX`, written to the gripper joints' force limit at reset; the harness's own limit is 50 N). The push goes down through the fingertips onto the flange, and the drag presses one flat into a pad, so neither depends on the pads' friction: the mount does not creep up the fingers, which stay about 40 mm above the face, with the flange between them and the wet paper.

The hand lifts the mount, carries it over the bowl's rim and lowers it to 8 mm above the paper, 3.5 cm from the platen's centre. There it turns against the tilt it sees on the mount until the axis is vertical, and touches down square. It presses to about 11 N, read from the platen's spring, and sweeps the mount between 3.5 and 8 cm from the centre and back every 4 s, aiming by where the mount is seen, since the soft arm sags back under the push and the drag carries the mount along the jaws' axis. It keeps turning against the tilt it sees (up to 12 degrees about the drag's axis), so the face stays within a few tenths of a degree of flat, with brief excursions to about 1 degree at the ends of the sweep; the ring and centre end between 0.28 and 0.32 mm, 0.041 mm apart (a 320 x 176 trace, ring 0.282 to 0.323 mm, centre 0.298 mm). When the red line reaches the face (about 200 s of grinding), the hand eases the push off and unwinds its turn while still on the paper, lifts the mount straight up and carries it over the rim to its rest. There it lowers the mount at 8 mm/s until it reads seated (its face within 0.5 mm of the rest's top, or pushed 0.5 mm up the grip), stops the hand where it is so nothing presses the mount into the rest, holds it 0.5 to 1 s until it reads still and flat, opens, rises straight up 6 cm and moves clear. The demo reads the mount's pose, the platen's spring and the line shown on the mount; it never moves an object except through the gripper. It takes 4,444 steps (222 s at 20 Hz) of the 6,000-step limit, and scores 1.0.

## Sizes, and what is simplified

`python -B check_sizes.py` prints the scene's key sizes against the published ones (no simulator):

```
quantity                                scene  published  source
mount diameter (mm)                     32.00      31.75  1.25 in epoxy mount, Allied High Tech (alliedhightech.com)
mount height (mm)                       35.00      35.00  trimmed to 35 mm by us
platen diameter (mm)                   203.00     203.20  8 in platen, Allied MetPrep guide (mse.ucr.edu/media/1273/download)
platen speed (rpm)                     150.00     150.00  Buehler SumMet PCB method 120 to 150 rpm
package strip thickness (mm)             2.86       2.86  FCBGA substrate class, 35 x 35 x 2.86 mm (estimate)
scribed line above the face (mm)         0.30       0.30  ours: the last 0.3 mm of the P180 step
scored ring radius (mm)                 14.00      14.00  2 mm in from the 16 mm rim
holder flange diameter (mm)             60.00      60.00  ours, printed PETG
knob across the flats (mm)              20.00      20.00  ours; the jaws open 94 mm
rest, top (mm)                          50.00      50.00  ours, a printed block
finger force cap (N)                    40.00      40.00  Robocurve's figure for the YAM
SIZES: PASS
```

Departures from the real thing, stated once. The mount is 32.00 mm across, 0.25 mm over the 31.75 mm (1.25 in) cup, a rounding in the mesh. The package strip is a plain 20 x 2.86 x 12 mm box set edge-on, flush with the face: its solder balls, die and layers are not modelled, because the act and its score are about the epoxy face, not the strip's inside (nothing reads the strip). The grinder body is a 40 x 30 x 10 cm box and the bowl a 32 cm ring, sizes ours (a bench grinder-polisher is about that size); the platen is an aluminium disc with a P180 paper layer at the real 8 in size. The abrasive's removal is a rule (below), not a simulated material. The mount's collider keeps its full height; removal is kept in a height map.

## How the grinding works

A simulator cannot remove material, so removal is a rule, and the rule needs the real work:

- **The push is measured, not assumed.** The platen rides on a stiff vertical spring (20 N/mm, under 1 mm at the push used); its deflection is the force the mount puts on the paper.
- **The push is spread over the face by the mount's tilt.** Each point of the face (a 2 mm grid, 200 points) is pressed into an elastic paper (0.1 mm under a 15 N flat push, assumed) by how far it lies below the face's contact level; a mount held tilted presses one edge and grinds a facet there. The spread follows the tilt the simulator reports: a push off the mount's centre that does not tilt it leaves the pressure even, where a real paper would load the near edge more.
- **Preston's law with a threshold** (Preston 1927, J. Soc. Glass Technol. 11:214: rate = k x pressure x speed; p0 = 4.5 kPa and k = 1.9e-10 per Pa are set here, not measured, to the rate a real hand is assumed to take, about 0.1 mm per minute). Each point loses height at a rate proportional to (its pressure - 4.5 kPa) x its speed over the paper, the speed from the platen's turn at that point's radius less the mount's own motion. A flat face under 15 N loses 0.3 mm in about 125 s, under 11 N in about 195 s, under 8 N it does not reach it in 300 s (0.27 mm). Under 5 N it loses about 0.08 mm in the whole 300 s; resting on the paper, nothing.
- The mount's collider keeps its full length; the removed depth lives in the height map. The red line drawn round the mount moves down toward the face as the mean stock left shrinks, in 0.05 mm steps, as a real face rises to its line.

Tilt sets the spread (removal rule alone, 13 N): a flat hold reaches the line with the ring and centre readings within 0.04 mm of each other; a 0.3 degree hold puts 0.18 mm between the lowest and highest ring stop (the highest past 0.37 mm), a 1 degree hold 0.50 mm, and a 3 degree hold 0.94 mm.

## Measuring a real trial

One rule, in the real trial and in `score()`: the removal at each of the 24 stops round the 2 mm inset ring (72 in the simulation) and at the centre is its height before the trial less its height after, and more than 0.37 mm at any of them fails the trial.

Removal is read round the whole 2 mm inset ring and at the centre, the same points `score()` reads in the simulation (`SCORED` in `task.py`): 24 stops every 15 degrees on the circle 2 mm in from the rim, counted from an index line scribed down the mount's side, and the centre. The simulation reads 72 stops, every 5 degrees (every third is a real stop), so no gap between real stops can hide a dip in the scored numbers; the real trial closes the same gap with one slow sweep (step 5).

Tools and cost (catalogue prices, November 2026; check before buying):

| Item | Part | Price |
|---|---|---|
| Dial indicator, 0.01 mm, 10 mm range | Mitutoyo 2046S | about US$100 |
| Granite surface plate, 12 x 18 x 3 in, grade B | Grizzly G9651 | about US$130 |
| Magnetic-base indicator stand with a fine-adjust arm | any, e.g. a Noga- or Shars-type stand | about US$40 |
| Turntable: 3 in lazy-Susan bearing, a printed PETG disc with a 32.2 mm centring cup and 24 notches every 15 degrees, a spring plunger, a pointer | bearing and plunger from a hardware supplier; printing ours | about US$20 |
| 35 mm gauge block (or any steel block of that height) | from a workshop | about US$20 |
| Total | | about US$310 |

1. Before the trial, with the holder off, stand the mount on its flat back in the turntable's cup, on the surface plate. The index line on the mount's side meets a pointer fixed on the turntable's base: that is stop 0. The stand holds the indicator over the plate with its tip on the ring, 14 mm from the mount's axis (a mark on the stand's arm).
2. Set the indicator to zero on the gauge block standing on the same turntable. Write down its reading on the face at the centre.
3. Turn the mount by hand to each of the 24 notches in turn and write down the height at each. The turntable's own height error repeats at each notch and cancels in the difference.
4. Turn it once more, slowly, all the way round, and note the lowest and highest readings and where they fall.
5. After the trial, take the holder off and repeat steps 1 to 4.
6. Removal at each stop is its reading before less its reading after (the face falls, so after is lower). Overgrinding anywhere is the largest removal at any stop, or the largest fall found by the sweep, exceeding 0.37 mm. The scored numbers are the 24 stops and the centre; the sweep is the check that nothing sharper lies between two stops.

Resolution, worked: the indicator reads to 0.01 mm, so each reading is good to 0.005 mm and a removal (two readings) to 0.01 mm. The limits it must resolve are the 0.05 mm threshold (5 divisions), the 0.10 mm spread (10), the 0.23 to 0.37 mm band (14 wide) and the 0.37 mm cap (37 divisions of stock): a stop at 0.38 mm is told from the cap by one division, so a stop within 0.01 mm of a limit is re-read. The notch puts the tip within about 0.5 mm of the same ring point and the face is flat to a few hundredths of a millimetre over that distance (a 0.3 degree tilt changes the height by 0.003 mm in 0.5 mm), so a stop repeats to about 0.02 mm. A reading takes about 20 s; the 25 readings, the sweep and the repeat take about 20 minutes a trial.

The rungs read those 25 numbers: 0.25 for at least 0.05 mm at one or more points; 0.5 for at least 0.05 mm at all of them, the largest and smallest removal within 0.10 mm; 0.75 for all between 0.23 and 0.37 mm; 0 for any over 0.37 mm, on the ring or at the centre. A removal over the cap anywhere on the ring is a failure that stays: the score stays 0 whatever the later state. Grinding inside the ring shows at the centre point, and a dip between the stops at 0, 90, 180 and 270 degrees shows at the stops between them.

**Real removal time.** The simulation and a real trial use one rate: about 0.1 mm per minute for a hand pressing about 11 N (assumed; range 0.05 to 0.2 mm per minute), from the order of Buehler's PCB method (P180 at 120 to 150 rpm, 5 lb, "until plane"), which gives no rate. A trial removes only the last 0.3 mm, as a technician does on the final pass to a line: about 3 min (1.5 to 6 min over the range), so the 300 s run limit (6,000 steps at 20 Hz) fits a real trial at the middle of the range. The earlier stock is taken beforehand and is not scored.

A finger on the paper is read from `fingertips()`, whose points sit about 5 mm below the fingers' real underside: a point 4 mm or more below the paper's top, over the platen, counts as a touch.

## Failure table

Scripted runs of the demo with one thing changed (`demokit.py --no-record --mode ...`), and the removal rule on hand-built states.

| Route | What the hand does | Score | What stops it |
|---|---|---|---|
| none | the arms wait | 0.0 | nothing ground |
| hover | holds the mount 4 mm above the turning paper for 30 s | 0.0 | no push on the platen's spring, so nothing is removed |
| light | a light touch: a 5 N push for 270 s, then sets it back | 0.25 | at 5 N the face grinds slowly and unevenly: after the run some stop is still short of 0.05 mm or the stops are over 0.10 mm apart (run scored 0.25 in 5,817 steps) |
| oneside | holds the mount 3 degrees over, across the jaws, for 60 s, grinding one side only, then sets it back | 0.0 | one stretch of the ring (at 245 degrees from the index line) lost 0.41 mm, past 0.37, while the stops on the far side were untouched (0.00 mm); the run scored 0.0 |
| tilted | grinds to the line holding the mount 3 degrees over, across the jaws | 0.0 | a facet: the high-side stops pass 0.37 mm before the mean reaches the line, while the far side is still unground (run scored 0.0 in 3,864 steps) |
| over | keeps grinding for 270 s, past the line, then sets it back | 0.0 | ground past the line: the highest stop passed 0.37 mm, and the score stays 0 (run scored 0.0) |
| spot | held flat, ground on past the line until one stop of the ring is 0.002 mm over the cap, then set back on the rest. The index line is scribed 15 degrees round (`GRIND_INDEX_DEG=15`) so the highest stop lies between the stops at 0, 90, 180 and 270 degrees | 0.0 | the ring's highest stop reads 0.373 mm (at 125 degrees); the stops at 0, 90, 180 and 270 degrees read 0.348, 0.368, 0.368 and 0.348 mm and the centre 0.356, with the mount on its rest: those four points and the centre alone would pass it at 1.0. The whole ring scores it 0.0 |
| leave | grinds to the line, then lets go of the mount on the turning paper | 0.0 | the platen carries the released mount off (the thrown latch): the 0.75 reached at the line is lost, as the scoring text says |
| keep | grinds to the line, lifts the mount off and holds it in the air, never sets it back | 0.75 | not on the rest: the 1.0 end state is not reached |
| rule, 0.3 degree held, 13 N | to the line | 0.0 | ring stops from 0.21 to 0.39 mm: 0.18 mm between lowest and highest, the highest over 0.37 |
| rule, 5 N for 300 s | a light steady touch | 0.5 | 0.08 mm removed at every stop, short of the line |
| rule, resting (0.5 N) | the mount's own weight and a light hand | 0.0 | under the 4.5 kPa threshold nothing is removed |
| rule, centre 0.12 mm past the edge points | a dished face | 0.25 | the ring and centre are not within 0.10 mm of each other |


## Sources

- Buehler, SumMet guide, manual preparation: "a firm hand pressure"; the specimen is "continually moved back and forth between the center and the edge of the wheel"; "After each step, the specimen is rotated 45 to 90°"; PCB method P180, P400, P1200 SiC, 120 to 150 rpm, P180 "Until Plane". https://pdfcoffee.com/bhuler-metal-preperation-pdf-free.html
- Allied High Tech, MetPrep guide: "For 1.25" (32 mm) Sample Mount: 4 – 6 LbF (per sample)", platen 200 to 300 rpm. https://mse.ucr.edu/media/1273/download
- Allied High Tech, EpoxyMount: 17.5 g resin and 5.25 g hardener for a 1.25 in mount. https://www.alliedhightech.com
- Pololu 37D metal gearmotors (#4744: 12 V, 70:1, 150 rpm no-load). https://www.pololu.com/category/116/37d-metal-gearmotors
- Adafruit Perma-Proto half-sized breadboard PCB (#1609, plated through-holes). https://www.adafruit.com/product/1609
- Buehler CarbiMet PSA P180 8 in (36-08-0180), via https://shop.jhtechnologies.com/products/b36080180
- UKAM SMART CUT LG grinder-polisher, 200 mm platen. https://ukam.com/product/smart-cut-lg-manual-metallurgical-grinder-polisher-with-single-wheel/

Maker and product names are for identification only.


## Loopholes closed

| Route | Fix |
|---|---|
| Another body (the other arm, the wrist) presses the platen while the mount hovers or barely touches | cutting needs the mount touching the paper; a spring push above 2 N for 0.5 s with the mount off the paper, or any push over 25 N, trips the touch rule (score 0). Limit: a second body pressing while the mount is also in contact is not told apart from the mount's own push |
| Mount held 1 to 3 mm above the paper with a push from elsewhere | same: no contact, no removal |
| Mount dropped or flicked onto the rest, or onto the machine deck | a mount faster than 0.3 m/s with no fingertip within 3 cm, or higher than the rest and off it with no hand near, counts as thrown (0) |
| Shaking the mount against the platen to cut faster | the relative speed is capped at the platen's own speed at that point |
| Gripping the mount lower so the fingers reach the paper | a fingertip 4 mm or more below the paper's top over the platen is a touch (0); with the holder the fingers stay about 40 mm above the face |
| Moving the mount after the end state | `on_rest` and hand-clear are read at every step, so moving it off scores 0.75 again |
| Holding the mount still at one radius without sweeping | not closed: the score is by effect, the instruction asks for the sweep, and the elastic paper here levels the small speed difference across the face (stated limit) |

## Runs

All on the task as it is in this folder.

| Run | GPU | Result |
|---|---|---|
| README demo, 640 x 360, full | three RTX 4070 Ti SUPER | 1.0 each, 4,444 steps (222 s), 0 black frames |
| README demo, 320 x 176, full | an RTX 4070 Ti SUPER | 1.0, 4,444 steps |
| README demo, 640 x 360, full, filmed | an RTX 5090, the machine to itself | 1.0, 4,444 steps (222 s); slowest 30 steps 5.67 s (0.189 s a step); 0 black frames on each camera |
| Dream-1, 6,000 steps, 20 rows a request (20 x 0.189 + 0.5 = 4.3 s) | the RTX 5090, alone | 0.0, valid, no session loss; film 6,001 frames, 3 cameras, 0 black |
| Demo with the mount's start shifted by up to 1 cm (`--jitter 0.01`), seeds 1 to 8, 320 x 176 | RTX 4070 Ti SUPER | 1.0 each: 4,452, 4,446, 4,437, 4,447, 4,445, 4,452, 4,441 and 4,446 steps |

## Dream-1

Dream-1, 6,000 steps, seed 1: score 0.0. Scripted demo: 1.0.
