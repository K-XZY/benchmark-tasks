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
```

The mount's upper end is clamped in a hand holder: a printed cup over the mount's top, a flange 60 mm across on the cup, and on the flange a knob 26 mm long and 20 mm across two flats. The right hand comes down over the knob with its fingertips just above the flange and closes across the flats, the jaws' axis along the paper's drag; each finger presses at the 40 N cap the task sets (`FINGER_N_MAX`, written to the gripper joints' force limit at reset; the harness's own limit is 50 N). The push goes down through the fingertips onto the flange, and the drag presses one flat into a pad, so neither depends on the pads' friction: the mount does not creep up the fingers, which stay about 40 mm above the face, with the flange between them and the wet paper.

The hand lifts the mount, carries it over the bowl's rim and lowers it to 8 mm above the paper, 3.5 cm from the platen's centre. There it turns against the tilt it sees on the mount until the axis is vertical, and touches down square. It presses to about 11 N, read from the platen's spring, and sweeps the mount between 3.5 and 8 cm from the centre and back every 4 s, aiming by where the mount is seen, since the soft arm sags back under the push and the drag carries the mount along the jaws' axis. It keeps turning against the tilt it sees (up to 12 degrees about the drag's axis), so the face stays within a few tenths of a degree of flat, with brief excursions to about 1 degree at the ends of the sweep; the five points end within 0.04 mm of each other. When the red line reaches the face (about 200 s of grinding), the hand eases the push off and unwinds its turn while still on the paper, lifts the mount straight up, carries it over the rim to its rest, sets it down, opens and moves clear. The demo reads the mount's pose, the platen's spring and the line shown on the mount; it never moves an object except through the gripper. It takes 4,379 steps (219 s at 20 Hz) of the 6,000-step limit, and scores 1.0.

## How the grinding works

A simulator cannot remove material, so removal is a rule, and the rule needs the real work:

- **The push is measured, not assumed.** The platen rides on a stiff vertical spring (20 N/mm, under 1 mm at the push used); its deflection is the force the mount puts on the paper.
- **The push is spread over the face by the mount's tilt.** Each point of the face (a 2 mm grid, 200 points) is pressed into an elastic paper (0.1 mm under a 15 N flat push, assumed) by how far it lies below the face's contact level; a mount held tilted presses one edge and grinds a facet there. The spread follows the tilt the simulator reports: a push off the mount's centre that does not tilt it leaves the pressure even, where a real paper would load the near edge more.
- **Preston's law with a threshold** (Preston 1927, J. Soc. Glass Technol. 11:214: rate = k x pressure x speed; p0 = 4.5 kPa and k = 1.9e-10 per Pa are set here, not measured, to the rate a real hand is assumed to take, about 0.1 mm per minute). Each point loses height at a rate proportional to (its pressure - 4.5 kPa) x its speed over the paper, the speed from the platen's turn at that point's radius less the mount's own motion. A flat face under 15 N loses 0.3 mm in about 125 s, under 11 N in about 195 s, under 8 N it does not reach it in 300 s (0.27 mm). Under 5 N it loses about 0.08 mm in the whole 300 s; resting on the paper, nothing.
- The mount's collider keeps its full length; the removed depth lives in the height map. The red line drawn round the mount moves down toward the face as the mean stock left shrinks, in 0.05 mm steps, as a real face rises to its line.

Tilt sets the spread (removal rule alone, 13 N): a flat hold reaches the line with the five scored points within 0.03 mm of each other; a 0.3 degree hold puts 0.18 mm between them (one edge point past 0.37 mm), a 1 degree hold 0.50 mm, and a 3 degree hold 0.94 mm.

## Measuring a real trial

Removal is read at five marked points on the face, the same five that `score()` reads in the simulation (`FIVE` in `task.py`): four on the circle 2 mm in from the rim, at 0, 90, 180 and 270 degrees from an index line scribed down the mount's side, and the centre.

1. Before the trial, with the holder off, stand the mount on its flat back on a surface plate.
2. Slip a printed locating cap over its face: a 32.2 mm bore, a plate with five 4 mm holes at the five points, and a key that sits on the index line.
3. Read each point through its hole with a 0.01 mm dial indicator on a stand, after setting it to zero on the same 35 mm gauge block each time. Write down the five readings; take the cap off and refit the holder.
4. After the trial, take the holder off and repeat steps 1 to 3.
5. Removal at each point is its reading before less its reading after.

The rungs read those five numbers: 0.25 for at least 0.05 mm at one or more points; 0.5 for at least 0.05 mm at all five, within 0.10 mm of each other; 0.75 for all five between 0.23 and 0.37 mm; 0 for any point over 0.37 mm. Grinding inside the 2 mm ring shows at the centre point: a centre removed 0.10 mm or more beyond the edge points breaks the 0.10 mm band, and over 0.37 mm it scores 0.

**Real removal time.** The simulation and a real trial use one rate: about 0.1 mm per minute for a hand pressing about 11 N (assumed; range 0.05 to 0.2 mm per minute), from the order of Buehler's PCB method (P180 at 120 to 150 rpm, 5 lb, "until plane"), which gives no rate. A trial removes only the last 0.3 mm, as a technician does on the final pass to a line: about 3 min (1.5 to 6 min over the range), so the 300 s run limit (6,000 steps at 20 Hz) fits a real trial at the middle of the range. The earlier stock is taken beforehand and is not scored.

A finger on the paper is read from `fingertips()`, whose points sit about 5 mm below the fingers' real underside: a point 4 mm or more below the paper's top, over the platen, counts as a touch.

## Failure table

Scripted runs of the demo with one thing changed (`demokit.py --no-record --mode ...`), and the removal rule on hand-built states.

| Route | What the hand does | Score | What stops it |
|---|---|---|---|
| none | the arms wait | 0.0 | nothing ground |
| hover | holds the mount 4 mm above the turning paper for 30 s | 0.0 | no push on the platen's spring, so nothing is removed |
| light | a light touch: a 5 N push for 270 s, then sets it back | 0.25 | at 5 N the face grinds unevenly: after 270 s one edge point has lost 0.03 mm and another 0.21 mm, so not all five reach 0.05 mm, nor lie within 0.10 mm |
| oneside | holds the mount 3 degrees over, across the jaws, for 60 s, grinding one side only, then sets it back | 0.0 | one edge point lost 0.41 mm, past 0.37, while three of the five were untouched (0.25 from about 20 s until it passed 0.37 mm at about 55 s) |
| tilted | grinds to the line holding the mount 3 degrees over, across the jaws | 0.0 | a facet: one edge point passed 0.37 mm at about 55 s; when the mean reached the line it had lost 0.87 mm while two edge points were still unground |
| over | keeps grinding for 270 s, past the line, then sets it back | 0.0 | ground past the line: an edge point passed 0.37 mm at about 250 s, and at the end all five had lost 0.39 to 0.42 mm, the centre 0.40 mm |
| leave | grinds to the line, then lets go of the mount on the turning paper | 0.0 | the platen carries the released mount off (the thrown latch): the 0.75 reached at the line is lost, as the scoring text says |
| keep | grinds to the line, lifts the mount off and holds it in the air, never sets it back | 0.75 | not on the rest: the 1.0 end state is not reached |
| rule, 0.3 degree held, 13 N | to the line | 0.0 | 0.18 mm between the five points, one edge point 0.39 mm |
| rule, 5 N for 300 s | a light steady touch | 0.5 | 0.08 mm removed at all five points, short of the line |
| rule, resting (0.5 N) | the mount's own weight and a light hand | 0.0 | under the 4.5 kPa threshold nothing is removed |
| rule, centre 0.12 mm past the edge points | a dished face | 0.25 | the five points are not within 0.10 mm of each other |


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

## Dream-1

Dream-1, 6,000 steps, seed 1: score 0.0. Scripted demo: 1.0.
