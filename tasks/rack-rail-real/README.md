# Hang a server's tool-less sliding rail in a 4-post rack: back end into the marked U, then draw out the front and seat it

Racking servers is core data-centre work. Following a tool-less rail's installation steps, the rail goes in back end first: the two pegs on its back end piece go into the lowest and highest holes of the chosen U on the rear flange, and the end piece is pushed on until its latch catches; the front end piece is then seated the same way on the front flange.

**Ability tested:** feeding a long part's pegs into square holes at a marked height, past a latch, and extending it to reach a second flange. The rack's flanges are a typical 0.74 m apart, longer than the collapsed rail (0.63 m). In the demo both hands carry the rail in level, and its back end's pegs are fed into the marked U's square holes and pushed past the latch. The back hand then lets go, and the front hand draws the front member about 11 cm out along its slide to the front flange, matches the same U, so the rail is level, and pushes it home until it latches.

**Nearest tasks.** The rack here is at a typical depth, with EIA-310 square holes, and the rail at a real 1U rail's length and mass. A blanking panel fitted at a marked U fits one part on one pair of flanges. Here a telescoping rail is seated on two flanges 0.74 m apart, its front drawn out along its slide, both ends latched at the same U. A server chassis slid onto ledges is a different act: it moves the server, not the rail.

**Arms.** In the demo both hands carry the rail, the left seats its back end and the right draws and seats its front. The instruction does not ask for two hands. In one test a single hand at the rail's middle lost the rail, ending in a solver artefact (`onearm` below); a one-handed route that grips near the back end was not tested.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Hang the rail on the marked U, back end first, then the front, then let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT                                                  # full score, with a video
python demokit.py task.py OUT --no-record --mode wrongu,frontoff,partial,tilted,onearm   # the cheat checks below
RAIL_DEBUG=1 python demokit.py task.py OUT --no-record                         # prints the end pieces, the pegs and the score each second
python check_score.py                                                          # the score lines on hand-built states, with no simulator
```

The demo reads the rail's pose and its slide's position from the simulator, standing in for what a policy estimates from its cameras: it lines the pegs up with the holes and levels the rail as it sees them. It reads nothing from the scorer.

## The scene and what is simplified

The rack's near posts carry square-hole flanges: EIA-310, 9.5 mm square holes, three to each 44.45 mm U, built as solid bars round real holes. The bottom U's lower edge is 6 mm above the table, and the hole heights follow from it. The marked U is drawn with yellow lines on both flanges. The rack's frame and far posts are drawn only.

The rail is a free mechanism of 1.35 kg (to weigh). Its outer member carries the back end piece. The front member, with the front end piece, slides along it on a prismatic joint with 3 N of friction (to measure), from 0.63 m collapsed to 0.88 m. Each end piece has two square pegs, at the bottom and top holes of a U, tapered from 7.5 mm at the end piece to 4.5 mm at the tip over 9 mm. That is a lead-in, as real pegs have, so a push guides them into the holes. The real pegs are to be measured.

The latches are coded. An end piece latches when its two pegs are within 1 mm of one U's bottom and top holes and its face is pushed home on its flange. Over its last 3 mm it must pass a 10 N detent, approximated and to be measured. Once home it is held where it seated, as the latch's hook holds it to the flange.

## Scoring

See `scoring` in `task.json`, and `judge` and `Hang` in `task.py`. The scorer holds its own 2 s hold, so `score()` and `success()` agree. The rail's line is the line through its middle along its length; level is its length's angle from horizontal. `check_score.py` runs `judge` on hand-built states. Two cases, a front end latched a U off and a rail falling after an end has latched, were not produced in a simulation run; they are checked in `check_score.py` and in a real trial.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps. Both hands reach down to the rail where it lies on the table and touch it, but never lift it to the rack; neither end goes onto a flange | 0 |
| `full` | The whole task | 1.0, at step 421 (21 s): both ends latched at the marked U, the rail level (0.00 degrees) |
| `wrongu` | Both ends seated one U above the marked one | 0.0: both ends latched at the U above |
| `frontoff` | The back at the marked U, then the front lifted toward the U above | 0.25: the back latched; the front, out of the arm's reach a U higher, never latched. A front latched a U off scores 0.25 in `check_score.py` |
| `partial` | The back pegs pushed only part way in, then held | 0.0: 4.2 mm short of home, no latch |
| `tilted` | The rail carried out of level and pushed home without levelling | 0.0: the pegs bind 0.8 mm short of home (the rail 1.4 degrees out of level and rolled 3 degrees); no latch |
| `onearm` | The left hand alone carries the rail by its middle | 0.0: the rail pitches in the single pinch and is lost from the hand; the solver throws it off the table |

## Runtime and seeds

The demo takes about 21 s of simulated time, about 2 minutes of wall time on one GPU. There is no randomness: `object_jitter=0`, so no seed is needed.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets. Rack hole dimensions: the EIA-310 standard. The steps follow the rail maker's tool-less installation sheet.

## Licence

MIT
