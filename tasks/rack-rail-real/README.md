# Hang a server's tool-less sliding rail in a 4-post rack: back end into the marked U, then draw out the front and seat it

Racking servers is core data-centre work. Dell's installation sheet for its tool-less ReadyRails II sliding rails gives the steps: "Align each end piece in the bottom and top holes of the desired U spaces. Engage the back end of the rail until it fully seats on the vertical rack flange and the latch clicks into place. Repeat these steps to position and seat the front end piece on the vertical rack flange." The rail here is that kit's B6 rail, the 1U rail of the PowerEdge R640.

**Ability tested:** feeding a long part's pegs into square holes at a marked height, past a latch, and extending it to reach a second flange. The rack's flanges are a typical 0.74 m apart, longer than the collapsed rail (0.63 m). In the demo both hands carry the rail in level, and its back end's pegs are fed into the marked U's square holes and pushed past the latch. The back hand then lets go, and the front hand draws the front member about 11 cm out along its slide to the front flange, matches the same U, so the rail is level, and pushes it home until it latches.

**Nearest tasks.** The rack here is at a typical depth, with EIA-310 square holes, and the rail at Dell's B6 length range (630 to 883 mm). A blanking panel fitted at a marked U fits one part on one pair of flanges. Here a telescoping rail is seated on two flanges 0.74 m apart, its front drawn out along its slide, both ends latched at the same U. A server chassis slid onto ledges is a different act: it moves the server, not the rail.

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

The rail is a Dell B6 ReadyRails II sliding rail, the tool-less 1U rail of the PowerEdge R640. In the simulation it is a free mechanism. Its outer member carries the back end piece. The front member, with the front end piece, slides along it on a prismatic joint (the rail's depth adjustment), from 0.63 m collapsed to 0.883 m, Dell's square-hole range between flanges. Each end piece has two square pegs, at the bottom and top holes of a U, and a spring latch whose release button is at the end piece's midpoint, as Dell's sheet shows. The rail's inner member, the server slide, is not modelled: it stays shut while the rail is hung.

The latches are coded. An end piece latches when its two pegs are within 1 mm of one U's bottom and top holes and its face is pushed home on its flange. Over its last 3 mm it must pass a detent. Once home it is held where it seated, as the latch's hook holds it to the flange.

### Figures and where they come from

| Figure | Value | Source |
|---|---|---|
| Rail | Dell B6 ReadyRails II, sliding, tool-less in square-hole 4-post racks | Dell, PowerEdge R640 Technical Guide, p. 30 and table 17 |
| Length between flanges | 630 to 883 mm (square holes); collapsed at the start, 0.63 m | Dell, PowerEdge R640 Technical Guide, table 18 |
| End pieces | two pegs each, in the bottom and top holes of one U; a latch that clicks as the end piece seats; release button at the end piece's midpoint | Dell, "Rack Installation - Sliding Rails" sheet |
| Rack holes | 9.5 mm square, three to each 44.45 mm U | EIA-310 |
| Flange spacing | 0.74 m | a typical rack depth, within Dell's range |
| Rail mass | 1.35 kg | simulation setting: Dell lists no mass; a steel rail of this length, carried level with two hands |
| Rail section | 18 x 40 mm channel | simulation setting: Dell gives no section; it fits within one U |
| Pegs | square, tapered from 7.5 to 4.5 mm over 9 mm | simulation setting: Dell gives no peg size; a lead-in into EIA-310's 9.5 mm holes |
| Slide drag | 3 N | simulation setting: light enough to draw by hand, enough that the front member stays where it is left |
| Latch detent | 10 N over the last 3 mm | simulation setting: so that seating needs a deliberate push. The real trial judges the latch by eye, not by force |

## Building it for a real trial

Use one Dell B6 ReadyRails II sliding rail and a 4-post rack with EIA-310 square-hole flanges set 0.74 m apart, with the third U from the bottom marked on both near flanges. Lay the rail collapsed on the table at the start position. Judge each end piece by eye: both pegs through the marked U's bottom and top holes, the end piece flat on its flange and its latch closed. Judge level with a spirit level on the rail. Then check on the camera that both hands are off and 3 cm or more clear of the rail for 2 s, and that the rail is still hung at the end. No force is measured.

## Scoring

See `scoring` in `task.json`, and `judge` and `Hang` in `task.py`. The scorer holds its own 2 s hold, so `score()` and `success()` agree. The rail's line is the line through its middle along its length; level is its length's angle from horizontal. `check_score.py` runs `judge` on hand-built states. Two cases, a front end latched a U off and a rail falling after an end has latched, were not produced in a simulation run; they are checked in `check_score.py` and in a real trial.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps. Both hands reach down to the rail where it lies on the table and close beside it, but never lift it to the rack; neither end goes onto a flange | 0 |
| `full` | The whole task | 1.0, at step 425 (21 s): both ends latched at the marked U, the rail level |
| `wrongu` | Both ends seated one U above the marked one | 0.0: both ends latched at the U above |
| `frontoff` | The back at the marked U, then the front lifted toward the U above | 0.25: the back latched; the front, out of the arm's reach a U higher, never latched. A front latched a U off scores 0.25 in `check_score.py` |
| `partial` | The back pegs pushed only part way in, then held | 0.0: 4.2 mm short of home, no latch |
| `tilted` | The rail carried out of level and pushed home without levelling | 0.0: the pegs bind 0.8 mm short of home (the rail 1.4 degrees out of level and rolled 3 degrees); no latch |
| `onearm` | The left hand alone carries the rail by its middle | 0.0: the rail pitches in the single pinch and is lost from the hand; the solver throws it off the table |

## Runtime and seeds

The demo takes about 21 s of simulated time, about 2 minutes of wall time on one GPU. There is no randomness: `object_jitter=0`, so no seed is needed.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Dell EMC, *PowerEdge R640 Technical Guide*, "Rack rails", tables 17 (static and sliding rails) and 18 (rails adjustability): https://coloradosos.gov/pubs/elections/VotingSystems/DVS-DemocracySuite517/documentation/DellPoweredgeR640TechGuide.pdf
- Dell, *Rack Installation - Sliding Rails* (B6 ReadyRails II sliding rail assemblies), "Installing and Removing Tool-less Rails (Square Hole or Round Hole Racks)": https://pcserverandparts.com/content/Spec%20Sheets/Servers/Dell/Dell%20PowerEdge%20R620/Rack%20Installation%20%E2%80%94%20Sliding%20Rails.pdf
- Rack hole dimensions: the EIA-310 standard.

Dell, PowerEdge and ReadyRails are trademarks of Dell Inc., named here for identification only.

## Licence

MIT
