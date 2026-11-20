# Square a crooked cabinet door: turn its top cup hinge's side screw with a screwdriver until the gap is even

Two frameless cabinets stand side by side. The left door hangs crooked: the gap to the right door is 1.7 mm at the top and 2.6 mm at the bottom. One hand opens the door by its pull and holds it open. The other takes a PH2 screwdriver, reaches into the open cabinet, seats the bit in the top hinge's side-adjust screw and turns it about two thirds of a turn with the wrist. It takes the screwdriver out, the door is pushed shut, and both hands let go: the gap is now even, about 2.5 mm top and bottom.

**Why it matters (Impact on jobs).** Carpenters (O*NET 47-2031.01) and general maintenance and repair workers (49-9071.00) adjust cup-hinged cabinet doors after every install and whenever a door drops out of line. Both occupations list this as Core work: install "hardware, using carpenters' hand or power tools", and "Adjust functional parts of devices or control instruments, using hand tools". On a Blum CLIP top hinge the front screw on the hinge arm moves the door sideways at that hinge, so turning the top hinge's screw swings the door's top edge and squares the gap.

**Ability tested:** a closed loop with a hand tool. The screw is reached only with the door open, but the gap shows only with the door shut. The bit must stay seated in a 9 mm screw head inside the cabinet while the wrist turns it, and the right screw must be turned the right way by about the right amount.

**Nearest tasks.** Other screwdriver tasks drive a screw home; here the screw is an adjuster, turned a set amount to move a part, and judged by a gap the robot cannot see while it turns. Shimming a house door's hinge jamb is a different object and motion.

**Arms.** One arm opens and closes the door, the other turns the screwdriver. One arm could do both in turn: the hinge's friction holds the door open above 25 degrees.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "The left cabinet door hangs crooked. Use the screwdriver on its hinges to make the gap between the two doors even from top to bottom, 2 to 3 mm wide, then shut the door and let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                  # full score, with a video
python demokit.py task.py OUT --no-record --mode MODE      # one of the routes below
python check_score.py                                      # the scorer on hand-built trials, with no simulator
```

What the demo does, each move closed on observed poses:
1. The left hand takes the left door's bar pull and swings the door open 45 degrees about its hinge line; the hinge's friction holds it there, and the hand keeps hold.
2. The right hand takes the screwdriver from its post by the handle, the fingers level and the handle along the hand's approach axis, so that the wrist roll joint turns the driver about its own axis.
3. It carries the driver into the open front of the cabinet, brings the bit's tip in line with the top hinge's side screw and squares the driver to its tool axis from the driver's observed pose.
4. It winds the wrist roll joint back by 112 degrees, then brings the tip into the screw's recess.
5. It turns the wrist roll joint through up to 224 degrees in one stroke. Each step, the hand's position is solved so that the tip, which sits a few millimetres off the wrist's axis in the grip, stays on the screw head. If the head has turned less than the turns needed, the bit comes out, the wrist winds back and the bit goes in again.
6. The bit comes out of the cabinet; the left hand swings the door shut by its pull and lets go.
7. The right hand sets the driver back on its post and lets go; both hands move clear.

The demo reads the door's pose, the screw head's angle, the driver's pose and the arms' joints; it reads nothing from the scorer. Nothing in the scene is random: `object_jitter=0`.

## The scene and what is simplified

Frame: x forward from the line through the arm bases, y left from the midline, z up from the table top, mm.

| Part | In the scene | Published | Source |
|---|---|---|---|
| Cabinet boxes | 300 wide x 300 deep x 450 high, 18 mm melamine board, on a 30 mm plinth, fronts at x 400 | frameless (European) carcase, 18 mm board | any 18 mm melamine carcase |
| Doors | 298 x 448 x 19 mm MDF, 1.9 kg, full overlay | | estimate (MDF about 750 kg/m3) |
| Hinges | two per door, 80 mm from its top and bottom, on the outer side panels; 110-degree swing | Blum CLIP top 110-degree full overlay, 71T3550 | blum.com |
| Mounting plates | 0 mm, 8.5 mm thick | Blum 173H7100 | build.com/blum-173h7100 |
| Side adjustment | front screw on the hinge arm, +-2 mm | +-2 mm | Blum CLIP top assembly instructions |
| Reveal | right door square, 2.5 mm from the left door | | the fitter's usual 2 to 3 mm |
| Screwdriver | PH2, 150 mm: handle 75 mm, 31 mm across the corners; shaft 6 mm x 75 mm; 0.15 kg | | any PH2 x 75 mm screwdriver |

- **The door** is a rigid body on one pivot for the hinge's 110-degree swing. Friction holds it where it is left above 25 degrees; below 25 degrees the hinge's self-close spring draws it shut. The door sits on two hidden joints, a sideways slide and a tilt in its own plane, which follow the two hinges' side offsets.
- **The side screws** are real bodies with a cross recess, each turning on its own joint. The thread and the bit's grip in the recess are a declared rule (`Thread` in `task.py`): while the bit's tip is in the recess (within 2.5 mm of its centre, the tip at most 2 mm short of the head's face and at most 0.5 mm into it, the driver within 30 degrees of the screw's axis), the driver's roll about its own axis turns the head, and each turn moves that hinge 1 mm sideways. The pitch is assumed: Blum gives the +-2 mm range, not the thread.
- **Forced past the stop:** a screw turned on past the end of its +-2 mm travel turns no further; half a turn of forcing strips it.
- **Not modelled:** depth and height adjustment (the depth screws and the plate cams are fixed), and the screw's turning torque (the head turns freely while the bit is seated).
- The 30-degree seating tolerance lets the driver come in through the door opening at the angle a fitter uses inside a 264 mm wide cabinet: the bit meets the screw about 25 degrees off its axis.

## Limits

- The demo grips the driver at 90 N per finger. This is a simulated grip setting, not the real need: at 35 to 40 N the simulated driver tilts in the pinch (point contacts, no wide pad) and the run scores 0. Our estimate of the real need (no cited source): a PH2 bit turning a small hinge screw takes about 0.3 to 0.5 N m and about 20 N of axial push to stay seated. Through a 31 mm handle (radius 15.5 mm) with pad friction 0.8, the torque needs 0.3/(2 x 0.8 x 0.0155) = 12 N per finger (20 N at 0.5 N m), the push needs 20/(2 x 0.8) = 12.5 N per finger, and both together about 17 to 24 N per finger. That is inside a YAM gripper's roughly 40 N. The 90 N hold is not a real-robot claim.
- The bit seats with the driver up to 25 degrees off the screw's axis (the rule allows 30); the demo reaches it through the door opening at that tilt. A tighter rule did not score in our tests.
- The screw's thread pitch (1 mm per turn) is assumed, and the head's turning torque and the contact force at the bit are not modelled.

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. The state is read after every physics step.

| What happened | Score |
|---|---|
| At the end: the left door within 2 degrees of shut; both gaps (20 mm below the top and 20 mm above the bottom) 2.0 to 3.0 mm and within 0.5 mm of each other; released (every point on both whole grippers, fingers, pads, palm and housing, at least 30 mm from the cabinets, doors and driver; the driver's tip at least 20 mm from every screw; door and driver still), held 1 s | 1.0 |
| At the end: shut with those gaps, but not released | 0.5 |
| At the end: the top hinge's side screw turned at least a quarter turn the right way (opening the gap at the top) | 0.25 |
| Any time: a side screw forced half a turn past the end of its travel | 0 |

The tiers read the state at the end; only the stripped screw latches.

The gaps follow from the two hinges' side offsets: at 1 mm a turn, a turn of the top screw opens the top gap by 1.21 mm and closes the bottom one by 0.21 mm. From 1.7 and 2.6 mm, anything from 0.28 to 0.99 turns of the top screw puts both gaps in the window; 0.63 turns makes them equal. Turning the bottom screw instead tapers the gap the other way.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` | The whole task, filmed (lab PC, RTX 3090 Ti) | 1.0 at step 1113 (55.65 s): the wrist's one stroke turns the head 0.62 turns; gaps 2.47 mm at the bottom and 2.42 mm at the top |
| Dream-1 | The Dream model, one valid session, seed 1, 16 rows per request, 2,400 steps (lab PC) | 0.0: the doors end shut and the screwdriver lies on the table; the screw is never turned. Film: attached to the PR |
| `none` | The arms wait | 0.0 |
| `door_only` | Opens the door and shuts it again, never touches a screw | 0.0 (gaps stay 2.60 and 1.67 mm) |
| `hover` | The bit seated in the top screw, never turned | 0.0 |
| `bottom` | Turns the bottom hinge's screw instead | 0.0: low in the cabinet the driver tilts in the hand and the head turns 0.015 turns; a full turn of the bottom screw is checked in `check_score.py` (0) |
| `reverse` | Turns the top screw the wrong way | 0.0: wound the other way first, the bit never seats and the head does not turn; the wrong-way turn itself is checked in `check_score.py` (0) |
| `open_end` | Turns the top screw right but leaves the door open | 0.25 (head turned 0.62 turns, door left at 45 degrees) |
| `hold_driver` | Turns it right and shuts the door, but keeps hold of the driver | 0.25: the right hand keeps the driver in the door's path, so the self-closing door stops at 33 degrees; the door shut with the driver still held is checked in `check_score.py` (0.5) |

`check_score.py` runs the real `Trial` on 13 hand-built trials, each scoring as the text says: the honest run (1.0); 0.35 turns (1.0); the bottom screw (0); the wrong way (0); 1.3 turns, the gaps out of the window (0.25); the bit seated but never turned (0); the door opened and shut (0); turned, door left open (0.25); opened again after the end state (0.25); a hand left on the pull (0.5); fingertips clear but the palm on the door (0.5); the driver left in the screw (0.5); a screw stripped (0).

## Runtime and seeds

The full demo takes 55.65 s of simulated time (1,113 steps of the 2,400-step limit), about 5.5 minutes of wall time with the film on one RTX 3090 Ti at 640 x 360, start-up apart. There is no randomness at `object_jitter=0`.

## A real trial

Build two 300 x 300 x 450 mm carcases from 18 mm melamine board and screw them together side by side. Hang a 19 mm door on each with two Blum 71T3550 hinges on 173H7100 plates (35 mm cups bored 80 mm from the door's top and bottom). Set the right door square at a 2.5 mm reveal. Set the left door by its top hinge's side screw until the gap reads 1.7 mm 20 mm below the top and 2.6 mm 20 mm above the bottom. Lay a PH2 x 75 mm screwdriver on a post in front of the cabinets. Read the gaps with a feeler-gauge set in 0.1 mm steps (about 10 USD) at two pencil marks, taking the thickest blade that slides in; evenness is the difference of the two readings; mark the screw head with paint to read the turn; read the release from the video. Protection for a real trial: pad covers on the gripper fingers, tape on the cabinet edges, and a soft mat under the screwdriver's post.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Blum CLIP top hinges and adjustment: side +-2 mm by the front screw, height +-2 mm by the plate cam, depth +3/-2 mm by the rear cam (blum.com, CLIP top BLUMOTION assembly; Blum CLIP top frameless hinge instruction sheet). Blum is a trademark of its owner, named for identification only.
- Plate 173H7100: build.com/blum-173h7100.
- O*NET 47-2031.01 (Construction Carpenters), 49-9071.00 (Maintenance and Repair Workers, General).

## Licence

MIT
