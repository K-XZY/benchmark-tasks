# Fit a shifter fork over its spring-loaded ball: hold the ball flush with a screwdriver as the fork's bore runs over it

A step in assembling or rebuilding a lawn-tractor transaxle: work of team assemblers (O*NET 51-2092.00) and of outdoor power equipment mechanics (O*NET 49-3053.00). The maker's technician's handbook gives it in three sentences: slide the shifter fork onto the rod until it comes to the hole with the indexing ball and spring; press the ball into the hole with a flat screwdriver and move the fork completely onto the rod; then move the fork to neutral, whose groove is seen through the hole in the fork.

**Ability tested:** a hand-over of a sprung part from a tool to a workpiece. One hand holds the ball flush with the screwdriver's blade while the other runs the fork's bore up to the blade. The blade lifts away just as the bore covers the ball: too early and the ball springs up in front of the bore, too late and the bore meets the blade. Then the fork is moved on through a click to the neutral groove.

**Nearest tasks.** Two accepted tasks slide a carriage along a rail past a spring plunger until it clicks. There the plunger has a ramp or a round nose, so the sliding part pushes it down itself. Here the fork's bore cannot pass the ball until a tool presses it flush (`blocked` below), and the ball is handed from the blade to the bore. Spring-pin index tasks hold a pin withdrawn while a part is moved and relocked; here the click at neutral is the end, not the standout.

**Arms.** In the demo the right arm runs the fork and the left holds the screwdriver.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Fit the shifter fork onto the shifter rod over the indexing ball and leave it in neutral, then put the screwdriver back in its holder and let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT                                  # full score, with a video
python demokit.py task.py OUT --no-record --mode blocked       # the fork run on with the ball not pressed: it stops at the ball
python demokit.py task.py OUT --no-record --mode hardpush      # the same, then pushed at the arm's limit for 4 s
FORK_DEBUG=1 python demokit.py task.py OUT --no-record         # prints the ball, the fork and the score twice a second
python check_score.py                                          # the score lines on hand-built states, with no simulator
```

The demo reads the fork's, the screwdriver's and the ball's positions from the simulator, standing in for what a policy estimates from its cameras. It reads nothing from the scorer.

## The scene and what is simplified

An open transaxle case half of cast aluminium stands on the table, open at its -y end. The shifter rod (9.5 mm steel) runs through bosses in an inner web and the far end wall, its free end out of the open end. The indexing ball stands half out of a cross hole on top of the rod, on a spring. A stop holds it there; on the real rod nothing does, and an uncovered ball springs out, which is why the handbook has it pressed in with a screwdriver. A lay shaft beside the rod carries a shift collar and a gear; another shaft carries a second gear. The case, the shafts, the collar and the gears are fixed; the gears are drawn only.

The fork is a free rigid body of 150 g: a hub with an 11.2 mm bore, three detent grooves inside it 7 mm apart, and a sight hole through its top over the middle one; an arm; and a two-tined yoke that fits round the shift collar at neutral. A cast rib under the arm's path keeps the fitted fork from swinging round the rod, as the gears it engages do in the assembled transaxle. The screwdriver is a free rigid body standing in a holder.

The handbook's text leaves open where the ball sits. It is built here with the ball and its spring in the rod's cross hole and the grooves inside the fork's bore, the neutral one under the sight hole. The ball is a 6.35 mm sphere on a prismatic joint with a preloaded spring: 2 N with the ball half out, 4 N pressed flush. Those forces are guesses, to be measured. The ball is up-weighted from about 1 g to 3 g for a steady solver. The bore's entry edge is square, so the ball standing proud meets it about 15 degrees off its equator, inside the friction angle, and the fork stops. A chamfered edge would let the fork cam the ball in by itself. The bore is 11.2 mm on the 9.5 mm rod, looser than the real fit, for its 20-sided collider.

## Scoring

See `scoring` in `task.json`, and `judge` and `Fit` in `task.py`. The scorer holds its own 1 s hold, so `score()` and `success()` agree. The ball counts as lost if, after it has been pressed in, it comes back up with neither the blade nor the fork over it. The fork counts as on the rod when its hub's axis is within 2 mm of the rod's and within 8 degrees of it; its turn about the rod is not checked, as the rib under its arm keeps it upright. It counts as dropped if its middle falls below 12 mm above the table after it was lifted. The real trial checks the ball and the fork on a video of the whole trial. `check_score.py` runs `judge` on hand-built states.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps, one model session throughout | 0.0: the left arm stayed near its start and the right hand moved over the case's edge; neither the fork nor the screwdriver was picked up |
| `full` | The whole task | 1.0, at step 846 (42.3 s): the ball in the neutral groove 0.15 mm from its centre, never lost, the screwdriver back in its holder |
| `blocked` | The fork run on with the ball not pressed | 0.25: the fork stops with its front 2.7 mm short of the ball's middle; the ball pushed in only 0.8 mm |
| `hardpush` | As `blocked`, then the fork's hand driven 15 mm past the stop and held 4 s at the arm's limit (18.8 mm behind its command) | 0.25: the fork still stops 2.7 mm short; the ball does not go in |

## Runtime and seeds

The demo takes about 42 s of simulated time, about 4 minutes of wall time on one GPU. There is no randomness: `object_jitter=0`, so no seed is needed.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets. Occupation codes are from the O*NET 30.0 Database by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 license. The procedure follows the transaxle maker's technician's handbook. Part sizes are estimates for a lawn-tractor transaxle's shifter set (a 3/8 in rod and a 1/4 in ball), to be checked against the real parts.

## Licence

MIT
