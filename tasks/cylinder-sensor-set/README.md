# Set a pneumatic cylinder's slot sensor: drag it along the slot by a hex driver seated in its clamp screw, then clamp it

A sensor is set on a pneumatic cylinder at every cylinder fit, stroke change and sensor replacement. Each of the three occupations that do it has a Core O*NET task for it: industrial machinery mechanics (49-9041.00: "Repair or replace broken or malfunctioning components of machinery or equipment."), machinery maintenance workers (49-9043.00: "Install, replace, or change machine parts and attachments, according to production specifications.") and millwrights (49-9044.00: "Replace defective parts of machine, or adjust clearances and alignment of moving parts."). The US employed 446,900, 59,700 and 40,700 of them in 2025 (BLS employment via O*NET OnLine), 547,300 together. With the rod pushed fully home, the small sensor is dragged along the cylinder's T-slot by a hex driver seated in its clamp screw. It goes until its LED lights, on through its switching band until the LED goes out, and back to the band's middle. It is then clamped there with a quarter turn of the driver (the build counts 75 degrees or more).

**Ability tested:** moving a small part by a tool kept engaged in it, stopping where an indicator says, and locking it there without moving it. The driver's tip must stay seated in the clamp screw through the drag; if it rides out, the sensor stays behind. The quarter turn must lock the sensor where it is.

**Nearest tasks.** Holding a loose barrel proximity sensor still while its connector ring turns is a counterhold on a threaded sensor. Here a slot sensor is dragged to a light by a tool and clamped, with the other hand driving the rod. Sliding a T-nut along a profile slot to a mark and turning it shares the slide-then-lock. Here the place is found by the sensor's own switching band, the part is moved by a tool seated in it, and the setting is tested by driving the rod.

**Arms.** In the demo the right hand pushes the rod home by its clevis and the left does the setting with the driver. The instruction does not ask for two hands.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `check_score.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Push the rod fully in, set the sensor so it switches in the middle of its switching range, clamp it, then let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --size 640x360                                   # full score, with a video
python demokit.py task.py OUT --no-record --size 640x360 --mode offband,noclamp,unseated   # the cheat checks below
python check_score.py                                                          # the score lines on hand-built states, with no simulator
```

The demo reads the rod's and the sensor's positions, the driver's pose and the LED's state from the simulator, standing in for what a policy sees with its cameras. It reads the LED to find the band's edges, and nothing else from the scorer.

## The scene and what is simplified

The cylinder is fixed, with its rod on a prismatic joint whose seal friction holds it where it is, up to 8 N. The sensor slides in the slot on a prismatic joint with 1.5 N of friction. Both frictions are guesses, to be measured. The LED is lit in code from the piston magnet's distance to the sensor's middle: on within 3.5 mm, off beyond 4.5 mm, a hysteresis. These are guesses within the maker's switching band, to be measured. A lit LED stands out of the sensor's face.

The driver is a free rigid body standing in a holder. Its tip is a 2 mm hex in a 3 mm square opening round the clamp screw, not the real 1.5 mm hex in a fitted socket. A fitted hex needed about 0.05 mm of placement in the solver and did not seat. The tip drags the sensor by bearing on the opening's wall. The clamp is coded: a 75-degree turn of the driver, clockwise from above with its tip seated, locks the sensor where it is (the real screw takes about 0.6 N m). Past 200 degrees the screw counts as stripped. The sensor cannot leave its slot in this model, so there is no out-of-slot case.

## Scoring

See `scoring` in `task.json`, and `_tier` and `Setting` in `task.py`. The band's middle is the piston magnet's position with the rod fully in, and inside the band is within 3.5 mm of it. A robot that sets the sensor at the midpoint of the LED's on and off edges lands about 0.5 mm from that middle, because of the hysteresis; the magnet's position is what counts. The hands are clear when every fingertip is 6 cm or more from the sensor and 5 cm from the clevis and the driver's tip is out of the clamp screw. In a real trial the band's middle is marked on the body once beforehand, with the rod fully in: a sensor is moved toward the magnet from each side, and the midpoint of the two places its LED lights is the middle; the 1 mm check is a caliper from the sensor's middle to that mark. Once the sensor is clamped and the hands are clear for 1 s, the scorer drives the rod 20 mm out and back, checks the LED goes off and on, and pushes the sensor along the slot with 10 N. The scorer holds its own 2 s hold, so `score()` and `success()` agree. `check_score.py` runs `_tier` on hand-built states. The stripped screw and the rod-not-home case are checked there; no simulation run produced them.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps. The hands move about over the cylinder, but the driver never seats in the sensor's clamp screw: the sensor stays where it started, 52 mm from the band's middle, its LED off | 0 |
| `full` | The whole task | 1.0, at step 1152 (57.6 s): clamped 0.56 mm from the band's middle; the LED off and on with the rod; the 10 N push held |
| `offband` | The tip seated, then clamped where the sensor started, never dragged into the band | 0.0: clamped 10.6 mm from the band's middle, outside the band (it starts 12 mm from the magnet's home position; seating the tip nudged it 0.5 mm along, and the pushed rod sits 1 mm further in) |
| `noclamp` | Dragged toward the band, never clamped | 0.0: not clamped (the clamp turn left out) |
| `unseated` | Dragged toward the band, then the clamp turn made with the tip lifted out of the screw | 0.0: the turn did not register (0 degrees counted); not clamped |

## Notes on the scripted demo

The three cheat modes were run one after another in one process after `full`'s film. After the first reset the drag did not replay the full run's: in `noclamp` the sensor ended 43 mm past the band, and in `unseated` 11.6 mm short of its middle. Both score 0 from the clamp gate whatever the sensor's place; a sensor centred in the band but never clamped is checked in `check_score.py`.

The scripted demo is sensitive to its own earlier moves. The simulator is deterministic, so the same script replays the same run to a hundredth of a millimetre. But changing an earlier move, even closing a hand a few seconds sooner, changes where the driver's tip lands. It can then miss the clamp screw, and the drag fails. The demo ends with the right hand's fingers closing before the scorer's check. It shows the task can be done; it is not a robust controller. The scorer is not affected.

## Runtime and seeds

The demo takes about 58 s of simulated time, about 5 minutes of wall time on one GPU. There is no randomness: `object_jitter=0`, so no seed is needed.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets. The steps follow the sensor makers' setting instructions: actuator at its end position, the sensor moved to where its LED lights, set in the middle of its switching range, then fixed. Occupation codes and the quoted task statements are from O*NET OnLine (https://www.onetonline.org/link/details/49-9041.00, 49-9043.00 and 49-9044.00) and the O*NET 30.0 Database by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 license. O*NET is a trademark of USDOL/ETA.

## Licence

MIT
