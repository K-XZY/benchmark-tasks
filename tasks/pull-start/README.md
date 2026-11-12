# Hold down a small engine and pull its recoil starter fast over the full stroke

A starter trainer, built like a small engine with a recoil starter, stands loose on the table. One arm presses it down by its top pad, and the other pulls the yellow T-handle out its full 30 cm in under half a second, as anyone does to start a lawn mower.

**Ability tested:** a fast pull over a set length, while the other arm takes the reaction.

**Both arms are forced by weight. With nobody holding, a top-speed pull drags the unit 46 cm and the handle never comes out its full length, scoring 0.**

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 300 \
  --instruction "Hold the engine down by its top pad with one arm, and with the other pull the yellow starter handle out fast over its full 30 cm, in under half a second. The engine must not shift more than 10 cm."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                        # full score, with a video
python demokit.py task.py OUT --no-record --mode one_arm          # nobody holding the unit: 0
python check_scorer.py                                            # the scoring table, with no simulator
```

The demo reads the unit's position, as a camera could see it, and the grips go where it is. The arm is soft under the spring, so it is commanded 36 cm to bring the handle 30 cm.

## Building it for a real trial

The trainer reproduces what the simulator models, and nothing more: a light unit that slides on the table, a pad to press it down by, and a handle whose pull resistance rises evenly over the stroke. There is no engine.

- **Base:** a plate 20 x 40 x 1.2 cm, of 10 mm XPS foam faced on both sides with 1 mm hardboard, painted grey.
- **Block and housing:** hollow PLA shells, painted dark grey. The block is 10 x 14 cm and 13 cm high above the table, with fins along its sides; the housing behind it is 9 x 13 cm and 10 cm high.
- **Top pad:** black neoprene rubber, 6 x 6 x 1 cm, glued on the block's top, its top 14 cm above the table.
- **Starter cover:** a red PLA cup, 12 cm across and 3 cm deep, with a grommet for the cord, its middle 9 cm above the table.
- **Handle and cord:** a yellow polypropylene T-handle, as sold to replace a small engine's starter handle, with a crossbar 7.1 x 1.9 x 2.4 cm, on 4 mm braided nylon cord. 50 g together.
- **Spring:** inside the cover and block, the cord runs 2:1 round a pulley on the free end of an extension spring of 200 N/m, anchored at the housing's back. The handle then feels 50 N/m. Choose a spring with at least 16 cm of extension, since the stop at 32 cm stretches it 16 cm, and under 2 N of initial tension, so the handle feels under 1 N at home. A stop knot ends the cord at 32 cm.
- **Weight:** 0.55 kg in all, with the handle. Steel washers in the housing, over the plate's middle, make up the weight and keep the balance point there.

Check it before trials, on the table the trials use:

| Check | Reading |
|---|---|
| Spring scale on the handle, pulled out 15 cm | 7.5 N, within 1 N |
| Spring scale on the handle, pulled out 30 cm | 15 N, within 1 N |
| Spring scale pulling the unit sideways, as it starts to slide | 3.5 N (static friction 0.65) |
| Spring scale pulling the unit sideways at a steady pace | 3.2 N (sliding friction 0.6) |

If the sliding readings differ on another table, face the plate's underside with a thin sheet until they match: paper to lower them, rubber to raise them.

In the simulator the unit's surfaces have friction 0.8 static and 0.7 sliding, and the table 0.5, averaged at each contact. The handle's slide carries 50 g, with 50 N/m and 0.2 N s/m of damping, and stops at 32 cm.

## Scoring

See `scoring` in `task.json`, and `task_score` in `task.py`, which `check_scorer.py` tests on hand-built states.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` | The whole task | 1.0, at step 165 (8.25 s), pull 0.30 s, unit moved 2.9 cm at most |
| `one_arm` | A top-speed pull at 1.32 m/s, nobody holding: the unit drags 46 cm | 0 |
| Dream-1, 300 steps at 20 rows per request | The handle did not move | 0 |
| `check_scorer.py` | The scoring table | All agree |

No seed is needed. Runtime per trial: about 1 minute on a shared GPU, at about 4 steps per second.

## Assets

All meshes are made by the task's own code from plain shapes. The T-handle's shape and size follow photographs of replacement starter handles sold for small engines, with no parts copied. No third-party assets. No textures.

## Licence

MIT
