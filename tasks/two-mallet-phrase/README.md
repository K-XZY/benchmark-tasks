# Play a six-note phrase from a printed card, with a mallet in each hand

A glockenspiel in two groups of three coloured bars, 40 cm apart, with a felt mallet beside each and a printed card of six coloured squares. The arms take turns to strike the card's six notes, one each half second, then lay the mallets back.

**Ability tested:** striking in time with a tool in each hand, reading the order from a printed card.

**Both arms are forced by a scoring rule and the tempo, not by reach alone: a note struck with the other side's mallet scores 0.**

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 400 \
  --instruction "Play the notes on the card in order, left to right, at one note every half second, with one mallet in each hand, taking turns, then lay the mallets back in their cradles."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full --seed 0      # full score, with a video
python demokit.py task.py OUT --no-record --mode full --seed 1      # the second card
python check_scorer.py                                  # the table and the note rules, with no simulator
```

The demo reads the card from the top camera's pixels, and each mallet's position as a camera could see it. It finds where each head is against its gripper once, at rest, and then plays to a fixed beat.

## Scoring

See `scoring` in `task.json`, and `task_score` in `task.py`, which `check_scorer.py` tests on hand-built states.

## Results

| Run | What it does | Score |
|---|---|---|
| Demo, seed 0 | The first card, six notes 0.50 s apart | 1.0, at step 245 (12.25 s) |
| Demo, seed 1 | The second card, read from the camera | 1.0, at step 246 (12.3 s) |
| Dream-1, default seed, the second card, 400 steps at 20 rows per request | Both arms moved over the bars; neither mallet was lifted | 0 |
| `check_scorer.py` | The table, gaps, order, the wrong mallet, a seventh note, the cards | All agree |

The seed sets the card: the reset seed modulo 3 of 0, 1, 2 gives the first, second and third card. `inspect-robots run` derives the reset seed from its `--seed`: the default `--seed 0` gives reset seed 1415158438, and so the second card. `demokit.py --seed` passes its seed to the reset as it is.

Demo seed 2's card was not played. Runtime per trial: about 1.5 minutes on a shared GPU, at 3 to 4 steps per second.

## Assets

All meshes are made by the task's own code from plain shapes. No third-party assets. No textures.

## Licence

MIT
