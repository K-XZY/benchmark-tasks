# Play a six-note phrase from a printed card, with a mallet in each hand

A small xylophone in two groups of three coloured bars, 40 cm apart, with a felt mallet beside each and a printed card of six coloured squares. The arms take turns to strike the card's six notes, one each half second, then lay the mallets back.

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

## Building it for a real trial

The instrument reproduces what the simulator models: six bars that each give a few millimetres on springs, two mallets with a flat grip, two cradles and a card. Notes are read from a recording.

- **Bars:** hardwood, painted red, yellow, green, blue, white and purple, each 11 x 3.8 cm, 8 mm thick and 30 g (about 0.9 g/cm³). Their tops are 3.5 cm above the table, 5 cm apart centre to centre in each group.
- **Rails:** two per group, dark hardwood, 17 cm long along y, 1.2 cm wide and 2.4 cm high, 8 cm apart centre to centre, under each bar 1.5 cm in from its ends. Each group's rails are joined and clamped to the table. Each rail has a notch 1 mm deep under each bar.
- **Mounting:** each bar stands on two coil springs, one on each rail, of 150 N/m each, so 300 N/m for the bar: it goes down 1 mm under 0.3 N. Each spring stands in a hole 5 mm across and 12 mm deep, drilled in the rail's top under the bar. It is music wire 0.3 mm thick, coiled 4.3 mm across, with about 8 active coils and a free length of 16 mm. At rest it stands 15 mm long, carrying half the bar's weight; at the 4 mm stop it is 11 mm long, well clear of its solid length of about 3 mm. A cord through a hole near each end of the bar keeps it from rising above rest, as cords or pins hold a real xylophone's bars. The springs are set to carry the bar's own weight at rest, so it sits lightly up against the cords. The bar's underside rests 3 mm above the rails, and the notches let it go down at most 4 mm.
- **Mallets:** 18 cm shafts of 1 cm beech dowel, each with a felt-covered head 2.4 cm across at one end, and a flat hardwood grip at its middle, 4 cm along the shaft, 1.2 cm thick and 2.2 cm high, faced with thin rubber. A brass insert in the grip brings each mallet to 60 g.
- **Cradles:** two wooden blocks per mallet, 11 cm apart along the shaft, each 1.5 x 4 cm and 3.5 cm high, with two lips 1.2 cm high on top that leave a 1.6 cm slot for the shaft, whose axis then lies 4 cm above the table.
- **Card:** white card stock, 8 x 40 cm and 2 mm thick, with six 5 cm squares printed in the bars' colours, 8 mm apart.

Check each bar before trials: a 30 g weight on its middle takes it down about 1 mm.

In the simulator each bar is a rigid body on a vertical slide with 300 N/m and 1.5 N s/m of damping, from rest down to 4 mm. The mallets are free bodies of 60 g with friction 1.2 static and 1.0 sliding.

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
