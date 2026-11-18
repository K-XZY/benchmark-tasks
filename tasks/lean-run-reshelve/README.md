# Press a leaning run of books upright and hold it while the other hand slides a book into the 0.7 cm gap

A shelf has a gap, and three books lean into it. Putting the green book back is not a simple insert: the leaning books cannot stand on their own. Their cases are racked, as books left leaning become, so pushed upright and let go they tip back onto the steel bookend and close the gap (at rest 17.4 degrees; held within 0.5 degrees while pressed; back at 17.5 degrees within 1 s of release). One hand must press the run upright with its fingertips and keep pressing while the other slides the green book fore-edge first into a gap with 3.5 mm of clearance each side, under the pressing fingers. One hand alone scores 0: in GPU runs the right hand alone, the left hand alone, and the two one after the other (the right presses the run upright and lets go, then the left brings the book) all scored 0, because the book cannot enter a leaning run.

**How it differs from removing a book.** Taking a book out of a packed, stable row never touches the neighbours, which stand by themselves. This task is not that act reversed: the neighbours here do not stand, and must be pushed upright and held before the insert can happen. The hold must last until the book is in, and the scorer reads it: a finger of the holding hand must be in contact with the run from the book entering the gap until it is seated (see Scoring). With the hold released at 4 cm in, the run tips back and the credit stays at 0.5 or less (0 in our run).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `mechkit.py`, `demokit.py` and `gatekit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 1800 --seed 1 \
    --instruction "Put the green book back on the shelf, in the gap beside the books that lean into it: push those books upright and hold them, slide the green book in spine out until its spine is flush with theirs, then let go and move both hands clear."
```

The scene has no random start (`object_jitter=0`); the seed changes nothing.

## The demo

```bash
python demokit.py task.py OUT
```

The scripted hands score 1.0 at step 485 of 1,800 (24 s of simulated time, about 2 minutes of wall time on one GPU at 640 x 360). The right hand presses the run upright with its fingertips and holds, in contact with the run at every step of the green book's travel (3.8 N during it); the left hand brings the green book in. `python check_score.py` checks every score line on hand-built states, with no simulator, including each route that must not score: a displaced or scattered run, a moved, lifted or turned bookend, a run book once knocked over, a hand or forearm left resting on the books, a seated book with the hold missing for one step, none, only a light touch, only after seating, a hover, or with the run leaning, and a book still turning.

Dream-1 (one run, seed 1, 1,800 steps, no session loss): score 0.0. The model did not complete the task.

## Scoring

`--scorer task_score` prints the total. A rung counts only after the end state has been at rest with both hands 3 cm clear for 2 s.

- **0:** a run book fell at any time or is not where it stood, or the bookend moved 6 mm, turned 5 degrees or tilted 4 degrees.
- **0.25:** the green book stands in the gap, 3 cm in.
- **0.5:** half its depth in. The score never rises above 0.5 unless the hold was in contact for the whole insertion: from the green book's fore-edge passing 5 mm beyond the run's spines (entering the gap) until its spine is within 10 mm of theirs (seated), at every step a fingertip of one hand presses on a run book with a contact force of 0.1 N or more, read from the simulator's contacts, and every run book is within 10 degrees of upright. One step without it caps the trial at 0.5 for good; being near the run does not count. In the lazy routes, GPU runs: one hand alone, the right then the left, hold released before the insertion, and a hover with the fingers 2 cm off the run all scored 0.
- **0.75:** spine square to the shelf within 6 degrees and flush within 10 mm, the green book within 12 degrees and every run book within 10 degrees of upright.
- **1.0:** the same with all within 8 degrees.

## Building it for a real trial

- Shelf: a plain pine bookcase shelf (any open pine bookcase's shelf, or a pine board) 26 x 46 cm, 2 cm up, with a 30 cm back panel and a left end panel; three packed books at the left end.
- Run: three 6 x 9 in trade hardcovers, 2.6, 3.2 and 3.0 cm thick, 0.55, 0.74 and 0.70 kg, ordinary used books, made to lean with a strip of 5 mm foam board, balsa or plywood, 15 mm x 150 mm, taped under the tail of each left board (the lower back edge of the left cover), so none stands straight on its own; thicken a strip in 1 mm steps if its book stands. Check: stood upright and released, each tips back.
- Bookend: a steel L of 1.2 mm sheet, 0.34 kg, plate 14 x 13 cm, base 14 x 13 cm under the gap and the run; mark its place with tape.
- Green book: a trade hardcover 13.0 x 19.5 x 3.3 cm, 0.55 kg.
- Friction: cloth 0.4, pine and steel 0.5. The simulated contact force of the fingers on the run (2 to 5 N) is read from the simulator, not measured; calibrate by pressing the run upright with a force gauge.

## Assets

Every mesh is generated by `task.py` and `mechkit.py`. There are no textures and no third-party assets.

## Licence

MIT
