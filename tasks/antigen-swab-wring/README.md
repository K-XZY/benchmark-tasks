# Wring a strep-test swab: squeeze the tube on the head, turn 5 times, wait a minute, draw it out under a moving squeeze

The swab step of a CLIA-waived rapid strep A test, as a medical assistant runs it in a clinic. The insert (ACON Flowflex Plus Strep A, FDA CLIA waiver
CW250006) says: "Rotate swab 5 times while squeezing, keep swab in test tube for 1 minute, and remove swab while rotating and squeezing the tube before
testing extracted material." The squeeze wrings the sample out of the swab head into the buffer.

The count of five is given in the instruction and nothing is read or chosen; the difficulty is in the hands:

- **A squeeze on a part that turns.** One hand keeps the soft tube squeezed onto the swab head at the tube's bottom, hard enough to bear on it but not
  so hard that the swab cannot turn, while the other turns the swab inside the squeeze.
- **Turning a thin shaft.** A parallel gripper turns the 3 mm shaft a third of a turn at a time: grip, turn, let go, turn back, grip again. Between strokes
  the squeeze alone holds the swab, and each grip must not pull it out of the squeeze.
- **A minute, then out through the squeeze.** The swab stays in the tube for the insert's minute. It is then drawn from the bottom up, turning, with the
  tube squeezed on the head wherever the head is, so the head is wrung as it leaves. A squeeze dents the tube only under the fingers, so the squeezing
  hand moves up the tube with the head. Let the head rise out of the squeeze, or draw it before the minute is up, and the wring does not count.

**The arms.** In the demo the right arm squeezes the tube and the left arm works the swab. The scoring asks for the result only: any division of the work
is allowed.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `mechkit.py` and `demokit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 8000 \
    --instruction "Do the strep test's swab step: put the swab head down to the bottom of the tube, squeeze the tube on the head and turn the swab 5 times, leave it in the tube for 1 minute, then take the swab out while squeezing the tube on the head and turning, drop it in the bin, let go of the tube and move both hands away."
```

The scene is the same for every seed.

## The demo

From this folder:

```bash
python demokit.py task.py OUT                                   # the full-score run, with video
python demokit.py task.py OUT_checks --mode open,yank,resqueeze,holdbin,shallow,nowait,fixed,creep,none --fast    # the failure checks
```

The demo is a scripted reference, not a policy. It reads the swab's and the tube walls' poses from the simulator; all of them are in view of the cameras.
It never reads the scorer.

1. The left hand takes the swab by its handle, over the box's edge, lifts it and swings its head down, then lowers the head to the tube's bottom.
2. The right hand comes level from the front and squeezes the tube just above its holder, over the head's upper half.
3. The left hand lets go, turns down above the swab and takes the shaft again 10.5 cm above the head's tip. It turns the swab in 21 strokes of 120
   degrees (grip, turn, let go, turn back), each time first centring its open fingers on the shaft where it stands.
4. The swab stays in the squeezed tube for the insert's minute.
5. The left hand takes the shaft once more and keeps it for the whole draw. It lifts the swab 10 mm at a time, turning it 40 degrees in each of the
   second to fourth lifts. After each lift it holds the swab still while the right hand eases the squeeze, moves up the tube to the head as it now is
   (its fingers' upper edge 3 mm under the head's top) and squeezes again.
6. The right hand lets go of the tube; the left lifts the swab clear of the rim and drops it in the bin. Both hands back away.

It scored 1.0 in GPU physics in 170.3 s of simulated time (3,406 steps): 5.1 turns counted at the bottom while squeezed, the minute, then the draw
from the bottom to the mouth, squeezed at every step it rose, turning 0.30 turn. A CPU run without cameras takes about 4 minutes.

Failure checks (`--mode`, CPU physics, one session):

| mode | what the script does | score |
|---|---|---|
| `open` | turns the swab in the tube with the tube not squeezed | 0.0 |
| `yank` | turns the swab 5 times squeezed and waits the minute, then lets go of the squeeze, takes the swab and pulls it straight out | 0.7 |
| `resqueeze` | turns 5 times squeezed and waits, lets the squeeze go, lifts the swab near the rim, squeezes again and draws it the last part, turning | 0.7 |
| `fixed` | turns 5 times squeezed and waits, then draws the swab out in one pull, turning, the squeeze left where it was at the bottom | 0.7 |
| `creep` | as `fixed`, but the pull is slow (0.27 mm a step) | 0.7 |
| `nowait` | turns 5 times squeezed, then draws it out as the demo does, at once, without the minute | 0.6 |
| `holdbin` | does everything, but stands the swab in the bin still held by its top | 0.8 |
| `shallow` | lowers the head only 2 cm into the tube; the squeeze, just above the holder, closes below the head, so no turn counts | 0.0 |
| `none` | does nothing | 0.0 |

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold: `score()` is 1.0 only once the 1.0 state has
held 20 control steps (1 s) in a row, and `success()` is true from then on, so the two agree.

The tube's two soft sides are each 8 segments of 9 mm. A segment is squeezed when both its sides are pressed in 2.3 to 3.4 mm (they touch a 5 mm head at
2.5 mm; with nothing between them they close fully, to 4.5 mm). The head is squeezed when at least 5 mm of its length lies in squeezed segments, and the
swab is within the bore and upright within 15 degrees.

- **0.12 per full turn**, **0.6** at 5 turns. A turn counts while the head is squeezed with its tip within 10 mm of the tube's floor; turns back count
  against the total.
- **0.7:** after 5 turns, the swab left in the tube with its head at the bottom for the insert's minute (60 s in a row). Lifted early, the minute starts
  again.
- **0.8:** after the minute, the head drawn from the bottom up and out of the tube, rising only while squeezed: while it is in the soft walls it may
  stand at most 1 mm above the highest point it has reached squeezed (that allows for an eased squeeze while a hand regrips), and it may rise no faster
  than 3 mm a step (60 mm/s); and turned at least 0.15 turn during the draw while squeezed. A head that rises otherwise ends the draw: it must go back to
  the bottom and start again (the minute is not repeated).
- **1.0:** after that, the swab in the bin (its middle inside the bin's walls), every fingertip at least 5 cm from the tube's axis and from every point
  along the swab, held 1 s.
- **0:** if the swab falls off the table (latched).

**What the cameras show, in a real trial.** A phone camera on a stand level with the tube, 30 cm beyond it, looks back at it along the arms' forward
direction, across the squeeze, so that both squeezed sides and the dent between them are in view; the station's top camera films too. This side camera
is for real trials only: it is not in the simulation, whose films come from the station's cameras (in the demo the right wrist camera shows the band
and the squeezing hand stepping up the tube). The real extraction tube is translucent, so the head shows through its wall; the simulation draws it
opaque.

- **Depth.** A band on the shaft is placed so that its lower edge is level with the tube's top edge (in the model, the top of the tube's wall, under its
  3 mm rim ring) exactly when the head's tip is 10 mm above the tube's inside floor (on the modelled tube, 64 mm from the tip). With the band at or below the top edge on the side camera, the tip is within 10 mm of
  the floor. The head's height in the tube is read off the band in the same way.
- **The squeeze.** On the side camera, the fingers' pads press the tube's sides in where the head is, and the dent closes on the head. The pads must
  span at least 5 mm of the head's 20 mm, at every frame in which the head rises.
- **Turns** are counted from the blue tab near the handle's end on the top camera. **The minute** is timed on the video clock, from the end of the fifth
  turn to the first frame of the draw.

`check_score.py` runs the scorer on hand-built states, with no simulator.

## Real trials: the build

- **Test kit.** A CLIA-waived rapid strep A kit with its extraction tube, buffer and sterile throat swab (for example ACON Flowflex Plus Strep A). Measure
  the tube (modelled 75 mm long, 13 mm across, 10 mm bore) and the swab (modelled 15 cm long, a 3 mm shaft, a 5 x 20 mm flocked head). Fix a small blue
  tab to one side of the swab 3 cm from its end, and a 2 mm blue band round the shaft at the depth for the tube in hand: its lower edge at the tube's
  inside depth less 10 mm from the head's tip (64 mm on the modelled tube, whose rim ring is not counted). Each trial uses a fresh tube with buffer and a
  fresh swab.
- **Squeeze check.** Before a trial, check on the bench that a squeeze about a finger wide (36 mm) with about 3 N dents the tube onto the swab head
  there, and only there (the model's segments give 0.25 N per mm each, four under a finger, up to 4.5 mm on each side).
- **Tube holder.** A painted steel block 4 x 4 x 2.5 cm at (28, 8) cm with a 14 mm socket 10 mm deep, the tube standing in it upright, its bottom 1.5 cm
  above the table. The tube's 65 mm above the socket are free to squeeze.
- **Side camera.** A phone on a stand level with the tube's middle, 30 cm beyond it at about (58, 8) cm, looking back at it along -x (across the
  squeeze), filming the trial.
- **Kit box and bin.** The kit's box (about 10 x 10 x 5 cm) centred at (20, 20) cm with the swab lying on it along y, its head tip at (20, 25) cm, its
  handle 5 cm over the edge. A waste bin 12 x 12 x 8 cm centred at (7, 20) cm.
- **Safety.** Unused swabs and buffer only; no patient samples. The buffer is a mild irritant: eye protection for anyone near, and wipe spills.

## The model behind it

The tube is one articulation fixed in its holder. Its frame is the tube's floor and two rigid side walls 3 mm thick, 1 mm from the head. Only the other
two walls, the ones across a squeeze from the left and right, are soft: each is 8 links, 9 mm each from the floor to the rim, on sprung slides (0.25 N/mm each, 4.5 mm travel): a gripper squeezing the tube pushes in the
segments under its fingers until they bear on the head, and the others stay where they are. A real tube's dent is smooth; here it is in 9 mm steps.
The buffer and its extraction are not simulated: a turn counts while the segments on the head bear on it. The swab is a free rigid body of 8 g (a real
swab is 1 to 2 g; the heavier body keeps contact steady). Friction: the wet walls 0.1, the swab 0.3 (assumed). The insert's minute is timed by the
scorer. The holder is painted steel, the bin polypropylene.

## Attribution

The task statement quoted in `task.json` is from the O*NET database (O*NET OnLine, task statements for 31-9092.00) by the U.S. Department of Labor,
Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence. O*NET is a trademark of USDOL/ETA. The insert's wording is quoted from
the FDA's public CLIA waiver record CW250006.

## Assets

Every mesh is generated by `task.py` and `mechkit.py`. There are no textures and no third-party assets.

## Licence

MIT
