# Check a banknote with a counterfeit pen: pin the 1 g note, draw one stroke, then slide it to the mat its mark calls for

A US $20 note lies flat on a counter's laminate top. A counterfeit-detector felt-tip pen stands uncapped in its cup. The note weighs about 1 g, so the felt tip's drag slides an unpinned note across the counter and smears the mark. One hand presses the note flat by one end; the other takes the pen, draws one stroke at least 1 cm long on the note and lifts off on the paper, then puts the pen back in its cup. The note is then slid along the counter under pressing fingertips, lifted no more than 5 mm, onto the green mat if its mark is amber or the red mat if dark. Amber for genuine and dark brown or black for suspect are the pen's own colours (see Sources).

In the demo the left hand pins the note's end with two fingertips spread across it, so that it cannot pivot about one point; the right sets the felt down with the pen leaning 30 degrees, pressed lightly into the paper, and draws a stroke 15 mm along the note, leaving an amber mark about 13 mm long; it puts the pen back, then slides the note onto its mat under two fingertips spread along it, so that it does not turn.

**Why it matters (Societal relatability).** A familiar moment at a shop counter: a customer pays with a $20, and before it goes in the till the cashier draws a quick line on it with a counterfeit-detector pen. Most people have stood on the other side of that counter and watched it. The pen's iodine ink reacts with starch in ordinary paper, so the line stays light amber on genuine note paper (one-fourth linen, three-fourths cotton) and turns dark brown or black on a suspect note. A business reports a suspect note to the Secret Service (Form SSF 1604), so the green mat stands for the till and the red mat for a suspect note set aside to report. The task is the physical act inside that moment: the mark on a note that weighs about 1 g and slides under the felt's drag unless it is held.

**Ability tested:** holding a 1 g sheet still against a felt tip's drag while drawing a stroke at least 1 cm long that stays on the paper, then sliding the sheet to its place.

**Nearest tasks.** A tape measure held out against its spring while the other hand dabs one mark: there the hold resists a spring and the mark is a touch; here the hold resists the stroke's own drag on a sheet that weighs almost nothing, and the stroke must run 10 mm or more on the paper without sliding it. A card swiped through a reader; sheets picked from a pile by an edge. Making change at a till, where the difficulty was arithmetic.

**Arms.** The instruction does not ask for two hands. In the demo the left pins and the right draws.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
  --instruction "Check the banknote with the counterfeit pen: hold the note flat by one end, draw one stroke at least 1 cm long on it, put the pen back in its cup, then slide the note onto the green mat if the mark is amber or onto the red mat if it is dark, and move your hands away."
```

`CASH_NOTE=suspect` sets a suspect note (the mark comes out dark, and the red mat is right). The standard trial uses the genuine note, so it does not test the sort as a decision; a suspect-note trial does.

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full              # full score, with a video
python demokit.py task.py OUT --no-record --mode nopin,hoverpin,hoverpen,none   # the checks below
python check_score.py                                  # the scorer on hand-built trials, with no simulator
```

The demo reads the note's and the pen's poses to place the hands; it reads no contact force and nothing from the scorer.

## The scene and what is simplified

The note is a rigid plate of a US note's size and weight, 156 x 66.3 mm and about 1 g (the Bureau of Engraving and Printing's figures), 1 mm thick (paper is about 0.11 mm): it does not bend, curl or crease. The counter's top is a laminate sheet on which paper slides easily. Friction is set so that the felt's drag slides an unpinned note (9.9 mm in the `nopin` run) and a pinned one does not, and a pressing fingertip carries the note along the laminate. In a real trial, check the same by drawing on an unpinned note.

The ink is code: a 3 mm dot shows on the note wherever the felt tip has touched the paper while moving across it, amber on the genuine note and dark on a suspect one. The pen is a slim felt-tip marker with a chisel tip, held upright in its cup. The mats are visual only: the note slides over them.

Touch is read from height, as calibrated from logged runs. The felt counts as touching the paper when its lowest point is within 0.6 mm of the paper's face (during the film's stroke it read from about 0 to 0.48 mm), and a stroke counts for 0.5 only if the felt is pressed on the paper, within 0.3 mm of its face, for at least half of it (a felt riding 0.4 to 0.9 mm over the paper, as in `hoverpen`, is not). A fingertip counts as pressing the note when its lowest reported point is at or into the face (pressing reads -0.26 to -0.32 mm; held just over the note, 4.2 mm). A stroke's length is measured over its inked dots only.

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. `check_score.py` feeds the real `Trial` hand-built states for the honest check and each lazy route.

| What happened | Score |
|---|---|
| The tip touched the note while it was pinned | 0.25 |
| One stroke whose mark spans 10 mm or more, all on the paper, pinned throughout, the note moving under 2 mm | 0.5 |
| Also the pen back in its cup, upright within 20 degrees, let go | 0.75 |
| Also the note flat on the mat its mark calls for (all four corners over it), still, every fingertip 3 cm or more from the note's plate, held 1 s | 1.0 |
| Ink on the counter, more than 3 strokes, the note lifted over 5 mm, the pen dropped, or the note on the wrong mat | 0 |
| A stroke under which the note slid over 5 mm (a smear) | at most 0.5 |

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 1,600 steps | 0.0: the model never took the pen; the note stayed where it lay, unmarked, to the step limit |
| `full` | The whole task (the film) | 1.0, at step 566 (28.3 s): the inked mark spans 13.4 mm, the felt firm on the paper, the note moving 0.9 mm under it, pinned throughout |
| `nopin` | The same stroke with the note not pinned | 0.0: the note slid 9.9 mm with the pen (a smear) |
| `hoverpin` | The left's fingertips held just over the note's end, not pressing | 0.0: never pinned (4.2 mm over the note); the note slid 10.6 mm |
| `hoverpen` | The felt held 1.5 mm over the paper instead of pressed 1.5 mm into it| 0.25: it touched the paper on the way down but never firmly along a stroke; its marks span 3.5 mm |
| `none` | The arms wait | 0.0 |

`check_score.py` also runs, on hand-built states: the note left on the wrong mat (0), left where it was checked (0.75), the pen left out of its cup (0.5), a pinned note that slid 3 mm (0.25), a 6 mm stroke (0.25), four strokes (0), the tip down on the counter (0), the note lifted 3 cm onto its mat or 6 mm off the counter on the way (0) and 4 mm (1.0), a hand held 3.5 cm over the note's middle at the end (1.0) or 2.5 cm (0.75), the pen dropped (0), the end held 0.9 s (0.75), the pin hovering 2, 0.9 or 0.5 mm over the note (0), the felt 0.9 mm over the paper (0) or riding 0.55 or 0.4 mm over it (0.25), a 2.5 mm scribble back and forth (0.25), five dabs then one stroke (0), and the note half off the green mat (0.75).

In the demo the right's lift of the pen out of its cup asks for more height than the arm reaches (the IK falls 51 mm short); the pen still clears the cup.

## Runtime and seeds

The demo takes 28 s of simulated time (566 steps), about 2 minutes of wall time on one shared GPU at 640 x 360, start-up apart. There is no randomness: `object_jitter=0`, so no seed is needed.

## A real trial

Use a genuine $20 note for the standard trial, and a blank sheet of ordinary starch paper cut to 156 x 66.3 mm for a suspect one; give each trial a fresh note or a fresh area of one. Any counterfeit-detector pen with a chisel felt tip fits; stand it in a cup of 22 mm bore and 45 mm height. The mats are thin vinyl, 12 x 20 cm. Lay the note on a laminate sheet. To read the slip under a stroke, draw two pencil ticks on the laminate at the note's ends before the trial and measure their offset from the ends afterwards with a ruler, or track a corner in the top camera. Nothing in the trial is sharp, hot or heavy.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- The note's size, 156 x 66.3 mm, and weight: the U.S. Bureau of Engraving and Printing ("The approximate weight of a note, regardless of denomination is one gram.").
- The pen's ink: Dri Mark, Money Guide (https://drimark.com/wp-content/uploads/2020/06/Money-Guide.pdf): "Federal Reserve note paper is one-fourth linen and three-fourths cotton, and contains red and blue security fibers. Dri Mark counterfeit Detection Ink can detect this special paper." Dri Mark is a trademark of its owner, named for identification only.
- The pen's colours and ink: QSR Web, "Is it real or counterfeit? Part 2 Detection devices" (2012, https://www.qsrweb.com/blogs/is-it-real-or-counterfeit-part-2-detection-devices/): "The pen is felt tipped and contains an iodine-based ink and costs a few dollars."; "The mark by the pen will be a light amber color on a genuine bill."; "The iodine solution reacts with starch in wood based paper and the mark turns dark brown or black." The same page notes that the Secret Service does not count the pen as dependable: the task tests the mark, not the pen's verdict.
- A suspect note at a business: uscurrency.gov, Report counterfeit (https://www.uscurrency.gov/report-counterfeit): "If you are associated with a financial institution or business, you may report a suspected counterfeit note by using the Secret Service Form (SSF) 1604."

## Licence

MIT
