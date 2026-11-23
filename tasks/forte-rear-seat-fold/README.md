# Fold a sedan's rear seatback from the trunk: one hand holds the spring-return knob, the other tips 9 kg past upright

The 40% rear seatback of a 2019-2021 Kia Forte sedan stands on the bench as it sits in the car, seen from the trunk: hinged at its foot, reclined 25 degrees toward the robot, held by a striker latch at its top outer corner, with the seatback lock knob on the trunk-side trim beside its top. The robot does what Kia's owner's manual gives: with the headrests already lowered, "pull out the seatback locking knob in the trunk, then fold the seat toward the front of the vehicle." One hand pulls the knob out 20 mm and holds it; the other pushes the seatback top forward until it passes upright, and gravity lays it on the cushion.

**What makes it hard.** The knob springs back. Let go before the seatback has swung 5 degrees off its striker and the latch catches it again, so the hold has to last into the push, and the push comes from the far side of the trunk opening, at the top of a 0.6 m seatback. The seatback weighs about 9 kg with its centre about 0.3 m from the hinge: reclined 25 degrees, that is about 11 N m about the hinge, about 19 N pushed square to the seatback at its top, falling to zero at upright. Pushing on the closed latch moves nothing.

**Who does it, and how often.** Folding a rear seat to load something long through the trunk is a first-hand act for most drivers: the US had 240 million licensed drivers in 2024 ([FHWA, Nation's Highways 2026](https://www.fhwa.dot.gov/policyinformation/pubs/our_nations_highways_2026/drivers.cfm): "The Nation reached 240 million licensed drivers in 2024"), and the Forte sedan alone sold 115,929 cars in the US in 2021 ([Kia America full-year 2021 sales by model](https://www.best-selling-cars.com/usa/2021-full-year-usa-kia-america-sales-by-model)), with its 60/40 folding rear seat. How often a driver folds the seat is not surveyed.

**How the demo does it.** The left hand passes under the trim block to in front of the knob, comes in level along +x, closes its jaws across the front of the knob's flat pull tab (12 mm thick, 25 mm tall) and draws it 20 mm out to its stop. The right hand, closed, sets its fingertips on the seatback's trunk face 52 cm up from the hinge (8 cm below its top) and walks them forward along the seatback's arc, never more than 3 degrees ahead of the measured angle. Once the seatback reads 8 degrees, the left hand lets go of the knob and backs off; the right hand carries on, sliding down to 45 cm up as the seatback nears upright, and backs off at 40 degrees. Gravity carries the seatback down onto the cushion.

## Real items

- Kia genuine rear seatback, Back Assembly-Rr Seat 89300M7330B4U, 2019-2021 Kia Forte sedan (BD): [kiapartsnow listing](https://www.kiapartsnow.com/genuine/kia-back-assy-rr-seat~89300m7330b4u.html). New, it is a dealer part priced on that listing; the real trial uses a used one with its latch, knob and cable, typically $50 to $150 from a salvage yard (estimate).
- Use steps: [Kia Forte 2024 owner's manual, folding the rear seat](https://ownersmanual.kia.com/full_webhelp/BDm/2024/en_US/topics/t00043.html). Knob location: [Kia Forte Features & Functions Guide, p. 28](https://manualslib.com/manual/2484180/Kia-Forte.html?page=28) ("Pull Seatback Lock Release Lever", "Lever located in trunk"); the owner's manual names the same part "the seatback locking knob(1) in the trunk", and its figure shows it at the top corner of the trunk opening beside the seatback top: a flat tab marked PULL on a short rod, pulled straight back toward the trunk opening, which the sim models. Both free online ($0).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder, the scripted demo (the README command):

```bash
python demokit.py task.py OUT --mode full                                                     # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode none,pushonly,pullonly,letgo,shove            # failure checks, below
```

Scores land in `OUT/<mode>/result.json` (`task_score`).

| Mode | What the robot does | Score (GPU) |
|---|---|---|
| full | pull and hold the knob, push the seatback past upright, let go | 1.0 at step 233 (11.65 s) |
| none | nothing | 0 |
| pushonly | push the seatback top for 40 s without touching the knob | 0 (the latch holds) |
| pullonly | pull and hold the knob, no push | 0 (the seatback stays reclined) |
| letgo | one hand at a time: pull the knob, let go, then push | 0 (the latch has caught again) |
| shove | no knob; the push hand driven 8 cm into the latched seatback top | 0 |
| Dream-1 (dream-1-beta) | the model drives both arms from the three camera views for the full 1,200 steps | 0.0 |

## Scoring

Judged on the state at the end, from the seatback's hinge angle (0 at the latched pose). Off the striker: swung 5 degrees or more, which needs the latch released (0.33). Past upright: swung more than the 25-degree recline, so the seatback is beyond vertical (0.67). Folded: within 10 degrees of the folded stop on the cushion, 113 degrees (1.0). Nothing is remembered from earlier in the run: a seatback lifted back up loses its tier.

## Sizes

| Item | Sim | Real | Source | Sim vs real |
|---|---|---|---|---|
| Seatback, 40% section | 0.55 m wide, 0.60 m tall, 0.10 m thick, rigid | same, foam on a steel frame | estimate from the part listing photos and the manual figure; no published drawing | rigid face: the push point does not sink in; neutral |
| Seatback mass | 9 kg, centre 0.30 m up from the hinge | 8 to 11 kg (bound: a 40% sedan seatback of steel frame and foam); weighed in the real trial | estimate | both ends pass (range check) |
| Recline at the latch | 25 degrees from vertical | 23 to 28 degrees (sedan rear seatback angles) | estimate; angle gauge in the real trial | both ends pass (range check) |
| Folded stop | 113 degrees of swing, lying on the cushion | lying on the cushion (manual figure) | manual figure | neutral: gravity does this part |
| Hinge | damping 2 N m s/rad, friction 0.5 N m | a bare pivot bushing: small | estimate | both ends pass (0 and 4 N m s/rad, 0 and 1.5 N m) |
| Lock knob pull tab | flat tab 25 mm tall, 12 mm thick, 15 mm long along the pull (20 mm also run) | a flat tab marked PULL on a short rod, about 25 mm tall, 15 to 20 mm long | [owner's manual](https://ownersmanual.kia.com/full_webhelp/BDm/2024/en_US/topics/t00043.html) figure OBDM031068L (seatback locking knob) for the shape; sizes estimated from it; calipers in the real trial | the shortest real head, so the hardest; 20 mm passes too |
| Knob stem (head to trim) | 10 mm, the knob standing 25 mm out of the trim (20 mm and 40 mm also run) | 10 to 20 mm, so the knob stands 25 to 40 mm out of the trim | estimate from the same photos; ruler in the real trial | the shortest real stem, so the hardest; 20 mm passes too |
| Knob travel | 20 mm | about 20 mm to its stop | estimate; ruler in the real trial | neutral |
| Knob return spring | 350 N/m, about 7 N at full pull | 5 to 10 N at full pull | estimate (cable release springs); spring scale in the real trial | both ends pass (range check) |
| Latch release | at 15 of 20 mm of pull | 10 to 19 mm (near full pull) | estimate; ruler in the real trial | both ends pass (range check) |
| Striker clearance | 5 degrees of swing (about 5 cm at the top) | 3 to 7 degrees | estimate; angle gauge in the real trial | both ends pass (range check) |

The latch is a stiff joint lock on the hinge (50,000 N m/rad, effort 2,000 N m: no push of the arm moves it), released while the knob is pulled 15 mm or more, and engaged again if the knob returns while the seatback is under 5 degrees, as a rotary latch whose striker is still in its mouth does. The knob is a colliding sprung slider the hand must pull; the seatback is a colliding hinged body under gravity, tipped by the hand's contact. Contact friction 0.9 static, 0.8 dynamic (the push is along the face's normal, so friction matters little).

**Range check.** Each estimate was run at both ends of its bound (environment variables `FRSF_KG`, `FRSF_RECLINE`, `FRSF_SPRING`, `FRSF_RELEASE`, `FRSF_HEAD`, `FRSF_STEM`, `FRSF_DAMP`, `FRSF_FRIC`, `FRSF_CLEAR`). The default run already has the knob at the low end of both its bounds (15 mm head, 10 mm stem): full 1.0 at step 233. Hardest end (11 kg, recline 28 degrees, 10 N knob spring, latch freed only at 19 mm of pull, damping 4, friction 1.5 N m, striker clearance 7 degrees): full 1.0 at step 246, pullonly 0. Easiest end (8 kg, recline 23 degrees, 5 N spring, latch freed at 10 mm, 20 mm head on a 20 mm stem, no damping or friction, 3 degrees): full 1.0 at step 213, letgo 0, pushonly 0.

## Real trial

**Bench setup.** A used 40% rear seatback for a 2019-2021 Forte sedan with its hinge brackets, its own seatback lock (latch) assembly, the trunk-side lock knob with its cable, and the latch striker, bought together from a salvage yard or a used-parts seller (typically $50 to $150 for the seatback; an estimate). The seatback's hinge brackets bolt to a 19 mm plywood base (60 x 70 cm, clamped to the bench with two C-clamps), the hinge 6 cm above the bench top and 48 cm in front of the midpoint between the arm bases, spanning y -38 to 17 cm. The striker bolts to a plywood upright (19 mm plywood, 15 cm wide, 70 cm tall, screwed to the base with two steel corner braces) behind the seatback's top outer corner, set so the latched seatback reclines 25 +/- 1 degrees (angle gauge). The knob's housing screws to a plywood trim block (6 cm deep, 12.5 cm wide, 22 cm tall, its trunk face at x 22.5 cm, spanning y 18.5 to 31 cm and 40 to 62 cm above the bench, on a 2 x 4 post screwed to the base) beside the seatback top, the knob's face at x 20, y 27, 52 cm above the bench, its cable run to the latch as in the car. A 54 x 55 cm sheet of 1 cm closed-cell EVA foam (an interlocking gym floor tile, cut to size) stands for the cushion top in front of the hinge; it only catches the folded seatback, and any foam 1 to 2 cm thick does (check: the folded seatback lies on it within 10 degrees of the angle measured at setup). Rig beyond the arms: about $200 (seatback, plywood, bolts, gauges; estimate).

**Bench checks before trials.** Weigh the seatback (luggage scale, 0.1 kg): accept 8 to 11 kg. Hook a spring scale (0 to 50 N) at the seatback top, square to it, with the latch open: the reading that starts it moving is about 19 N at 9 kg (11.2 N m over the 0.6 m seatback); accept 16 to 24 N (8 to 11 kg). Pull the knob with a 0 to 20 N spring scale: accept 5 to 10 N at full pull; mark with a ruler (1 mm) the pull at which the latch frees (accept 10 to 19 mm). Measure the knob with calipers (0.1 mm): its flat pull tab about 25 mm tall, 10 to 14 mm thick and 15 to 20 mm long along the pull, standing 25 to 40 mm out of the trim (stem 10 to 20 mm); full pull about 20 mm. With the knob held, swing the seatback slowly and read on a digital angle gauge (0.1 degree) on its trunk face the swing at which the latch can no longer catch: accept 3 to 7 degrees.

**Reset.** About 30 s, nothing consumed: lift the seatback by its top and push it back until the latch clicks; check that the knob has sprung back and the seatback does not move when pushed.

**One trial.** Start with both arms at their ready pose and the seatback latched. Run the policy for 60 s.

**Scoring by hand.** A digital angle gauge on the seatback's trunk face, zeroed at the latched pose, read at the end of the trial: 5 degrees or more scores 0.33; more than 25 degrees (past upright) scores 0.67; lying on the cushion, within 10 degrees of the folded pose measured at setup, scores 1.0. A video from the side (30 fps, the whole swing from latched to folded in view, a 10 cm scale on the trim block) records the run for review. There is no whole-trial or void rule: the score is the end state alone.

**Gripper protection.** None needed: the knob is plastic and the push is on fabric.
