# Take out an interrupted suture: forceps lift the knot, a stitch cutter cuts flush, then draw it out toward the wound

A silicone suture practice pad lies on the table with one interrupted nylon stitch across its closed incision. The robot holds dissecting forceps in one hand and a single-use stitch cutter in the other. The forceps grip the knot and lift it a few millimetres; the cutter's hook slides in under the leg away from the knot and cuts it flush with the skin; the forceps draw the stitch out by the knot, pulling back toward the wound so its edges are not drawn apart, and lay it on a gauze square. Both instruments go back on their trays.

**Where it comes from.** Medical assistants have the core task "Help physicians examine and treat patients, handing them instruments or materials or performing such tasks as giving injections or removing sutures" (O*NET 31-9092.00, about 811,000 US jobs; USDOL/ETA, CC BY 4.0; O*NET is a trademark of USDOL/ETA). Limbs & Things, a maker of suture practice pads, teaches the removal in its suture aftercare manual (https://limbsandthings.com/media/busponzi/suture-aftercare-manual.pdf): "Grasp the knot with dissecting forceps, and then cut the suture precisely with scissors, flush with the tissue surface, so that the exposed length of suture (which may be contaminated) does not have to pass through the tissues", then "Remove the suture by pulling back towards the wound so that the wound edges are not drawn apart by a distracting force." UK and EU suture removal kits pair the forceps with a single-use stitch cutter, as here; US kits mostly pair them with small Littauer scissors, which make the same cut. Limbs & Things is named as a source only; this task is not affiliated with it.

**What this task tests.** Two instruments, one in each hand, working together on one strand at a point a millimetre or two across. The forceps are closed through their sprung arms by the gripper's squeeze, and only their jaws hold the knot, by friction: they must keep it up while the other hand works (lifted 1 mm, the stitch leaves room under it for the hook; the score asks for 2 mm; pulled 5 mm, it tears through the skin). The cutter's hook must slide in level under the lifted stitch on the leg away from the knot and cut with no more than 2 mm of the leg left standing above the skin, without pressing its foot 2 mm into the silicone. Then the jaws must pull the cut stitch out of the skin themselves, against its drag, back toward the wound (a pull away from it opens the incision), carry it to the gauze and let it fall there.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Take out the stitch: lift the knot with the forceps, cut the leg away from the knot flush with the skin with the stitch cutter, draw the stitch out toward the wound, put it on the gauze and the instruments back in their trays, and take your hands away from the pad."
```

A trial runs up to 2,400 steps (120 s of simulated time at 20 Hz). One fixed layout; no seed changes the scene.

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                   # the full-score reference, filmed
for m in nolift away above preclosed hover drop; do python demokit.py task.py OUT_$m --no-record --no-cameras --mode $m; done   # the failure checks, each in its own session
python check_score.py                                                       # the hook's way under the stitch, the instruments' return and the scoring table, no simulator
```

The demo is a scripted reference: it reads the stitch's and the instruments' poses from the simulator, and a policy gets only the cameras. The left hand takes the forceps from their rest lightly (0.8 N, under the arms' 1.6 N spring preload, so the tips stay open) and holds them upright, the jaws closing along x. It sets the open jaws either side of the knot, squeezes them shut on it at 6 N, and lifts the knot 3 mm; while the right hand works, it keeps the knot 3 mm up by the knot's own observed height. The right hand takes the stitch cutter by its flat blade and leans it 50 degrees back toward -y, the hook's notch opening toward -x, so the two hands work the one stitch from either side of it. It brings the hook down to the skin 10 mm to the +x side of the far leg, the notch's floor 0.6 mm over the skin line, and slides it in level across the leg in one smooth motion, steered by the hook's own observed place: the leg goes into the notch through its mouth and, pressed on, meets the notch's sharp inner edge and is cut at the skin line. If the leg slips past the notch, the hook backs out 10 mm and slides in again, up to three tries. The forceps then pull the cut stitch out by the knot along its path, 13 mm up and 3.5 mm toward the wound, against its drag; out of the skin, it is a free strand in the jaws, which carry it over the gauze and open 2.5 cm above it, so it falls onto the gauze. The cutter goes back on its tray; the forceps, now held shut at 6 N, go back on theirs, lowered until their tips reach the rest's floor before the hand lets go. Both hands rise and move away.

**Result** (scripted, GPU physics on an NVIDIA GeForce RTX 4070 Ti SUPER, 20 Hz, filmed at 640 x 360): 1.0 in 1,081 steps (54.05 s of simulated time), within the 2,400-step limit. The jaws closed on the knot (6.1 mm of their 9 mm opening) and lifted it 3.9 mm at most; the far leg was cut 0.5 mm over the skin line on the first try (the notch floor's lowest while the leg was in it), the cutter's lowest corner never more than 1.4 mm into the silicone; the wound's gap stayed under 0.1 mm. The jaws then pulled the cut stitch out along its whole path, carried the free strand about 22 cm to the gauze (closed on its knot, 6.4 mm of their opening), and opened 2.5 cm over it; it came to rest on the gauze, 15 mm from its middle. Both instruments ended in their trays (the forceps standing in their rest, the cutter lying in its tray). The film's first frame is the start state on all three cameras: no black or flat frames on any camera.

**Model run** (Dream-1, on the revised task as filmed, 20 rows per request, seed 1): no score. The provider's API refused the first inference request, before the policy had acted: "DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Cutting suture near skin with sharp tool risks cutting a person." The trial ended on the refusal, so the scorer gave no score; this is not a 0 the policy earned. The runs on the earlier versions of the task were refused the same way, and the instruction was not changed. The model video holds the 33 frames (1.65 s) from the start to the refusal, from all three cameras.

| Failure check | What it does | Score |
|---|---|---|
| `nolift` | the forceps stop 4 mm over the knot and never take it; the cutter goes for the far leg as in the demo | 0.0 in 1,048 steps: the knot never lifted, so the hook never got under the stitch and nothing was cut |
| `away` | as the demo, but the stitch is pulled out away from the wound | 0.5 in 1,098 steps: the far leg cut flush, but the pull opened the incision 4 mm (the knot-side half on its spring) and the stitch came only 43 % of the way out along its path |
| `above` | the knot lifted as in the demo; the hook brought straight down through the loop onto the far leg, then pressed across it | 0.25 in 560 steps: the knot lifted; the hook never got under the stitch the way a hook can, so nothing was cut |
| `preclosed` | the forceps squeezed shut before they reach the knot, then pressed to it and lifted; the rest as in the demo | 0.0 in 1,525 steps: shut before they reached the knot, the forceps never caught it, so nothing was lifted, and none of the hook's three tries cut |
| `hover` | the demo, but the instruments are held over their trays at the end, never let go | 0.75 in 1,127 steps: everything done and the stitch on the gauze, but neither instrument counted as back in its tray |
| `drop` | as the demo, but once pulled out the stitch is let go over the pad, and the forceps go on to the gauze with nothing in their jaws | 0.75 in 1,177 steps: cut flush and pulled out, but let go over the pad, the stitch fell onto the skin and stayed there; the forceps reached the gauze empty |
| `check_score.py` | the hook's way under the stitch on hand-built paths (slid in level from beyond the leg or from beside the strand: under it; brought down through the loop at any speed, or down inside the strand's 3 mm band and then sideways: not under it), the cut in the cutter's frame, upright and leaning 50 degrees back toward -y as the demo holds it (slid in across the leg and pressed onto the notch's edge: cut; stopped in the notch, 1.5 mm short and nudged, upright and 1 to 2 mm off the leg across the blade's plane, leaning with the hook 1 mm toward -y so the blade's plane meets the leg under the notch, the cutter turned 90 degrees, slid in from the spine's side, or the notch above the leg: no cut), the instruments' return (standing in the rest with both hands away: in the tray; still gripped, 3 cm up, or on the gauze's corner over the tray: not), the free strand on the gauze (at rest with both ends on it: yes; still sliding, on the pad's block, half over the edge or still in the jaws 3 cm up: no), and the scoring table on hand-built records | every case as expected |

## Scoring

Cumulative:

| Score | When |
|---|---|
| 0.25 | the knot caught between the forceps' jaws as they close on it (the point between their tips within 3 mm of it, the jaws stopped short of shut by the knot) and lifted 2 mm or more |
| 0.5 | also the far leg (the one away from the knot) cut with no more than 2 mm of it left standing above the skin: the cutter's hook slid in level under the lifted stitch, from beside it, never down through it, the leg entering the hook's notch through its mouth, touching the blade's steel, and pressed onto its sharp inner edge, read in the cutter's own frame (the cut's height is the notch floor's lowest over the skin while the leg was in it). Both legs cut: 0.5 at most (the buried part stays in the skin) |
| 0.75 | also the stitch pulled out of the skin along its path, which only the jaws can do: its track turned at least 95 % of the way (the knot 12 mm up and 3 mm toward the wound), the wound's gap under 3 mm throughout |
| 1.0 | also the stitch lying on the gauze (its knot and its far end inside the gauze's outline, within 6 mm of the table, at rest), both instruments let go (every fingertip 3 cm from points along them) and resting in their trays (standing in the rest or lying in the tray: the lowest point within 1.5 cm of the tray, the middle inside its outline and off the gauze), every fingertip of both hands 4 cm outside the pad block's outline, all held 2 s |
| 0 | at any moment: the knot pulled 5 mm up before the cut (torn through the skin), or any of the cutter's lowest corners (its blunt foot's bottom, the blade's end) more than 2 mm into the silicone anywhere on the skin |

On a real trial: before the trial, the strand is ink-marked where it leaves the skin on both legs. A top camera over a mm scale, both wrist cameras and a side camera at skin level film the trial. Afterwards the cut is measured on the drawn-out stitch with a loupe and a mm scale, from the far leg's ink mark to its cut end, and the pad is checked for any cut in the silicone.

## Building it for a real trial

- **Pad:** a silicone suture practice pad (a Limbs & Things skin pad or the same kind), about 12 x 8 cm, on a 2.2 cm block, taped to the table; a fresh stitch tied by hand for each trial across the incision's middle, to the lab's standard: 3-0 monofilament nylon, simple interrupted, a square knot of 3 throws (two flat throws and a third to lock), both tails cut to 5 mm, 4 mm bites from the incision's edge, snug to the skin, the knot on the +y side, ink-marked where it leaves the skin. Replace the pad when its incision tears.
- **Instruments:** 12 cm non-toothed dissecting forceps standing tips-down in a slotted foam rest on a steel tray; a single-use stitch cutter standing hook-down in a foam rest on a second tray.
- **Gauze:** a 7.5 cm gauze square on the table, its corner over the forceps tray's.
- **Safety:** the cutter's hook is sharp; keep hands out of the station while the robot holds it.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| The incision's opening | the pad's knot-side half on a 3000 N/m spring across the incision, up to 4 mm | **Not measured.** Pull the knot away from the wound with a small force gauge and read the gap on the top camera |
| The stitch's lift | the knot on a 200 N/m spring normal to the skin | **Not measured.** Lift the knot with a force gauge on the forceps and read the lift on the side camera |
| The tear limit | the knot pulled 5 mm up before the cut | **Not measured.** Lift the knot of a spare stitch with the forceps until the skin tears, read on the side camera |
| The drag on the cut stitch | 0.3 N along its path (assumed; 0.1 to 1 N) | **Not measured.** Draw a cut stitch out with a force gauge on the forceps |
| The dig limit | the cutter's lowest corner 2 mm into the silicone | Check the pad for a cut after the trial |
| The forceps' preload | the arms open with 1.6 N at the hand's hold | **Not measured.** A force gauge across the arms where the gripper holds them |
| The stitch's thickness | drawn 0.6 mm; the knot 3 x 2.2 x 2.5 mm | 3-0 nylon is about 0.25 mm |

## What the simulator leaves out

The stitch is not a thread. It is a rigid strand, drawn 0.6 mm thick (3-0 nylon is about 0.25 mm): a soft-body thread that thin would need far smaller physics steps. What is physical: the knot and the loop have colliders, and only the forceps' jaws move them, by friction. The knot rides a sprung lift joint normal to the skin (200 N/m), on the pad's knot-side half, which itself rides a stiff spring across the incision (3000 N/m), so a lift tugs the skin and a pull away from the wound opens the incision. Until the far leg is cut, the stitch's path through the skin is locked. Once it is cut, the strand slides out along that path (a 26 mm-radius track that carries the knot 13 mm up and 3.5 mm toward the wound) against a set drag of 0.3 N (assumed; 0.1 to 1 N for nylon drawn from a closed track), and it moves only as far as the jaws pull it: jaws that slip or open leave it where it is. Drawn out to the end of its path, the strand on the track gives way to a free rigid strand of the same knot and a 20 mm tail (1 g, heavier than real nylon so the solver holds it), put 3.5 mm above the drawn knot between the same jaws; the drawn strand keeps its colliders 0.3 s more, then goes, and the jaws settle onto the free knot. The forceps carry it to the gauze and let it go, and it falls and lies where it lands, which the scorer reads. What is a rule: the cut. The cutter's lowest 6 mm has no collider: a cut is the leg entering the hook's notch through its mouth and pressed onto its sharp inner edge, read in the cutter's own frame, after the hook reached under the lifted stitch from beside it, never down through it; the cutter's depth is read at its lowest corners. The swap from the strand on the track to the free strand is also code. No cutting force is simulated. The forceps' arms are a real sprung hinge closed by the gripper's squeeze; the fingers' pinch on an arm resists turning over a 4 mm patch, as a finger pad does on a 6 mm arm.

## Assets

Every shape in the scene (pad, stitch, forceps, cutter, rests, trays, gauze) is built in code from boxes in `task.py`; no third-party meshes, textures or fonts are used. The YAM arms and the station come from `yam-isaac-lab-arena`.

## Licence

MIT
