# Slide a vegetable kebab off its skewer onto a plate with a fork: one hand pins the skewer, the other pushes

A kebab lies on a cooling rack with its point over a plate. Four pierced pieces (pepper, onion, courgette, mushroom) grip a 3 mm bamboo skewer. A push along the stick that moves them also drags a loose skewer, so one hand pins the skewer by its bare end while the other sets a dinner fork's tines astride the stick behind the last piece and drives the pieces off the point onto the plate. Recipes say it plainly: "Use a fork to slide the vegetables off of the skewers."

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,one_arm,hold_only,strip
```

The harness's instruction (Dream-1 run): `--instruction "Take the vegetables off the kebab onto the plate with the fork. Keep the skewer where it lies, with its point over the plate, and do not touch the vegetables with your grippers."`

`KB_SKM` (stick mass, default 0.010 kg), `KB_GRIP` (slide force per piece, default 1.0 N), `KB_PIN` (pinch distance from the blunt end, default 0.04 m) and `KB_YAW` (the fork's heading, default -35 degrees) are debug knobs.

**Result** (scripted, GPU physics, 20 Hz): the full mode scores 1.0 on RTX 3090 Ti and RTX 4070 Ti SUPER hosts (602 steps, 30.1 s, against a limit of 1,200; 0 blank frames at 640 x 360). Each route in the table below ran on GPU on the final scorer.

**Dream-1 run** (the harness's own policy, 1,200 steps, one trial): 0. The policy did not hold the skewer or take anything off the kebab in 1,200 steps (no tier reached). The film is `model_run.mp4` (left, right and top cameras side by side).

## Scoring, and the routes that score less

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | left comes straight down on the bare stick 40 mm from its blunt end and pinches it (40 N); right takes the fork by its handle, lays the tines across the top of the stick behind the mushroom (fork 55 degrees off its line), pushes 0.2 m along it; the four pieces drop onto the plate in turn; the right draws the fork away, the left lets go and lifts clear | 1.0 (602 steps, 30.1 s; the stick drifts 18 mm at most in the pinch) |
| hold_only | the left pinches the stick and holds it, nothing else | 0.25 (held 1 s) |
| hover | the left hand open, fingertips 8 mm over the bare stick, 4 s, nothing else | 0.0 (jaws not shut: not held) |
| one_arm | the right alone pushes with the fork, the stick loose on the rack | 0.0 (the stick is dragged 0.11 m along with the pieces, nothing comes off) |
| strip | no fork: the left pinches the stick and drags it 0.14 m back so the pieces catch on the rack bar and come off the point | 0.0 (all four pieces come off with no fork pushing them: FAULT) |
| none | nothing | 0.0 |

The score is read from the state at the end; only the failures latch (a gripper on a vegetable, a piece off the point without the fork pushing and the skewer held, a piece landing on the bench). The 0.25 tier reads the hold now, not a latched flag. `check_score.py` runs the tier table on hand-built records with no simulator.

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Skewer | 300 x 3 mm bamboo, axis 41.5 mm over the bench, point toward -y; 10 g in sim | 12 in x 3 mm; about 2 g | Roland 71516; mass: see "What is modelled" |
| Pepper | 6 x 30 x 30 mm, 6 g, red | | estimate |
| Onion | three layers, 18 x 25 x 30 mm, 12 g | | estimate |
| Courgette | round, 12 mm thick, 32 mm across, 9 g | | estimate |
| Mushroom | domed cap 35 mm across, 30 mm tall, with a stem stub, 15 g | | estimate |
| Run | 66 mm long, ending 40 mm from the point; 194 mm bare behind it; a 4 mm hole in each piece | | estimate |
| Slide force | 1.0 N per piece along the stick | | estimate, to measure |
| Rack | two 4 mm bars 160 mm long, 40 mm up, 60 mm apart, on legs and rails | | estimate |
| Plate | 260 mm across, floor 8 mm, rim wall 20 mm high, centre (0.38, -0.26) | 26 cm dinner plate | estimate |
| Fork | 190 mm, four tines 2.5 mm wide on 5 mm gaps, handle 10 x 4 mm, 70 g | | estimate |
| Cutlery board | 150 x 100 x 12 mm under the fork's head and tines | | estimate |

## Colliders

| Body | Colliders | Free or fixed |
|---|---|---|
| Rack | one box per bar, leg and rail | fixed |
| Plate | a cylinder for the floor, 28 box segments for the rim | fixed |
| Skewer | one 300 x 3 x 3 mm box | free, 10 g |
| Pepper, onion, courgette, mushroom | the outline as four convex hulls round a 4 mm hole, so nothing overlaps the stick at spawn | free, each on the stick by a sliding joint (below) |
| Fork | a box for the handle, a hull for the neck, a box for the head, a box per tine | free, 70 g |
| Fingers | the harness's YAM finger colliders | the robot |

## Robot figures the demo keeps within

The YAM's jaws open to 94 mm and grip about 40 N a finger; each arm pushes about 20 N and reaches about 60 cm. The left pinches the 3 mm stick at 40 N (a pinch in free air, jaws across x, fingers straight down); the right pinches the 10 x 4 mm fork handle at 40 N and pushes at 20 mm/s with a few newtons. Loads about 0.02 N m.

## Real trial

Cut fresh pieces for every trial to the sizes above, spear them on a 12 in bamboo skewer so the run's rear face is 70 mm from the point, and check each piece slides on about 1 N (a kitchen scale pulled along the stick). Lay the skewer across a cooling rack with its bars 40 mm up, the point 90 mm from the centre of a 26 cm plate, and a dinner fork on a board at the right. Keep the pieces at room temperature and use them within 5 minutes.

What the video and the top photo see, by the scorer's terms:
- HELD: the left hand's jaws shut on the bare stick (both pads within 4 mm of it, jaws 6 mm apart or less; a hand open over it is not holding), the stick staying within 30 mm of where it lay.
- PUSHED: each piece leaves the point with the fork behind the run, the stick held.
- ON THE PLATE: the piece resting inside the plate's rim after the run (top photo).
- Void: a gripper touching a vegetable; a piece coming off the point with no fork pushing it; a piece on the bench.

| Real-trial kit | Use | Price (about) |
|---|---|---|
| Bamboo skewers 12 in x 3 mm (100) | the stick | $5 |
| Red bell pepper, red onion, courgette, button mushrooms | the pieces | $8 |
| Wire cooling rack, 26 cm dinner plate, stainless dinner fork | scene | $15 |
| Phone and tripod | top photo and video | $0 to $15 |
| Total | | under $45 |

## Safety for a real trial

Raw vegetables only; wipe the fork and plate between trials. The skewer's point is sharp: keep faces clear of the plate while the arms work.

## What is modelled, and what is not

- Each piece is a rigid body of its real size and mass, held on the skewer by a joint made on the live stage: locked in every direction but the slide along the stick, where a drive caps the force at 1 N (an estimate). The joint is taken off when the piece's rear face passes the point, and the piece falls free. A real pierced piece is soft and grips by friction; the force is a measurement to take on the real kit.
- The skewer is 10 g in sim, not 2 g: a 2 g stick flicks at about 1 m/s when pinched (contact jitter) and the pieces slide off it. The 40 N pinch, held 1.5 s, lets the stick drift up to 20 mm under the push; the scorer allows 30 mm.
- The fork, plate, rack and board are free or fixed bodies with real colliders; the fork's tines are thin boxes, the tines' gap (5 mm) clears the 3 mm stick.
- The demo reads the skewer's and the fork's poses to aim the tines and to push along the stick; a mushroom stem is a stub.
- A piece rests on the plate, or not; juice, softness and crumbling are not modelled.

## Sources

- 'Use a fork to slide the vegetables off of the skewers. Serve as soon as possible.' (oven-roasted or grilled veggie kabobs, https://www.justapinch.com/recipes/side/vegetable/oven-roasted-veggie-kabobs-or-grilled.html?r=3)
- 'Tell your guests to remove the veggies from the skewers using a fork' (grilled vegetable kebab serving tip, https://eatsomethingvegan.com/simple-vegetable-skewers/)
- 'Hold the skewer horizontally over your plate ... use your fork to gently push or slide the food items off' (https://ova.it.com/dining-etiquette/how-do-you-remove-food-from-a-skewer)
- Bamboo skewer 12 in x 3 mm: Roland 71516, https://rolandfoods.com/product/71516-bamboo-skewers-12-x-3mm
- Piece sizes, masses, slide force, rack and plate sizes: estimates.

Roland is named for identification only; this task is not affiliated with it.

## Licence

MIT
