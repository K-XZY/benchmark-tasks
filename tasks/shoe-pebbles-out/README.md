# Get the stones out of a running shoe: tip it opening-down with the toe up, shake all 8 out, set it back down

A men's US 9 road running shoe stands on the table with eight gravel pebbles inside: four in the toe box under the closed upper, two at the midfoot and two at the heel. The right gripper pinches the shoe's heel counter from the side, fingers leaned 10 degrees off level (one finger inside the collar, one behind the heel), lifts it, turns it toe-up past vertical so the opening faces down, shakes it along its length, turns it until the opening faces straight down for the last stones, shakes again, and sets it back down on its sole.

## Run

From this folder, with the harness's Python on a GPU host:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,collar_up,heel_up,on_side,drop,drop_low,drop_25mm,toe_first,wide_toe_first,relift_3cm,collar_hook_drop
python check_score.py                              # the scorer on hand-built states, no simulator
```

The harness's instruction (Dream-1 run): `--instruction "There are some small stones in this shoe. Get them all out, then put the shoe back down on the table, sole down."`

## Scoring, and the routes that score less

A stone is in while its centre lies within the shoe's outline above the sole (over the collar opening, up to 1 cm above the edge, so a stone on the lip is still in). The count is read from the current state at the end; only the void latches.

| Score | When |
|---|---|
| 0.25 | at least 2 of the 8 stones out |
| 0.5 | at least 4 out |
| 0.75 | at least 6 out, or all 8 out with the shoe not set down |
| 1.0 | all 8 out, the shoe resting on its sole on the table within 20 degrees of level, both grippers open and every part of them (fingers, pads, palm, housing, wrist flange) at least 3 cm from the shoe, held for the last 2 s |
| 0 | once a gripper has held it, it is free of both grippers with any point of its sole edge (heel, toe, either side) more than 1.5 cm above the table (let go in the air, or tipped over after release; the highest such point at the last held step counts, held = both fingertips of one gripper within 3 mm of the shoe and moving with it, each slipping less than 0.1 m/s over the shoe's surface: a finger that merely sits in the collar while the shoe slides off does not hold it), or it moves faster than 0.54 m/s (a 1.5 cm fall) while not so held (dropped or thrown), or ends off the table. A shoe no gripper ever held is voided only by the speed rule or by leaving the table |

Heights in the score column are run results: the sole's highest edge point above the table at the last step the hand held the shoe.

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | heel counter pinched, toe turned up 135 degrees and shaken, then 162 degrees and shaken along and across the shoe, rolled level, set down, hand clear | 1.0 (682 steps; the sole is under 1 mm above the table at release) |
| none | nothing | 0.0 |
| collar_up | lifted and shaken upright, opening up | 0.0 (0 out; the shoe left the pinch 17.9 cm up, void) |
| heel_up | opening turned down with the toe kept low (heel up), shaken | 0.0 (0 out; the shoe came out of the pinch 26.0 cm up, void) |
| on_side | emptied as in full, then rolled toward its side | 0.0 (the roll is out of the wrist's reach: the shoe left the pinch 19.2 cm up, void) |
| drop | emptied as in full (8 out), then the hand opened 10 cm above the set-down point | 0.0 (the sole was 9.4 cm up at the last held step, void) |
| drop_low | the same, hand opened 3 cm above the set-down point | 0.0 (the sole was 3.7 cm up at the last held step, void) |
| drop_25mm | the same, hand opened 2.5 cm above the set-down point | 0.0 (the sole was 3.2 cm up at the last held step, void) |
| toe_first | emptied as in full and rolled level, then lowered toe first until the toe touches, the heel about 4 cm up, and let go | 0.0 (heel 4.3 cm up at the last held step, void) |
| wide_toe_first | emptied and set down as in full (all 8 out), then the heel gripped across its two sides (opening 7.5 cm), lifted 7 cm, the toe pitched down 8 degrees (heel about 5 cm up) and let go | 0.0 (8 out; heel 5.4 cm up at the last held step, void) |
| relift_3cm | the full run, then pinched again, lifted 3 cm and let go | 0.0 (sole 3.1 cm up at the last held step, void) |
| collar_hook_drop | emptied and set down as in full, then both fingertips go into the collar opening, spread against its inner walls (no pinch from outside), lift 5 cm and ease off so the shoe slides off | 0.0 (8 out; the sole was 4.4 cm up at the last held step, void) |
| Dream-1 | the model, given the instruction above, 320 x 176 cameras, 1200 steps, seed 1 | 0.0 (the shoe never moved from the table; all 8 stones stayed in) |

All 8 out with the shoe still held or not upright scores 0.75 (`check_score.py`). The task is one-handed: the second arm is not needed, as when a person tips a stone out of a shoe with one hand.

## Real items

| Item | Maker, model, link | Size in the scene (source) |
|---|---|---|
| Running shoe | Nike Pegasus 41 men's road running shoe, US 9 (nike.com/t/pegasus-41-mens-road-running-shoes-LMhfRGdO); list price about $140 (nike.com), used pairs cheaper | 297 g (Nike); stack 37 mm heel, 27 mm forefoot, drop 10 mm (Fleet Feet listing); outer length 290 mm (estimate: Nike US 9 foot length 262 mm plus toe room); collar opening about 100 x 67 mm (estimate; 103 x 67 mm in code); heel counter 120 mm tall, 12 mm thick (estimate; the real one is foam-padded and softer); walls 4.5 mm (the real mesh upper is thinner, see Simulation) |
| Stones | Washed pea gravel, 4 to 8 mm (a few dollars a bag; Wentworth fine pebble class, zenodo.org/record/1431467), quartz 2.65 g/cm3 (webmineral.com/data/Quartz.shtml) | 8 stones, 4.1 to 8.0 mm long, 0.04 to 0.28 g each |
| Gripper | the YAM arm's parallel gripper (the program's robot, i2rt.com) | 40 N a finger in the demo (the YAM's figure) |

## Simulation

- The shoe is one rigid free body: 141 convex pieces built in `task.py` (sole, side walls 4.5 mm thick, a closed vamp over the toe box, a solid rigid heel counter and toe cap (the real counter is foam-padded), the counter 12 mm thick and 120 mm tall at the back, the hand's only handle). The real Pegasus 41 has a thin flexible mesh upper (about 1 to 2 mm, estimate) and a foam-padded collar; here the 4.5 mm rigid walls and the solid 12 mm counter stand in for them, because 4 mm stones tunnel through thinner walls at 240 Hz and the shoe is one rigid body, so the cavity is open only at the collar. Physics at 240 Hz on the GPU, with CCD on each body and on the scene, contact offsets of 2 mm (shoe) and 1.5 mm (stones). Before a real trial, press the real collar and counter at 40 N to measure how far the foam gives, and turn the shoe opening-down in the pinch to check it does not twist.
- The stones are irregular convex hulls 4.1 to 8.0 mm long, mass from hull volume at 2.65 g/cm3 (0.04 to 0.28 g).
- The demo pinches the heel counter with about 40 N a finger (the YAM's figure is about 40 N) and lifts 0.3 kg; under the toe torque (about 0.35 N m with the toe up) the shoe turns in the pinch, which the turn angles absorb, and after the shake it hangs rolled about 40 degrees about its length, so the hand rolls it level before the set-down; it runs 682 of the 1200 steps (34 s). It reads only the arm's own joints; it never reads the stones or the shoe's pose.

## Real trial

Parts: one lace-up road running shoe, US 9 (the Nike Pegasus 41 itself; a used pair does), and about 20 pea-gravel stones sieved between 4 mm and 8 mm (a bag of washed pea gravel and two kitchen sieves, a few dollars; estimate). Pick the 8 whose lengths run from 4 to 8 mm.

Reset (about 2 minutes): stand the shoe on the table, laces tied, heel at the mark; place the 8 stones on the insole through the collar with tweezers: 4 in the toe box under the vamp (22 to 26 cm from the heel), 2 at the midfoot (16 to 17 cm), 2 at the heel (5 to 7 cm), each on a pencil dot drawn on the insole.

Before the trial, close the gripper pads on a hand-held force gauge until the gripper's motor effort reads 30 to 40 N a finger, then pinch the heel counter at that effort and confirm the pinch lifts the shoe without slipping.

Score: lift the shoe and count the stones left on the table; tip the shoe into a hand to confirm none is inside. A phone video of the whole trial (30 fps or more, two cameras at table height, one side-on and one facing the toe, a 1.5 cm tape mark beside the shoe: a release with any edge of the sole (heel, toe or either side) above the mark voids; the grippers at least 3 cm from the shoe at the end, and any drop) and a photo of the table and of the shoe's resting pose, level within 20 degrees by eye against a card wedge, is the whole record. The same rule runs in `score()`: a stone is out once it is clear of the shoe's outline.

Protection: a table mat so stones do not roll away, and a rubber pad on each fingertip so the pinch does not mark the heel.

## Sources

- Nike Pegasus 41 men's road running shoe (nike.com/t/pegasus-41-mens-road-running-shoes-LMhfRGdO): 297 g, 10 mm drop. Fleet Feet Pegasus 41 listing: 37/27 mm stack.
- Nike men's footwear size chart (nike.com/size-fit/mens-footwear): US 9 = foot length 10 5/16 in (262 mm); the 290 mm outer length adds toe room (estimate). Collar opening 100 x 67 mm, widths along the shoe, heel counter height and upper heights are estimates.
- Wentworth (1922) grade scale: fine pebble 4 to 8 mm (zenodo.org/record/1431467). Quartz density 2.65 g/cm3 (webmineral.com/data/Quartz.shtml).
- World Footwear Yearbook 2025 (APICCAPS), via La Conceria, 'Some buy, some produce: the map of shoe consumption in 2024' (laconceria.it/en/footwear/some-buy-some-produce-the-map-of-shoe-consumption-in-2024): the US bought about 2.1 billion pairs in 2024, about 10% of the world total.
- Outdoor Industry Association, Running Report 2026 (outdoorindustry.org/2026-running-report): 52.3 million US road runners and 17.2 million trail runners in 2025; the core segment runs 50 or more times a year.
- J. Clarke, How to keep stones out of your shoes on the trail, Advnture, 3 Jun 2023 (advnture.com/how-to/keep-stones-out-of-shoes).
- Friction (0.5 stones, 0.6 shoe) and zero restitution are estimates.
- All meshes are built in `task.py`; no third-party meshes are used.

## Licence

MIT
