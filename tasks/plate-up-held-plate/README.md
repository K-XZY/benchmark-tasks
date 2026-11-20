# Plate up a canteen meal on a hand-held dinner plate: steak, hot dog, two wings and a bread roll, all inside the rim

A serving counter holds a short stack of 26 cm melamine dinner plates, two steam-table pans of food (a seared rump steak and a hot dog in its bun in one, two whole chicken wings and a crusty bread roll in the other) and, across the counter, the pick-up zone on the pass: a shelf pad outlined by a yellow tape line, where the guest collects the plate. One hand takes the top plate off the stack by its rim and holds it up in the air for the whole loading; the other hand lifts the five portions out of the pans and lays each on the held plate so that nothing sticks out past its edge; then the served plate is delivered into the pick-up zone and let go.

In the demo the right hand first presses the top plate's far rim down with about 15 N, so the plate tips 5.5 degrees on the plate below and its near rim rises about 1.5 cm. The left hand slides level jaws round the raised near rim, 11 cm from the plate's centre, and pinches it with 40 N (the YAM's figure), one pad under the rim and one on top. The right hand lets go of the far rim, and the left lifts the plate 6.5 cm straight up off the stack and brings it toward the pans. From then on the left wrist reads the plate's tilt and turns about the pinch to bring it level, before the loading and after each item. The right hand takes the steak by its narrow end (8.3 cm across there; the jaws open 9.4 cm, too little for its 10.3 cm width) and lays it along the plate on the side toward the pans, then the hot dog beside it, then the bread roll on the wide far end of the steak's top and one wing on its narrow near end, and last the other wing on the plate floor past the hot dog; each wing is taken by its drumette. Each item is lifted to 24 cm and carried above the held rim before it goes down, and each is let go with the jaws opened only a few millimetres wider than the item, so a finger does not sweep the portion beside it. The left hand then raises the loaded plate 8 cm (its housing hangs below the jaws and would drag across the stack) and carries it over the pick-up zone in 5 cm steps, levelling it after each. The palm and housing reach 5 cm below the jaws, so a level pinch cannot lay the plate flat: the wrist pitches the fingers 10 degrees down, the plate's far edge goes down onto the pad, the jaws open to their approach gap while the hand drops by its sag under the load (or it would spring up and flick the rim), and the hand sinks while pitching to 25 degrees, so the near rim comes down on the lower finger. It then draws its fingers 10 cm clear of the rim before lifting away.

**Why it matters.** Impact on jobs. On a staff-served line (school and hospital cafeterias, university dining, carveries) the server takes a plate, holds it by its edge and fills it from the pans. O*NET lists "Apportion and serve food to facility residents, employees, or patrons" for Cooks, Institution and Cafeteria (35-2012; 435,640 US workers, 137,290 in schools; BLS OEWS May 2023) and "Select food items from serving or storage areas and place them in dishes, on serving trays, or in take-out bags" for Fast Food and Counter Workers (35-3023, titles include Cafeteria Server; 88,840 in schools and 141,760 in contract food service). A food-safety rule for serving staff: "HOLD plates by the bottom or at the very edge" (Alberta Health Services).

**Ability tested:** packing five rigid shapes onto a free, round, wall-less plate held in the air by its rim: the two long portions only fit in bands across the plate, the others must go on top of them or in the space left, each must be laid without nudging those already there off the open rim, and the holding hand must keep the plate level as the load grows. Neither hand can do the other's job: the plate must stay held off the counter while the other hand serves.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3000 \
  --instruction "Take the top plate off the stack and, holding it up the whole time, serve the steak, the hot dog, both chicken wings and the bread roll onto it from the pans, with nothing sticking out past the plate's edge. Then deliver the served plate to the pick-up zone on the pass and let go."
```

## Scripted demo

The demo stays inside the YAM's figures: the plate pinch is 40 N a finger, the steak grip 40 N and other food grips 22 N, the press on the far rim about 15 N, the loaded plate 0.74 kg (payload 1.5 kg).

From this folder:

```bash
python demokit.py task.py OUT --mode full                    # full score, with a video
python demokit.py task.py OUT --no-record --mode none,stack_prop,set_down,idle_server   # the check runs below
python demokit.py task.py OUT --no-record --mode roll_over_edge,dropped_wing,no_handoff
python demokit.py task.py OUT --no-record --mode drop_on_mat
```

The demo reads the plate's pose to find its rim, to level it and to aim each item at its place on the plate, and each item's pose in its pan to take it. It moves nothing except through the grippers.

## The scene and what is simplified

Every part is at true size. Positions are in cm from the midpoint between the arm bases (x forward, y left), heights above the counter, which is the station table.

| Part | In the scene | Source |
|---|---|---|
| Plates | eight Carlisle Dallas Ware 4350002 melamine dinner plates, 26.0 cm across, 2.15 cm tall, 0.38 kg each, stacked at (30, +10), nesting 1.2 cm apart; the seven lower ones one fixed body, the top one free | Boelter listing; rim section and nesting pitch estimated from photos and the case size (48 plates, four stacks of 12 in a 6.38 in box) |
| Pans | two Vollrath 30222 half-size pans, 32.5 x 26.5 x 6.5 cm, fixed as if set in the line's wells, centred (38, -30) and (38, -57) | Vollrath |
| Steak | a seared rump steak with a fat edge along one side, 16 cm long, 8.3 cm wide at one end widening to 10.3 cm, 2.2 cm thick, 270 g, in pan A at (38, -24.5), long axis along x, narrow end toward -x | a canteen rump or sirloin portion (15 to 18 by 10 to 12 cm, about 2 cm thick); mass a cooked portion (estimate) |
| Hot dog in bun | 16 x 6 x 5 cm, 100 g, in pan A at (38, -36) | Display Fake Foods SKU 4540 or a real one (size an estimate from photos) |
| Wings | two whole chicken wings (drumette, flat and tip joined, slightly bent), 9.5 x 4.3 x 3 cm, 60 g each, in pan B at (29, -53) and (29, -64), drumette toward -x | a cooked whole wing (mass an estimate) |
| Bread roll | a round crusty bread roll, 7.6 cm across, 5 cm tall, 45 g, golden-brown domed crust split across the top, floury base, in pan B at (41, -57) | a crusty dinner roll (mass an estimate) |
| Pick-up zone | a 35 x 35 x 1 cm stainless shelf pad at (38, +44), just left of the stack, outlined on the counter by a 1.5 cm yellow tape line round it; it takes one 26 cm plate with 4.5 cm to spare each way | the pass where a served plate is collected; real trial: any flat 1 cm board of that size and floor-marking tape |

Departures, each stated once:
- The food is rigid simple solids generated at real portion sizes, each with one convex-hull collider (the hot dog's takes in the frank on top of the bun; a wing's takes in the small notch where it bends). The real trial uses cooked food from a canteen or grocery (about 15 USD a set) or rigid display replicas of the same sizes. Friction is estimated: food on melamine 0.4, food on food 0.5, plate on plate and on the pad 0.8.
- The seven lower plates are one fixed body (2.7 kg of plates that do not shift under a hand).
- Physics: rigid bodies at 240 Hz on the GPU.

## Scoring

Mostly positions: four tiers, one per key substep (plate off the stack, all five served onto it, plate delivered into the zone and let go, delivery kept), and one failure that voids the run. Terms:
- **On the plate:** the item's collider lies wholly inside the plate's 13.0 cm outer radius, in the plate's own frame (real: a photo from straight above), its centre above the plate's floor, it has not fallen to the counter or the stack since leaving its pan, it is moving slower than 0.2 m/s, and no gripper point (fingers root to tip, palm and housing, the wrist link) is within 8 mm of its collider's surface.
- **Rests on:** a plate point (72 round its edge, 72 round its foot) within 3 mm of the counter or the zone's pad, or a support point within 3 mm of the plate's real surface (its whole section, foot to edge). The support points lie about 5 mm apart on each pan's wall tops and on the stack: its top plate's well, rim and edge (144 directions x 6 radii) and its outer side (144 directions, every 5 mm up). A food item under the plate within 3 mm of it also counts. `check_score.py` lowers a plate onto each of these (the stack centred, its foot side over the stack's edge every 10 degrees round, a pan wall from three sides, the counter, the zone, a plate on edge against the stack) and checks the test fires within 3.5 mm of the touch.
- **Held clear:** the plate rests on nothing, a gripper point is within 4 mm of its surface (a gripper holds it), and its lowest point is at least 5 cm above the counter (real: the front camera at counter height, against one mark 5 cm above the counter).
- **Off the stack:** held clear, or touching no part of the stack with its centre at least 26 cm from the stack's centre.
- **In the zone:** the plate's centre within 4.5 cm of the zone's centre in x and y (so the whole 26 cm plate lies inside the 35 cm line), its lowest point within 3 mm of the pad, and its tilt at most 6 degrees.
- **Dropped:** an item low over the counter outside the pans, or on the stack, and not on the plate. It never counts again.

| Score | Condition, read from the state now |
|---|---|
| 0.25 | the plate is off the stack |
| 0.5 | the plate is off the stack, with all five items on it |
| 0.75 | the plate is in the zone with all five items on it, both grippers at least 8 mm clear of the plate and the food, and the plate and every item slower than 3 cm/s |
| 1.0 | the 0.75 state, unbroken for the last 2 s (40 steps at 20 Hz) |

Void, score 0: food on the plate while the plate rests on anything (the stack, the counter, a pan's rim, the zone or food under it) before all five items have been on it together while it was held clear; or the plate let fall at any time (any point of its edge or foot going down faster than 0.45 m/s, a free fall from 1 cm, so a plate that tips off a finger counts too). So loading the plate on the stack or propped on its rim, or setting it down to free a hand, scores 0; one arm alone cannot score above 0.25. The 0.2 m/s, 8 mm, 3 cm/s and 0.45 m/s tests are simulation only; real: the item seen still on the plate, a visible gap between fingers and plate or food, nothing moving for 2 s, and the plate never seen released above the pad or sliding off a finger in the front video. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## Building it for a real trial

Real trial (about US$105 with cooked food): the plates (about $5 each), two half-size pans (about $30), the food (cooked steak, hot dog in bun, two wings and a roll from a canteen or grocery, about $15 a set; or rigid display replicas chosen to the sizes above, such as the Display Fake Foods hot dog on bun) and the pick-up zone (a 35 x 35 cm flat board about 1 cm thick, about $15, and a roll of yellow floor-marking tape, about $5). Weigh and measure each item before the first trial and record it, and measure the loaded plate's pinch (how far it sags in the fingers at 0.74 kg) on the real hand. Nothing in the kit can shatter: the plates are melamine, so a dropped plate or portion is no hazard to people or the robot. Reset, about 2 minutes: restack the plates, return each item to its tape mark in its pan, send the arms to their ready pose. Film one video from straight above (the on-plate test) and one from the front at counter height (held clear, against one mark 5 cm above the counter, and the drop check).

## Results

Run on GPU physics at the README's commands, on lab PCs with an RTX 4070 Ti SUPER or RTX 3090 Ti; the check runs record no video.

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | the whole task, 640 x 360 | 1.0, at step 2554 (127.7 s) |
| `none` | nothing done | 0.0 |
| `stack_prop` | the plate lifted and held, then lowered until its foot side rests on the stack's rim (tilted 8 degrees, still in the hand), the steak and hot dog laid on it there, then lifted clear | 0.0 (void: food on the plate while it rests on the stack) |
| `set_down` | the plate lifted, put back on the stack, the steak and hot dog laid on it there, then lifted and delivered to the zone | 0.0 (void: food on the plate while it rests on the stack) |
| `idle_server` | the plate lifted and held, the other arm never serves | 0.25 |
| `roll_over_edge` | the roll set on the rim, half past the plate's edge, the plate held to the end | 0.25 |
| `dropped_wing` | one wing dropped on the counter, picked up again and served, the plate held to the end | 0.25 (dropped food never counts) |
| `no_handoff` | all five served, the plate held to the end, never delivered | 0.5 |
| `drop_on_mat` | all five served held clear, then the plate let go 4 cm above the zone | 0.0 (void: all five were on the plate, so the void is the drop rule) |
| Dream-1 | the Dream model through the harness's walkthrough command, one valid session, seed 1, 20 rows per request, 3,000 steps, lab PC with an RTX 3090 Ti, on the final code | 0: the plate never left the stack; the right hand reached into the pans. Film: attached to the PR |

The `full` run on the final code scored 1.0 (step 2554); earlier runs of the same motion with the pass mat in place of the zone scored 1.0 at step 2574 twice. The three check runs that serve all five (`dropped_wing`, `no_handoff`, `drop_on_mat`) kept all five on the held plate. The demo is still not certain: the leaning fingers carry each item 1 to 3 cm past where they let it go, the lay targets allow for that, and the wing on the plate floor ends about 1.5 cm inside the rim. A run that loses an item before all five are on the held plate scores 0 once the plate is set down.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Workers: O*NET 35-2012.00 Cooks, Institution and Cafeteria (https://www.onetonline.org/link/summary/35-2012.00) and 35-3023.00 Fast Food and Counter Workers (https://www.onetonline.org/link/summary/35-3023.00). This includes information from O*NET OnLine by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence; O*NET is a trademark of USDOL/ETA. BLS OEWS May 2023, 35-2012 (https://www.bls.gov/oes/2023/may/oes352012.htm) and 35-3023 (https://www.bls.gov/oes/2023/may/oes353023.htm).
- Plate handling: Alberta Health Services, Serving food properly (https://www.ahs.ca/assets/wf/eph/wf-eh-serving-food-properly.pdf).
- Plate: Carlisle 4350002 Dallas Ware 10 1/4 in melamine dinner plate (https://shop.boelter.com/tabletop-serving/dinnerware/melamine-dinnerware/carlisle-melamine-dinnerware/dallas-ware/carlisle-4350002-dallas-ware-10-1-4-white-dinner-plate-melamine.html).
- Hot dog replica (optional): Display Fake Foods, Hot Dog On Bun (https://displayfakefoods.com/hot-dog-on-bun/).
- Pans: Vollrath 30222 Super Pan V, half size, 2 1/2 in deep.
- Carlisle, Vollrath and Display Fake Foods are named here for identification only; the names are their owners' trademarks.

## Licence

MIT
