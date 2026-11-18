# End-of-day chairs up: lift a preschool chair, turn it over and set it seat down, legs up, on the table

A preschool chair stands on the floor. Both hands take it by the side edges of its seat, lift it, turn it over through half a turn about the line between the hands, and set it down on a low table seat down with its legs up, its back hanging over the table's near edge. Then they let go and move away.

In the demo the two hands pinch the seat plate at its side edges, 5.1 cm behind the seat's middle (through the chair's centre of mass), with the fingers pointing in. They lift the chair 25 cm, turn it about the line between the hands while that line moves back and up, slide it over the table at height, and lower it until the lower fingertips are 12 mm over the top, the chair's seat about 4 cm over it. They open the fingers, so the chair drops that 4 cm onto the top, and the hands move 11 cm out to the sides.

**Why it matters (Societal relatability).** At the end of the school day the chairs go up on the tables so the floor can be swept and mopped. Almost everyone has done it, or watched it done, in a school classroom; a preschool does it with child-sized chairs and low tables. The task is the physical act inside that routine: turning a light, smooth chair over without letting it swing or slip, and landing it flat.

**Ability tested:** a two-hand lift and half-turn of a light, slippery, forward-heavy object, held level through the turn, ending in a flat landing on a table with its edge in a stated place.

**Nearest tasks.** Picking and placing an object keeps it the same way up; here the whole difficulty is the turn, and the landing is on a table's edge with the back hanging over.

**Arms.** Both. Each hand needs to hold the chair on its own side of the line through the centre of mass; with one hand the chair swings out of the grip (`one hand lifts it` in the table below scores 0).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `sweep2.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
  --instruction "Put the child's chair up on the table: take the chair standing on the floor by the two side edges of its seat, lift it, turn it over and set it down seat down with its legs up on the tabletop, the seat's back edge on the table's near edge line or up to 3 cm past it, so that the chair's back hangs over the edge, without a leg touching the top while you carry it and without dropping it. Then let go and move both hands away from the chair."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full          # full score, with a video
python demokit.py task.py OUT --no-record --mode drop,none   # the checks below
python check_score.py                              # the scorer on hand-built trials, with no simulator
```

The demo reads the chair's pose and the arms' joint states to place the hands and set the grip; it reads no contact force and nothing from the scorer. Nothing in the scene is random: `object_jitter=0`.

## The scene and what is simplified

The scene is the robot bench, whose top stands for the classroom floor, with a table on it and the chair. Positions are (x forward from the arm bases, y left) in cm.

- **Chair:** Flash Furniture Whitney stackable preschool chair, model YU-YCX-003: one moulded polypropylene shell on four splayed plastic legs; 12.5 in wide, 14 in deep, 20 in high; seat 10 x 10 in at 10.75 in; back 11 in wide rising from the seat's rear edge. In the scene the seat and back are 12 mm plates and the legs are 25 mm tubes, with no moulded curves. It stands with its seat centre at (10, 0), facing the arms. Not confirmed by the maker: the mass (1.4 kg used; the listings give "weight 6" with no unit and a 10 lb shipping weight for the pair), the shell thickness and the leg section.
- **Table:** Community Playthings Woodcrest rectangular table C241 as a 12 in high table (the maker's 12 in leg option): solid maple top 30 x 22 in, 1 1/8 in thick, no apron. Long side across the arms, near edge at x = 29.8, top face 30.5 cm above the floor. The legs are 45 mm square (not confirmed by the maker). The table is fixed.
- **Why a 12 in table:** it is the lowest height the maker offers (12 to 20 in), and the arms reach it. The top is lower than a classroom's usual table height; the turn and the landing are the same at any height.

Friction is set so that the shell can be held by two pinching hands at 35 N per finger. The seat plate is held at about 12 mm between the pads; in the demo the chair turns in the grip by at most 4.7 degrees. The chair tilts off the planned turn axis by at most 1.4 degrees.

Touch with the top is read from the poses of the chair's feet, back corners and seat; no contact force is read.

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. `check_score.py` feeds the real `Trial` hand-built states for the honest placing and each lazy route.

| What happened | Score |
|---|---|
| Both hands hold the chair (pads on both faces of the seat plate) with its lowest point 8 cm or more off the floor | 0.25 |
| Also both hands hold it with the seat's face within 30 degrees of straight down | 0.5 |
| Also the seat down on the top within 5 degrees (each corner over the top within 12 mm of its face), the chair let go and still (under 1 cm/s) for 1 s | 0.75 |
| Also the seat's back edge on the table's near edge line or up to 3 cm past it, every fingertip 3 cm or more from the chair (seat edges, back, legs), held 2 s | 1.0 |
| Let go with any part of it more than 8 cm over the floor or the top it lands on, or falling faster than 1.2 m/s with no part of it on either; a leg's foot or the back's top corners on the tabletop's face while the chair is held or moving; the seat sliding more than 3 cm along the top after first touching it | 0 |

A real trial reads the same with a ruler and a photo from above: the seat's rear edge against the table's near edge, the seat's tilt, and the fingers' distance. A drop shows on video as a fall from more than a hand's width.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` | The whole task (the film) | 1.0 at step 432 (21.6 s) |
| `drop` | The same, but the fingers open at the turn's height, so the chair falls about 17 cm onto the top | 0.0: dropped (the seat 17 cm over the top at let-go) |
| `none` | The arms wait | 0.0 |

`check_score.py` also runs, on hand-built states: the chair left standing on the floor (0), knocked onto its side on the floor (0), lifted by one hand only (0), lifted by both hands and set back down upright (0.25), lifted and put upright on the top (0.25), turned over and set down upside down on the floor beside the table (0.5), put down on its side on the top (0.25), the seat tilted 20 or 8 degrees on the top (0.5), 3 degrees (1.0), a hand left 2 cm off the seat edge (0.75) or still pinching it (0.5), the end held only 1 s (0.75), the back edge 25 mm past the edge (1.0), 45 mm past (0.75), 25 mm inside it (0), let go 6 cm over the top (1.0) or 12 cm over it (0), the seat dragged 9 cm along the top (0), the chair pushed off the table (0), and a carried chair with its feet on the top (0).

## Runtime and seeds

The full demo takes 21.6 s of simulated time (432 steps), about 1 minute of wall time on one RTX 3090 Ti at 640 x 360, start-up apart. There is no randomness.

## A real trial

Use a Flash Furniture YU-YCX-003 chair and a Community Playthings C241 table, 12 in high, or any preschool chair of about that size with a flat seat plate and a table of about that height. Put the chair on the floor 10 cm from the table's line; read the seat's rear edge against the table's near edge with a ruler, the seat's tilt with a level or a photo, and the hands' clearance by eye against a 3 cm mark.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Chair sizes: dealer listings of the Flash Furniture Whitney stackable preschool chair, 2-pack YU-YCX-003 (webstaurantstore.com item 354CX003GRN; quill.com item 2YUYCX003BLUE) and Flash Furniture's own product page 2-YU-YCX-003-RED-GG. Flash Furniture is a trademark of its owner, named for identification only.
- Table: Community Playthings, Rectangular Woodcrest Table C241, 12 in high (communityplaythings.com; dimension drawing and product guide). Community Playthings is a trademark of its owner, named for identification only.

## Licence

MIT
