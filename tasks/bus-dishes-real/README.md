# Bus two flat 9 in porcelain plates into a deep tub: one hand tips each plate up by its rim so the other can grip it

A full-size porcelain dinner plate lying flat on a table gives a gripper nothing to hold until one hand presses its rim down and tips it on its foot ring (about 6 to 11 N for foot rings 11 to 14 cm across, calculation below; the demo presses 15 N and tips it 18 degrees): it stands 22 mm high, and the underside of its rim is 16 mm off the table at the very edge and less further in, too low for a finger to slide under. Two such plates lie face up where two diners left them on a wood table, and an empty deep bus tub stands on the table at the robot's right. Both plates have to go into the tub, set down (not dropped) and let go, and then both hands must move clear.

In the demo the left hand presses one side of a plate's rim down with about 15 N, so the plate tips 18 degrees on its foot ring and the opposite rim rises about 6.5 cm. The right hand slides level jaws round the raised rim, 9.8 cm from the plate's centre, and pinches it with 40 N (the YAM's figure), one pad under the rim and one on top. The left hand lets go, and the right lifts the plate, turns it about the vertical so it lines up with the tub, carries it with its lowest point 5 cm over the 17.6 cm wall, and lowers it into the tub. It then lays the plate down about its far foot ring until it is 3 to 4 degrees from flat, its far edge on the tub's floor or on the first plate, opens the jaws, lowers the open hand 1.2 cm so the near rim rests on the lower jaw, and slides the jaws out from under the rim before rising. The second plate goes on the first. Both arms then return to their start posture.

**Why it matters.** Impact on jobs. Clearing plates from tables is a core task of two large restaurant jobs. O*NET lists "Scrape and stack dirty dishes and carry dishes and other tableware to kitchens for cleaning" as a Core task of Dining Room and Cafeteria Attendants and Bartender Helpers (35-9011.00), the bussers, with 542,750 US jobs, and removing dishes from tables and taking them to the kitchen as a Core task of Waiters and Waitresses (35-3031.00), with 2,270,910 US jobs (BLS OEWS, May 2025).

**Ability tested:** taking a flat, rimmed, rigid plate off a hard table when no edge of it stands clear of the surface. One hand must tip the plate on its foot ring by pressing its rim, without sliding it away, while the other hand closes on the rim that rises; the plate must then be laid flat inside a deep tub, past its walls, and let go close enough to its support that its centre falls no more than 1.5 cm after the jaws open. The plate is the hard part: the plate's 15.9 cm well is solid and flat, so no finger gets under it (the rim's underside is 1.6 cm off the table at the edge and 0.5 cm where the well ends); pressing the rim tips the plate on its 11.6 to 13.2 cm foot ring and raises the near rim about 6.5 cm; once lifted, the plate swings in the pinch and the open jaw must catch its near rim, which is why the set-down is measured from the frame the jaws open; the lower jaw under the rim holds the near rim up, so even a plate laid flat settles about 8 mm. Neither hand can do both jobs: the hand that presses holds the tip, and the other's jaws need the raised rim. A single hand cannot drag a plate to the table's edge and pinch the overhang here: the side edges are out of the arms' reach, and the run below that tries the near edge never lifts the plate.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3000 \
  --instruction "Set both plates down in the bus tub, then move both hands clear."
```

## Scripted demo

The demo stays inside the YAM's figures: the pinch is 40 N a finger, the press on the rim about 15 N, and each plate 0.54 kg, held 9.8 cm from its centre, within the 1.5 kg payload.

From this folder:

```bash
python demokit.py task.py OUT --mode full                              # full score, with a video
python demokit.py task.py OUT --no-record --mode none,drop,hold        # the check runs below
python demokit.py task.py OUT --no-record --mode hover,one_hand
python demokit.py task.py OUT --no-record --mode flop,swing,wedge
python check_score.py                                                  # the scorer on hand-built poses, no simulator
```

The demo reads each plate's pose to find its rim and to lay it down, and the tub's pose to aim at its floor. It moves nothing except through the grippers.

## The scene and what is simplified

- Table: 120 x 80 cm, 75 cm high, wood. The two arm bases stand 61 cm apart at its near edge.
- Bus tub: a deep polypropylene bus box, 38.6 x 51.3 x 17.6 cm outside with a 46.3 x 33.6 cm floor, empty, its long side along the table's depth, free to slide, 1.1 kg (a design value). Its walls are thin sloping slabs; the moulded ribs and handles are left out.
- Plates: two 9 in (22.9 cm) round porcelain plates, 2.2 cm tall, 0.54 kg each, face up. Each rim is 24 convex segments that follow the rim's section, so a finger can go under its edge; the well and foot ring are solid. The rim's under-profile (1.6 cm off the table at the edge, 0.5 cm at 7.9 cm from the centre, where the 6 1/4 in (15.9 cm) well ends) and the foot ring (11.6 to 13.2 cm across) are design values: the maker gives the diameter, height and well only (well 6 1/4 in, 15.9 cm).
- Friction is estimated: porcelain on wood 0.4, on polypropylene 0.4 (0.35 sliding).

Estimated values, their bounds and how a real trial measures them. The scorer reads only the plates' and the tub's poses and the hands' positions, so its rules are the same for every value in these ranges.

| Value | Simulator | Bound and source | Real-trial measure | Effect on difficulty |
|---|---|---|---|---|
| Rim underside at the plate's edge | 16 mm | 13 to 19 mm: the plate's 22.2 mm (7/8 in) height, from the maker, less a porcelain rim 3 to 9 mm thick | feeler gauges or a steel rule under the rim edge, plate on a flat table (range 10 to 20 mm) | lower is harder: less room under the rim, so the tip matters more |
| Foot ring diameter | 11.6 to 13.2 cm | 11 to 14 cm: inside the maker's 15.9 cm well | calipers or a rule across the plate's underside (range 10 to 16 cm) | press force to tip it (next row) |
| Force to tip the plate by its rim | about 9 N | 6 to 11 N for the foot-ring range: tip about the near foot edge, F = m g r_f / (r_p - r_f), m 0.54 kg, r_p 10.4 cm press radius, r_f 5.5 to 7.0 cm foot radius | a luggage or spring scale pressed on the rim (range 0 to 20 N) | all within the demo's 15 N press and the arm's force |
| Friction, porcelain on wood and on polypropylene | 0.4 (0.35 sliding) | 0.3 to 0.6; a vertical press puts no sideways load on the plate, and the 40 N pinch holds the 0.54 kg plate for any pad friction above m g / (2 x 40 N) = 0.07 | tilt test: tilt the table top or a board with the plate on it until it slides, friction = tan(angle), range 17 to 31 degrees | lower friction lets a plate slide more readily when nudged |
| Bus tub mass | 1.1 kg | 0.9 to 1.5 kg for a 6 15/16 in deep polypropylene bus box of this size | kitchen scale (range 0 to 5 kg) | a lighter tub slides more easily if a hand or plate pushes its wall |
- Real-trial side camera and scale strip: not in the simulator, which reads the plate's pose directly; they let a real trial measure the same centre drop by eye on video, to about 3 mm (Building it for a real trial).
- The layout is the same in every trial. Food scraps and cutlery are left out: the act is the take and the set-down.

## Scoring

- In the tub: every sampled point of the plate (its rim's top and edge, and its foot ring) is inside the tub, above its floor, below its rim and inside its sloping walls, either way up.
- Let go: no point of either gripper (fingers, pads, palm, housing) or wrist link is within 1 cm of the plate.
- Set down: once let go, the plate's centre comes to rest no more than 1.5 cm below where it was at the frame the jaws open, measured to where it ends (a plate resting on an open hand has not come to rest). The simulator's rule: a hand whose finger joint reads more than 7.5 mm (a pinched rim reads 5.8 mm, open jaws 13 mm) no longer holds the plate; a plate that no closed hand has touched since it last came to rest is held while any part of a hand is within 1 cm of it and it moves, so a plate carried on open fingers is measured from where it was last carried. Real trial, read on the side camera's video (see Building it for a real trial): at the frame the jaws open and at the first frame the plate has stopped, read the height of its near rim and of its far rim against the scale strip. A rigid plate's centre lies midway between two opposite rim points, so its centre drop is the mean of the two rims' drops, and the plate is set down when that mean is no more than 1.5 cm, the simulator's threshold. When the view shows the far rim already resting on the floor or the plate below as the jaws open (its drop 0), this is the near rim dropping no more than 3.0 cm. A plate that falls further is dropped. A dropped plate never counts, even if it is picked up again.

| Score | When |
|---|---|
| 0.4 | for each plate in the tub, set down and let go |
| 1.0 | both plates in the tub, set down and let go, and for the last 1 s both grippers and wrist links at least 3 cm from each plate and the tub, with nothing moving more than 1 mm (real trial: nothing visibly moving for the last 1 s; the code checks only the body origins of the two plates (the centre of each plate's foot) and of the tub (the centre of its outer bottom)) |
| 0 | a plate leaves the table (5 cm below the tabletop) |

The run is scored on its state at the end. Taking a plate back out of the tub takes its 0.4 away again.

## Building it for a real trial

The two Libbey 1502-10235 plates and the Cambro 21157CBP bus box are stock restaurant-supply items, sold by many dealers: the box costs about US$15 to $25 and a plate about US$10. Put them on a wood or laminate table at the positions in `task.json`, with the arms as above.

The tub is opaque and the overhead camera looks straight down, so it cannot show how far a plate falls inside it. Two additions make the drop rule measurable:

- Side camera: a phone or webcam on a tripod beside the table at the tub's outer long side (the robot's right), level with the middle of the tub's length, its lens 90 cm above the tabletop and 10 cm out from the tub's outer wall, pitched 60 degrees down. From there it looks over the near wall (17.6 cm high) onto the whole tub floor except a band about 2.5 cm wide along the near wall (a plate at the set-down spot lies at least 5 cm from that wall), and onto the inside face of the far wall, so it sees both rims of a plate at the set-down spot and the moment its far edge touches down. Record at 30 frames per second or more.
- Height reference: a printed paper scale strip, 0 to 17 cm in 5 mm ticks, its zero at the tub's inner floor, taped upright on the inside face of the far long wall at the middle of its length, behind the set-down spot.

How to read a drop: the strip gives heights at the far wall. The plate stands 10 to 20 cm nearer the camera than the strip, so a vertical centimetre there spans more pixels; use the plate's own 2.2 cm height, seen at its near rim, as the scale at the plate, checked against the strip for the far rim, which lies close to the wall. A rim reading is good to about 3 mm, well inside the 1.5 cm threshold: the demo's plates fall 0.7 and 1.05 cm, and the plates in the `flop` and `drop` runs fall 1.9 to 12.7 cm.

## Results

Run on GPU physics at the README's commands, on an RTX 3090 Ti and an RTX 4070 Ti SUPER; the check runs record no video.

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | the whole task | 1.0, at step 1502 (75.1 s), the same on both GPUs; the first plate's centre fell 10.5 mm after the jaws opened, the second's 7.1 mm (limit 15 mm) |
| Dream-1 | the task's instruction, 3,000 steps, one run | 0.0 (both plates stay on the table, none reaches the tub; no session lost) |
| `none` | nothing done | 0.0 |
| `drop` | each plate carried over the tub and let go with the jaws opened 10 cm above the floor | 0.0 (both plates' centres fell 11.7 and 12.1 cm: dropped, never counted) |
| `hold` | both plates laid in the tub, the second never let go | 0.4 |
| `hover` | both plates laid in the tub, the jaws backed out of the second plate's rim only 4.5 cm and held there | 0.4 and 0.0 in two runs, each after another mode in the same simulator session (the first plate was let go about 1.4 cm over the floor and its centre fell 21 to 22 mm: dropped; the jaws left within 3 cm of the second plate also rule out the bonus) |
| `flop` | each plate laid in the open tub, clear of the walls, 13 degrees from flat, its far edge on the floor, and let go | 0.0 (the plates' centres fell 19.2 and 25.4 mm: dropped, never counted) |
| `swing` | the same, 30 degrees from flat | 0.0 (the plates' centres fell 44.0 and 41.8 mm: dropped) |
| `one_hand` | one hand only: presses on the plate's well and drags it toward the table's near edge to pinch the rim where it hangs over | 0.0 (the drag stops with the rim still 10 cm short of the edge, a pinch there is out of the arm's reach, and the plate is never lifted) |
| `wedge` | each plate taken with the jaws closed only to 7.7 mm a finger, just past the 7.5 mm at which the scorer stops counting a closed hand, carried and let go 10 cm over the floor | 0.0 (the loose jaws cannot lift the plate: it pivots out of them and stays on the table; the scorer case of a plate carried on open fingers and let go 10 cm up is in `check_score.py`) |

`check_score.py` runs the scorer on hand-built poses with no simulator: a plate flat on the floor, on the first plate, face down or leaning inside the tub counts; one beside the tub, on its rim, partly across a wall, on edge sticking out or under the tub does not. It also checks a plate let go 3 cm up (dropped), 8 mm up (set down), a plate that rests on the open jaw and drops 6 cm once the hand leaves (dropped), a plate whose jaws open under its rim and that then falls 4.5 cm (dropped), a plate carried on open fingers that were never closed and let go 10 cm up (dropped), open fingers resting still under a set-down plate and then withdrawn (set down), still held (0.4), a hand left 2 cm off (0.8), a plate taken back out after the hold (0.4), a plate off the table (0) and nothing done (0). All pass.

## Real items

- Bus tub: Cambro (maker), Cambox deep bus box, model 21157CBP, polypropylene, 15 3/16 x 20 3/16 x 6 15/16 in (38.6 x 51.3 x 17.6 cm). https://www.katom.com/index.php/144-21157CBP180.html
- Plate: Libbey (maker), Empire 9 in porcelain plate, model 1502-10235, 9 in (22.9 cm) across, 7/8 in (2.2 cm) tall, well 6 1/4 in (15.9 cm). https://www.webstaurantstore.com/libbey-1502-10235-empire-9-alpine-white-porcelain-plate-case/959150210235.html

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Workers: O*NET 35-9011.00 Dining Room and Cafeteria Attendants and Bartender Helpers (https://www.onetonline.org/link/details/35-9011.00) and 35-3031.00 Waiters and Waitresses (https://www.onetonline.org/link/details/35-3031.00). This includes information from O*NET OnLine by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence; O*NET is a trademark of USDOL/ETA.
- Job counts: BLS Occupational Employment and Wage Statistics, May 2025 national estimates, 35-9011 and 35-3031 (https://www.bls.gov/oes/).
- Bus tub: Cambro 21157CBP Cambox deep bus box, 15 3/16 x 20 3/16 x 6 15/16 in (https://www.cambro.com/Products/vending-carts/cambox--bus-boxes/poly-cambox/; https://www.katom.com/index.php/144-21157CBP180.html).
- Plate: Libbey 1502-10235 Empire 9 in porcelain plate, 7/8 in tall, 14.35 lb per case of 12 (https://www.webstaurantstore.com/libbey-1502-10235-empire-9-alpine-white-porcelain-plate-case/959150210235.html; https://www.restaurantsupply.com/products/libbey-1502-10235-9-inch-diameter-round-plate-porcelain-bright-white).
- Cambro and Libbey are named here for identification only; the names are their owners' trademarks.

## Licence

MIT
