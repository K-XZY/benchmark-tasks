# Bus two flat 9 in porcelain plates into a deep tub: one hand tips each plate up by its rim so the other can grip it

A full-size porcelain dinner plate lying flat on a table gives a gripper nothing to hold until one hand presses its rim down with 15 N and tips it 18 degrees on its foot ring: it stands 22 mm high, and the underside of its rim is 16 mm off the table at the very edge and less further in, too low for a finger to slide under. Two such plates lie face up where two diners left them on a wood table, and an empty deep bus tub stands on the table at the robot's right. Both plates have to go into the tub, set down (not dropped) and let go, and then both hands must move clear.

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
python demokit.py task.py OUT --no-record --mode flop,swing
python check_score.py                                                  # the scorer on hand-built poses, no simulator
```

The demo reads each plate's pose to find its rim and to lay it down, and the tub's pose to aim at its floor. It moves nothing except through the grippers.

## The scene and what is simplified

- Table: 120 x 80 cm, 75 cm high, wood. The two arm bases stand 61 cm apart at its near edge.
- Bus tub: a deep polypropylene bus box, 38.6 x 51.3 x 17.6 cm outside with a 46.3 x 33.6 cm floor, empty, its long side along the table's depth, free to slide, 1.1 kg (a design value). Its walls are thin sloping slabs; the moulded ribs and handles are left out.
- Plates: two 9 in (22.9 cm) round porcelain plates, 2.2 cm tall, 0.54 kg each, face up. Each rim is 24 convex segments that follow the rim's section, so a finger can go under its edge; the well and foot ring are solid. The rim's under-profile (1.6 cm off the table at the edge, 0.5 cm at 7.9 cm from the centre, where the 6 1/4 in (15.9 cm) well ends) and the foot ring (11.6 to 13.2 cm across) are design values: the maker gives the diameter, height and well only (well 6 1/4 in, 15.9 cm).
- Friction is estimated: porcelain on wood 0.4, on polypropylene 0.4 (0.35 sliding).
- The layout is the same in every trial. Food scraps and cutlery are left out: the act is the take and the set-down.

## Scoring

- In the tub: every sampled point of the plate (its rim's top and edge, and its foot ring) is inside the tub, above its floor, below its rim and inside its sloping walls, either way up.
- Let go: no point of either gripper (fingers, pads, palm, housing) or wrist link is within 1 cm of the plate.
- Set down: once let go, the plate's centre comes to rest no more than 1.5 cm below where it was at the frame the jaws open, measured to where it ends (a plate resting on an open hand has not come to rest) (the simulator's rule: a hand whose finger joint reads more than 7.5 mm, a pinched rim reading 5.8 mm and open jaws 13 mm, no longer holds the plate). Real trial, on the overhead-camera video, which sees both rims: after the jaws open, the plate's near rim drops no more than about the plate's own height (2.2 cm, the scale in the frame), with its far edge already on the floor or the plate below. A plate that falls free from higher is dropped. A dropped plate never counts, even if it is picked up again.

| Score | When |
|---|---|
| 0.4 | for each plate in the tub, set down and let go |
| 1.0 | both plates in the tub, set down and let go, and for the last 1 s both grippers and wrist links at least 3 cm from each plate and the tub, with nothing moving more than 1 mm (real trial: nothing visibly moving for the last 1 s; the code checks only the two plate centres and the tub centre) |
| 0 | a plate leaves the table (5 cm below the tabletop) |

The run is scored on its state at the end. Taking a plate back out of the tub takes its 0.4 away again.

## Building it for a real trial

The two Libbey 1502-10235 plates and the Cambro 21157CBP bus box are stock restaurant-supply items, sold by many dealers: the box costs about US$15 to $25 and a plate about US$10. Put them on a wood or laminate table at the positions in `task.json`, with the arms as above.

## Results

Run on GPU physics at the README's commands, on an RTX 3090 Ti or an RTX 4070 Ti SUPER; the check runs record no video.

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | the whole task | 1.0, at step 1429 (71.45 s); the first plate's centre fell 7.7 mm after the jaws opened, the second's 8.0 mm (limit 15 mm) |
| Dream-1 | the task's instruction, 3,000 steps, one run | 0.0 (both plates stay on the table, none reaches the tub; no session lost) |
| `none` | nothing done | 0.0 |
| `drop` | each plate carried over the tub and let go with the jaws opened 10 cm above the floor | 0.0 (both plates' centres fell 11.9 and 12.7 cm: dropped, never counted) |
| `hold` | both plates laid in the tub, the second never let go | 0.4 |
| `hover` | both plates laid in the tub, the jaws backed out of the second plate's rim only 4.5 cm and held there | 0.8 (both plates set down and let go; the jaws stay within 3 cm of the second plate, so there is no bonus) |
| `flop` | each plate laid in the open tub, clear of the walls, 13 degrees from flat, its far edge on the floor, and let go | 0.0 (the plates' centres fell 19.5 and 25.5 mm: dropped, never counted) |
| `swing` | the same, 30 degrees from flat | 0.0 (the first plate's centre fell 44 mm; the second, laid on a plate that had fallen, is not set down either) |
| `one_hand` | one hand only: presses on the plate's well and drags it toward the table's near edge to pinch the rim where it hangs over | 0.0 (the drag stops with the rim still 10 cm short of the edge, a pinch there is out of the arm's reach, and the plate is never lifted) |

`check_score.py` runs the scorer on hand-built poses with no simulator: a plate flat on the floor, on the first plate, face down or leaning inside the tub counts; one beside the tub, on its rim, partly across a wall, on edge sticking out or under the tub does not. It also checks a plate let go 3 cm up (dropped), 8 mm up (set down), a plate that rests on the open jaw and drops 6 cm once the hand leaves (dropped), still held (0.4), a hand left 2 cm off (0.8), a plate taken back out after the hold (0.4), a plate off the table (0) and nothing done (0). All pass.

## Real items

- Bus tub: Cambro (maker), Cambox deep bus box, model 21157CBP, polypropylene, 15 3/16 x 20 3/16 x 6 15/16 in (38.6 x 51.3 x 17.6 cm). https://www.cambro.com/Products/vending-carts/cambox--bus-boxes/poly-cambox/
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
