# 1.7 N·m of club on one grip, 0.7 N·m held: lift it with two hands, stand it upright, lower its grip end into a bag cell

A head-heavy 0.94 m, 0.42 kg iron puts about 1.7 N·m on the grip when held level, against the 0.6-0.7 N·m one hand holds, so one hand cannot lift it and every round of on-course golf ends with a two-handed return into a divider bag. A 7-iron lies flat on a table. One hand on the grip cannot lift it: the club turns in the grasp. The left gripper takes the shaft and the right takes the grip; together they lift it, roll it a quarter turn, raise it upright, head up, and stand it on its grip end. The left lets go. The right carries it upright over a cell of the bag's four-cell divider top, lowers the grip end in and lets go, and the club drops to the bag's bottom and stands, head up. The bag is a four-way stand bag with full-length dividers, free on the floor, rim 13 cm above the bench.

## Run

From this folder, on one GPU of the harness's kind:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,one,lifted,beside,hold,high,toss
```

The task's instruction field reads: "Put the golf club in the bag with both hands (the club puts 1.7 N·m on one grip; one hand holds 0.7 N·m): lower its grip end into a cell, let go, so it stands head up." The Dream-1 run used an earlier instruction, not this one: "Put the golf club in the golf bag: pick it up, lower its grip end into a cell, then let go so it stands in the bag, head up."

## Scoring, and the routes that score less

Every tier is read from the current state, so credit undone later is lost; only the failure latches and voids the run.

- **1.0**: the club's grip end inside the bag and within 3 cm of the bag's bottom, its head above the rim (the code checks only that; in the end photo the head end reads about 10 cm above the rim by ruler for a club seated at the bottom of the modelled bag, 9.5 cm in the demo; for another bag the band is (94 cm minus its measured inner depth) plus or minus 3 cm, 7 to 13 cm here, and a head end above the band means the club is stuck high), the club within 35 degrees of upright, the bag within 15 degrees of upright, every part of both grippers (fingertips, pads, palm, housing) at least 3 cm from the club, and all of it still (club under 2 cm/s and 0.2 rad/s) for 2 s (40 steps).
- **0.67**: the grip end inside the bag, below the rim, and the head above the rim, the bag within 15 degrees of upright (the club standing in a cell, head up, but not seated, still, or hands-clear).
- **0.33**: the whole club 8 cm or more above the bench where it lay.
- **0**: none of these, or the club's low end within 10 cm of the floor outside the bag with every part of both grippers 3 cm or more from the club (it fell). A club still carried or held low beside the bag does not void (`check_score.py` tests this rule on hand-made facts: free fall voids, a hand within 3 cm does not).

Real trial: a side phone video of the bag top and rim and a top-down video; a steel ruler held upright against the rim for the end photo (head end 7 to 13 cm above the rim); a 3 cm spacer block for the hands-clear check in the video; a spirit level or phone inclinometer on the bag collar for the 15-degree tilt; a tape mark 10 cm above the floor on the table leg, read in the side video, for the fell-outside check (a club's low end outside the bag and below the mark, with every part of both grippers 3 cm or more from it). Each check reads the same rule as the simulation. Any cell scores.

Run on an RTX 3090 Ti; scores are the harness's task_score.

| Mode | What it does | Score |
|---|---|---|
| full | both hands lift, raise and stand the club; right carries it over a cell, lowers and lets go | 1.0 |
| none | both arms wait | 0.0 |
| one | the right hand alone on the grip: the club turns in the grasp (63 degrees from upright at the lift) and ends on the floor | 0.0 |
| lifted | as full, then holds the club upright 6 cm higher and never carries it to the bag | 0.33 |
| beside | carried to a spot beside the bag and let go: it falls to the floor | 0.0 (void) |
| hold | as full, but never lets go in the cell | 0.67 |
| high | let go 3 cm over the cell, not lowered in | 0.0, a simulation outcome and not a release-height rule: the club falls in at speed, the 1 cm dividers and rigid bottom do not hold it, and it tips out over the rim and lands on the floor outside the bag (93 degrees from upright, run-end value), so the fell-outside void fires; the bag stays upright and the hands are over a metre away, so it is not hand drag. The full demo's lowered release does not tip out |
| toss | let go 30 cm over the cell | 0.0 (the club lands on the cuff and stays caught there, grip end 51 cm above the bag's bottom and lean 9 degrees at the end of the run) |
| Dream-1 | the harness's policy, given the earlier instruction above | 0.0 (1,800 steps) |

## Real trial

An iron (a Titleist T150 7-iron, 32 degree loft, 37 in / 94 cm, about 0.42 kg with its shaft; any steel-shaft 7-iron of that length serves), a Titleist Players S4 stand bag (4-way top with full-length dividers: top cuff 8.5 in = 21.6 cm, 34.8 in = 88.4 cm tall, 4.8 lb = 2.18 kg; modelled at these sizes with 8 mm walls and the cuff as the round top), a rubber mat under the bag, a table, the ruler, spacer block, level and two phone videos above. Approximate prices (estimates): a used steel 7-iron $20 to $40, the stand bag $150 to $250, a rubber mat $15, steel ruler, spacer block and level about $15 together. Reset: lay the iron flat along y with its grip end on the mark, head toward the left arm, stand the bag upright on its mark; about 20 s (estimate), nothing is consumed. Protection: use the YAM's rubber pad covers on the fingers, work on the mat, and keep feet clear of the floor under the club.

## Real items

- Iron: Titleist T150 7-iron (32 degree loft, 37 in), https://www.titleist.com/product/t150/560C.html. Any steel-shaft 7-iron of that length serves.
- Bag: Titleist Players S4 stand bag (4-way top with full-length dividers), https://www.titleist.com/product/players-s4/TB26SX4-001.html. Modelled at its top cuff width (8.5 in), height (34.8 in, as the maker lists it; the page does not say whether stand legs are included) and weight (4.8 lb); its stand legs, strap and pockets are left out.

## Sizes

| Part | In the scene | Source |
|---|---|---|
| 7-iron length, mass | 94 cm, 0.42 kg | Titleist T150 7-iron length 37 in (maker page); mass typical of a steel 7-iron |
| balance point | 63 cm from the grip end (0.42 kg body; the simulator's value) | estimate for a 7-iron |
| grip, shaft diameter | 25 mm (27 cm long), 12 mm | estimates |
| bag | top 21.6 cm across, 88.4 cm tall, 2.18 kg, 8 mm wall, two 1 cm dividers the full depth (four cells) | Titleist Players S4 (maker page: top cuff 8.5 in, height 34.8 in, 4.8 lb, 4 top-cuff dividers) |
| padded bottom, fabric drag | club velocity damped by 8 per second while its low end is inside the bag, whether or not a hand holds it; none in the air | stand-in for soft parts the rigid bag leaves out; the real slide time of a club in a bag is unmeasured |
| grip torque held by a pinch | 0.6 to 0.7 N m at 35 to 40 N | computed: 2 pads x (2/3) x mu 1.0 x F x r 12.5 mm |
| torque the club needs, held level by the grip 22 cm in | about 1.7 N m (0.42 kg x 9.8 x 0.41 m) | computed |
| grip friction, patch radius | 1.0, 12.5 mm | estimates |

## Run record

`python demokit.py task.py OUT --no-record --mode full,none,one,lifted,beside,hold,high,toss` on an RTX 3090 Ti: full 1.0; none 0.0; one 0.0; lifted 0.33; beside 0.0; hold 0.67; high 0.0; toss 0.0. README command (film run, `python demokit.py task.py OUT`): 1.0, 632 steps (31.6 s).

Dream-1 (one run, seed 1, 1,800 steps, 5 rows per request, one uninterrupted session): task_score 0.0.

## Sources

- Titleist T150 irons, https://www.titleist.com/product/t150/560C.html: 7-iron loft 32 degrees, length 37 in (set table).
- Titleist Players S4 stand bag, https://www.titleist.com/product/players-s4/TB26SX4-001.html: 4 top-cuff dividers, top cuff width 8.5 in, bag height 34.8 in, weight 4.8 lb.
- National Golf Foundation, Golf Participation in the U.S. 2025, https://www.ngf.org/member-publication/golf-participation-in-the-u-s-2025/: 48.1 million total participants (on and off course), aged 6 and over.
- Club + Resort Business, "NGF's Graffis report shows golf participation surpassed 29M for eighth consecutive year of growth", https://clubandresortbusiness.com/ngfs-graffis-report-shows-golf-participation-surpassed-29m-for-eighth-consecutive-year-of-growth/: more than 29 million on-course golfers.
- All meshes are built in `task.py` from boxes and cylinders; no third-party meshes are used.

## Licence

MIT
