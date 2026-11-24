# Tables and methods behind the README

The text and tables below are the detail of the [README](README.md).

## Trunk

Every part below is a fixed collider at full scale (the arms collide with all of them), in the station frame (x toward the seat from the midpoint between the arm bases, y to the robot's left, heights above the trunk floor, cm). Positions are in the middle column. No drawing of the Forte sedan's trunk was found; every size is either a page's own rough words, quoted, or an estimate.

| Part | Sim | Where the size comes from |
|---|---|---|
| Floor length, sill to seatback foot | 1.20 m, x -65 to 55 (1.10 and 1.30 m at the ends of the Range check) | [BestFamilyCars](https://bestfamilycars.com.au/blog/2022-Kia-Cerato-Review-and-Buying-Guide-big-boot-Small-car), a 2022 Cerato sedan review: "it's at least 1.2m again in length". The author writes just before it "I didn't get to do my usual boot measurements during this loan", so this is a rough figure, not a measurement |
| Width between the wheel arches | 1.20 m | same page, same rough standing: "about 1.2 metres between the wheelarches" |
| Height, floor to the parcel shelf underside | 60 cm | same page, same rough standing: "Height is between 500-600mm to the boot's 'ceiling'"; 60 cm is the top of it; 50, 55, 59, 59.5 and 61 cm were run and score 1.0, with a fingertip touching the shelf at 50 (Range check) |
| Width wall to wall | 1.35 m, walls at y 49.5 and -85.5 | estimate: the 1.20 m plus 7.5 cm of arch depth a side, itself an estimate |
| Knob's place | face at x 32 (97 cm in from the sill), 5 cm from the left wall, 52 cm up | estimate: placed, not measured; the manual's figure does not fix its depth (Knob place, below) |
| Wheel arches | 28 cm tall, 7.5 cm deep, x 7.5 to 57.5 | estimate, place and sizes; the 1.20 m between the arches is the rough figure above |
| Sill above the trunk floor | 12 cm, x -71 to -65 | estimate: a floor taken as 55 cm off the ground (not published). For scale, Practical Motoring, a Cerato S review that does not say sedan: "The load lip is around 67cm off the ground" |
| Opening | 70 cm long (sill to shelf edge), 1.15 m wide between its side flanges (10 cm in from each wall, x -65 to -50, 12 to 60 cm up) | estimate |
| Parcel shelf | underside at 60 cm, x 5 to 27, 3 cm thick, wall to wall, ending about 3 cm short of the seatback's top; it collides whole and is drawn as its two edges; the real trial uses an opaque board (What the cameras show) | estimate, except the height above |
| 60% seatback | upright and latched beside the 40% one, y -85.5 to -22, same recline | estimate; section widths, Sizes |

Cross-check, a rough one: the box inside the walls, 1.20 x 1.35 x 0.60 m, holds 0.97 m^3; the [2019 Forte manual](https://ownersmanual.kia.com/full_webhelp/BD/2019/en_US/manual/2019%20Forte%20OM.pdf), page 8-5 (page 534 of the PDF), gives the luggage volume as "434 l (15.3 cu ft)" (SAE), 45% of that box.

## Knob place

**Knob place.** The owner's manual figure does not fix the knob's depth: it could be at the opening's lip or under the shelf near the seatback's top, and it gives no distance. Three places were looked at, all at y 44.5, 5 cm from the left wall, with the trim block moved along with the knob.

- B: face at x 32, 52 cm up, this task's estimate and the only place in the scene. Nothing supports another place over it.
- A: face at x 5, the opening's front edge (the parcel shelf's rear edge), the tab's top 10 mm under the shelf's underside (57.75 cm up). This is one reading of the figure, and it is left out because it can be shown neither in the image nor clear of the trunk. Its tab is wholly outside the overhead image (by 116 px at 640 x 360 here, and at every base place looked at). In the scan (Base place) none of its passing places at 55 or 60 cm has clear straight joint paths. In the path check at 65 cm the grips then used, which are not the demo's, do not reach it, and with a grip taken from the scan the hand's way in under the trim block misses by 40 mm; not rerun with the demo's grip. The task is not shown for a knob at the opening's lip, and it has not been run there.
- C: face at x 30.1, the trunk-side top edge of the reclined seatback, the tab's top level with that edge (57 cm up). It is left out because it would need its own grip and its own way of letting go: with the grips then used, which are not the demo's, the wrist and a fingertip go 15 mm into the shelf, and with another grip, opening the jaws fully to let go puts a fingertip 4 mm into the left wall; not rerun with the demo's grip, whose jaws open to 33 mm to let go. Its tab is partly outside the overhead image (by 11 px). The task is not shown for a knob there, and it has not been run there.

Each miss is the best found, not a proof that nothing reaches.

- Release and fold sentence: [Kia Forte 2024 owner's manual, to fold down the rear seatback](https://ownersmanual.kia.com/full_webhelp/BDm/2024/en_US/topics/t00043.html), free online. Its figure OBDM031068L, the open trunk seen from outside, circles a place near each top corner and shows the release, "the seatback locking knob(1) in the trunk", as a tab marked PULL on a short rod, with an arrow out of the panel. The figure does not fix the knob's depth: it could be at the opening's lip or under the shelf near the seatback's top, and it gives no distance. The sim puts it beside the seatback top (this task's estimate; Knob place, below). It is not shown for a knob at the opening's lip or at the seatback's top corner, and was not run there. In the figure the right-hand knob can be seen from outside the car.

## Base place

**Base place.** Both bases stand 65 cm inside the sill at floor height (distances here are from a base's axis to the sill's inner face). Of the three distances compared, 55, 60 and 65 cm, it is the only one at which the latched seatback's top edge on the trunk side and the whole PULL tab fall inside the overhead image at all, and even there that edge is hidden by the drawn shelf edge. Nothing more is claimed for it. 55 and 60 cm were scanned and are not used: at 55 cm that edge is outside the image by 25.5 px and the tab at rest by 9.3 px, and at 60 cm by 4.8 px and 1.2 px (640 x 360, each at one sideways place, with the knob at B).

Sideways, the trunk stands so that the PULL tab stays whole in the image at rest and pulled; no score chose the place. At this base and place the knob hand's commanded path was checked by geometry only (the arm's collision meshes, no physics): its least clearance is 3.0 mm to the tab and 4.5 mm to the trim block, and 5.7 mm to the left wall when that wall is 3 cm from the knob. Gaps to the wheel arch and the 60% seatback were not measured on it. In the simulator, in the full run, the left hand keeps 82.8 mm (fingertips), 123.6 mm (gripper body) and 181.9 mm (link 5) from the parcel shelf. Checked by geometry at both ends of the recline, with 28 degrees the right hand's straight move from its ready pose to in front of the seatback goes 5.3 mm into the 60% seatback; the demo does not make that move, but a policy may try it.

The scan of base places is by geometry only (inverse kinematics to 5 mm and 3 degrees, the arm's collision meshes against the parts in the Trunk table, the trim block, the seatback and the headrest; no physics). Base places ran from 30 cm outside the sill to 65 cm inside it in 5 cm steps. At floor height there were 21 sideways places per distance, with the push point 33 cm up the seatback; a second set had 8 sideways places, base heights from the floor to 50 cm and push points 20 to 50 cm up. A place passes when the left hand holds the knob 20 mm out while the right hand pushes to 31 degrees, every step reaches, no link is more than 2 mm inside a part, both ready poses are clear and the arms stay 10 mm apart. The demo's own grips and sideways place were not required in the scan.

At floor height with the push point 33 cm up, the outermost base that passes is 55 cm inside the sill; nothing passes at 50 cm or nearer. Places passing out of 21, and of those the ones with clear straight joint paths between waypoints:

| Inside the sill | Knob at A | Knob at B (the scene's) | Knob at C |
|---|---|---|---|
| 50 cm | 0 | 0 | 0 |
| 55 cm | 2 (0) | 3 (2) | 3 (2) |
| 60 cm | 4 (0) | 8 (3) | 8 (2) |
| 65 cm | 7 (paths not computed) | 9 (paths not computed) | 9 (paths not computed) |

Distance to the left wall matters: with the knob at B 3, 5 or 20 cm from it, 2, 3 and 9 places pass at 55 cm. At the roomiest B place at 55 cm the arms keep 5 mm to the trim block, 7 mm to the parcel shelf and 23 mm to the 60% seatback; at all three B places there the sill is 465 mm or more and the opening's flanges 282 mm or more from every link.

At 50 cm a floor-height place passes only with the push point 20 to 25 cm up, where the same torque needs more force: 45 to 56 N by the 1/height rule (11.2 N m divided by the height) against 34 N at 33 cm, and no force has been tested. The pull alone, one hand and no push, passes nearer: 35 cm at B and C, 10 cm at A.

From a base 10 cm or more outside the sill, no hand reaches the knob or the seatback in any case: knob at A, B or C, any act, any base height from the floor to 50 cm. The nearest is one hand pulling a knob at A from a base raised 15 to 50 cm, standing on the sill line or over the sill (its axis 0 to 5 cm outside the inner face); no support under a raised base is modelled and the scene has none. Smallest misses found with any hand orientation and collisions ignored, floor height, push point 33 cm up and 31 degrees (mm):

| Base axis | Left hand to the knob, A / B / C | Push |
|---|---|---|
| 20 cm outside | 234 / 459 / 459 | 707 |
| 10 cm outside | 147 / 365 / 367 | 609 |
| 0 cm | 62 / 273 / 277 | 511 |
| 10 cm inside | 0 / 182 / 188 | 413 |
| 20 cm inside | 0 / 93 / 101 | 317 |
| 30 cm inside | 0 / 6 / 18 | 220 |
| 40 cm inside | 0 / 0 / 0 | 125 |
| 50 cm inside | 0 / 0 / 0 | 31 |

With the base at any height to 50 cm and any push height, the smallest misses from 10 cm outside are 21 / 291 / 272 mm for the left hand, 206 / 435 / 419 mm for the right hand, and 567 mm for the push.

Limits: a pass is sure only for the colliders modelled, which do not include the knob itself, the trunk lid, the body above 60 cm or the lid hinges. Each miss, and each place that does not pass, is the best found (for the places five starting poses, a finite set of grips and straight joint paths; for the two lists of misses three million sampled arm poses), not a proof that nothing reaches. The trim, knob and gap sizes are estimates, and a real car can differ from them by more than the 2 to 8 mm clearances printed here. No force, sag or slip was tested, and the scan itself tested no path of the demo (the check above did, by geometry).

## Opening reach check

A check by geometry of whether the left hand reaches a knob at the opening's lip (place A) from outside the trunk: the left hand alone, any hand orientation, joint limits kept 3 degrees in, collisions ignored, bases at floor height. 11,016 place-and-target cases: the base's axis at the sill's inner face and 10 and 20 cm outside it, 68 base places 2 cm apart across the trunk, nine turns of the trunk (0, 15, 30, 45 and 60 degrees each way), two knob heights (A's own, 57.75 cm, and 52 cm) and three targets on the tab (the fingertips' centre at the tab's face, 5 mm behind it, and the demo's fingertip point on the tab's lower half). None comes within 5 mm, and none of 31,311 tries that also asked for the hand's orientation (within 5 mm and 3 degrees) passes. Least misses (mm), at base y 26 to 28 cm and within 0.2 mm the same at every turn:

| Knob height | Base axis at the sill's inner face | 10 cm outside | 20 cm outside |
|---|---|---|---|
| 57.75 cm | 56.3 | 141.8 | 229.8 |
| 52 cm | 26.2 | 115.0 | 205.6 |

At the sill's inner face the base and the arm's first two links stand up to 30 mm inside the sill bar in the least-miss poses, and the base up to 14 mm inside the left wall. Not tried: raised bases, two hands, knob places B and C, other knob heights, a knob at A's depth 15 cm inboard, a knob nearer than the lip, force and sag. The arm poses are sampled, so a miss is the best found, not a proof.

## Both hands: reach check and held-seat check

The score reads the hinge angle and has no rule on the hands; a hand at the foot could touch the folded seatback. Reach check, by geometry (100 to 108 degrees of swing, either arm): in every pose found, the gripper point nearest the seatback is 0 to 14 cm up from the hinge, and none is 20 to 60 cm up. Held-seat check: none of the poses tried held the seatback at 103 to 112 degrees, which is not a proof: three sets of poses per arm and swing (13,280 aimed at the foot, A; 2,324 position-only, B; 60,000 random, C) at 103, 104, 106, 108, 110 and 112 degrees give 1,223 with a gripper point within 10 mm of the seatback and no arm point more than 2 mm inside anything. A linear program with the simulator's own joint limits (27 N m for joints 1 to 3, 7 N m for 4 to 6) and friction 0.9 at the nearest point, against the seatback's 25.4 to 26.0 N m, finds no hold when a contact at an edge may use the normals of faces within 0.5 or 1 mm, and 1, 2, 5 and 9 poses when they may be 1.5, 2, 3 and 5 mm away, all the left arm at one corner of the foot with closed fingers; with friction 0.5 none. Each of those 9 and all 118 poses with an arm point within 2 mm of a fixed part were run in the simulator, 288 runs on 242 poses in all: 0 held, the seatback at 113.0 degrees in the last second of every run. Not tried: two arms together, the arm colliding with itself, the path to a pose, part-open fingers, joint stops within 3 degrees, other base places, friction above 0.9, poses outside the three sets, poses touched only by the arm's other links (links 3 to 5 reach the seatback in many), moving or pulsed holds, a hand 2 to 20 mm from a fixed part that sags, and the 981 of the 1,223 poses that were judged by the program only.

## Sizes

| Item | Sim | Real | Source | Sim vs real |
|---|---|---|---|---|
| Seatback, 40% section | 0.55 m wide, 0.60 m tall, 0.10 m thick, rigid | foam on a steel frame; size not published | estimate. 55 of the 118.5 cm of both sections here, 46 percent; 60:40 is the trade name ([Practical Motoring](https://www.practicalmotoring.com.au/car-reviews/2019-kia-cerato-s-review/), a Cerato S review, "The rear seats are 60:40 split fold"; the Forte is sold as the Cerato in Australia, [Wikipedia](https://en.wikipedia.org/wiki/Kia_Forte)), and the section widths are estimates | rigid face: the push point does not sink in; neutral |
| Seatback mass | 9 kg, centre 0.30 m up from the hinge | 8 to 11 kg, weighed in the real trial | estimate (no weight published) | both ends run (Range check) |
| Seatback inertia | 0.45, 0.28, 0.30 kg m^2 along x, y (the hinge's axis) and z, about the centre of mass; the same at 8 and 11 kg | not published | estimate; a uniform slab of this size and 9 kg gives 0.50, 0.28, 0.23 | sets how fast the seatback falls once past upright; not varied |
| Recline at the latch | 25 degrees from vertical | 23 to 28 degrees | estimate; angle gauge in the real trial | both ends run (Range check) |
| Folded stop | recline plus 88: 113 degrees of swing, the hinge's limit (111 at 23, 116 at 28) | lying on the cushion | estimate; the manual's fold figure shows the seatback lying on the bench | neutral: gravity does this part |
| Folded, as scored | within 10 degrees of the stop, at rest for 1.0 s, angle moving 1.0 degree or less | gauge and film, 1 degree or less over 1 s | this task's rule | the demo's seatback moves 0.0 degrees over that second |
| Cushion top | a plate 54 x 55 x 1.2 cm, x 66 to 120, collides; the 60% cushion beside it is drawn 11 cm tall, not a collider | a foam bench; sizes not published | estimate | it does not catch the folded seatback: self_collisions are off, and the hinge limit stops the seat 0 to 19 mm above the plate |
| Hinge | damping 2 N m s/rad, friction 0.5 N m, effort 200 N m | a bare pivot bushing: small | estimate | both ends run (Range check) |
| Lock knob pull tab | flat tab 25 mm tall, 12 mm thick, 15 mm long along the pull | a tab marked PULL on a short rod; sizes not published | [owner's manual](https://ownersmanual.kia.com/full_webhelp/BDm/2024/en_US/topics/t00043.html) figure OBDM031068L for the shape; sizes are estimates (25 mm tall, 15 to 20 mm long); calipers in the real trial | 15 mm by default and at the hard end, 20 mm at the easy end (Range check) |
| Knob stem (head to trim) | 10 mm, the knob standing 25 mm out of the trim | 10 to 20 mm, so the knob stands 25 to 40 mm out of the trim | estimate; ruler in the real trial | 10 mm by default and at the hard end, 20 mm at the easy end (Range check) |
| Knob's motion | mass 0.03 kg, travel 20 mm, slider damping 2 N s/m, friction 0.2 N, effort 100 N | about 20 mm to its stop; mass and drag not published | estimate; ruler for the travel | neutral: the 7 N spring is the force |
| Knob return spring | 350 N/m, about 7 N at full pull | 5 to 10 N at full pull | estimate (cable release springs); spring scale in the real trial | both ends run (Range check) |
| Contact friction, seatback and knob | 0.9 static, 0.8 dynamic | carpet, fabric and plastic; not measured | estimate | 0.5 for both was run too, 1.0 (Range check) |
| Latch lock | a stiff joint lock on the hinge, 50,000 N m/rad, effort 2,000 N m | a steel claw on a striker: rigid | estimate | neutral: pushing on it moves nothing (pushonly, shove) |
| Latch release | at 15 of 20 mm of pull | 10 to 19 mm (near full pull) | estimate; ruler in the real trial | both ends run (Range check) |
| Striker clearance | 5 degrees of swing (about 5 cm at the top) | 3 to 7 degrees | estimate; angle gauge in the real trial | both ends run (Range check) |

No page bounds the estimates; the ranges run for them are this task's own.

The latch is the joint lock above, released while the knob is pulled 15 mm or more, and engaged again if the knob returns below that while the seatback is under 5 degrees; a seatback the latch has caught is then brought back to the striker by 0.1 degree a step (2 degrees a second). The return of the knob, the hold and the catch are this task's latch model, an estimate; the manual does not say the knob is held. Its sentence, "Pull out ... then fold", fits a hold and a release alike, and the hold is this task's choice: pulling the knob, letting it go and then pushing is a different act, which needs the latch to stay free with the knob back in, and that latch is not modelled. The knob is a colliding sprung slider the hand must pull; the seatback is a colliding hinged body under gravity, tipped by the hand's contact.

## Range check

**Range check.** Run on this code, mode `full` at the 25-degree recline and the other defaults unless said, each run on an RTX 3090 Ti and on an RTX 4070 Ti SUPER with the same score at the same step. The trunk parts and the knob's place are moved one at a time; the seat and latch estimates are moved together at the two ends (`FRSF_*` below are the switches). No point fails.

| Changed | Score | Success step |
|---|---|---|
| Hard end: 11 kg, recline 28, spring 500 N/m (10 N), release 19 mm, hinge damping 4, friction 1.5, striker 7 degrees, floor 1.30 m, tab 15 mm, stem 10 mm | 1.0; pullonly 0.0 | 240 (rest 116.0, pull 20.0 mm against a release at 19) |
| Easy end: 8 kg, recline 23, spring 250 N/m (5 N), release 10 mm, hinge damping 0, friction 0, striker 3 degrees, floor 1.10 m, tab 20 mm, stem 20 mm | 1.0; letgo 0.0, pushonly 0.0 | 200 (rest 111.0) |
| Contact friction 0.5 static and dynamic | 1.0 | 216 |
| Shelf 50 cm | 1.0 (touch, below) | 216 |
| Shelf 55 cm | 1.0 | 216 |
| Shelf 59 cm | 1.0 | 216 |
| Shelf 59.5 cm | 1.0 | 216 |
| Shelf 61 cm | 1.0 | 216 |
| Left wall 3 cm from the knob | 1.0 | 216 |
| Left wall 4 cm | 1.0 | 216 |
| Left wall 20 cm | 1.0 | 216 |
| Knob 2 cm nearer the robot | 1.0 | 216 |
| Knob 2 cm farther | 1.0 | 216 |
| Knob 2 cm higher (54 cm) | 1.0 | 216 |
| Knob 2 cm lower (50 cm) | 1.0 | 216 |

The seatback rests at 113.0 degrees in every run but the two ends. The shelf touch: with the shelf at 50 cm the left fingertip touches the shelf's front lower edge for 59 steps (from step 102 to 161), largest single point force 14.5 N (step 148), score unaffected, not at the default shelf (82.8 mm away at the least). The tab is at 20.0 mm at step 100, before the touch, with 0.76 mm of gap; before the tab moves the least gap is 11.89 mm; while touching, the tab is held at 19.88 to 19.94 mm. The gripper body stays 23.8 mm or more and link 5 82.3 mm or more from that shelf.

The demo takes the knob's place from the scene, so the knob and wall points show reach and clearance, not sensing. With the knob 2 cm lower the approach point in front of it is missed by 23.3 mm and 6.2 degrees, and with the knob 2 cm farther by 9.8 mm and 2.6 degrees; the grip itself is reached. The shelf runs move the walls' and the flanges' tops with it. The floor length moves only the sill and the flanges, which touch nothing, so it tests no access. The distance between the walls, the arches, the sill, the flange sizes, the opening, the 60% seatback's place and the knob's sideways place were not run. Nothing beyond these runs is claimed as ranged. Switches: `FRSF_KG`, `FRSF_RECLINE`, `FRSF_SPRING`, `FRSF_RELEASE`, `FRSF_HEAD`, `FRSF_STEM`, `FRSF_DAMP`, `FRSF_FRIC`, `FRSF_CLEAR`, `FRSF_FLOOR`, `FRSF_SHELF`, `FRSF_WALL`, `FRSF_MU`, `FRSF_KNOBX`, `FRSF_KNOBZ`.

## How the demo does it

**How the demo does it.** The left hand comes to the knob from below and in front, along its fingers, which point 51 degrees up, are turned 29 degrees toward the side wall and rolled 16: under the parcel shelf, clear of the side wall and the trim block. Its jaws, about 33 mm open, close across the two flat faces of the PULL tab, on its lower half, with 40 N a finger commanded, and it is commanded 36 mm back, against the tab's 20 mm stop, where the spring pulls back with about 7 N. The right hand, closed, sets its fingertips on the seatback's trunk face 33 cm up from the hinge and walks them forward along the seatback's arc, never more than 3 degrees ahead of the seatback angle, which the demo reads from the simulator. A policy does not get that angle. Once the seatback reads 8 degrees, the left hand opens its jaws to 33 mm only, since the side wall and the shelf are near, and backs off along its fingers. The right hand is commanded on to 31 degrees, 6 degrees past upright, but it leaves the seatback at about upright: its right fingertip is on the seatback from step 124 to 179 and is gone at step 180 (24.9 degrees), 20.7 mm away at 31.8 degrees. Its largest contact force is 51.7 N (dynamic, step 176), against the 34 N static figure. Gravity then folds the seatback down, and it rests there for the one second that full score takes. In the last 20 steps the least distances to the seatback are 443 mm (left gripper body), 444 (left fingertips), 459 (left link 5), 285 (right gripper body), 276 (right fingertips) and 323 (right link 5).

## Probe runs

Probe runs on the same code, with switches that are not among the folder's modes:

| Probe | What it does | Score |
|---|---|---|
| stop at the fold | `full`, ended at the first step the seatback reads 103 degrees or more (109.9 degrees, step 196) | 0.667: within 10 degrees of the stop but not at rest |
| held swing | knob pulled, the seatback swung to 10.3 degrees and held there with both hands to the end | 0.333 |
| early release | swung to at most 4.4 degrees, knob let go, hand off: the latch catches and brings it back to 0 degrees, then pushed at the same point (0.02 degrees) | 0.0 |
| pull again | as above, then pulled, held and pushed once more | 1.0 at step 492 |
| jaws round the tab, closed fully without a pull, at four places: the demo's own, 8 mm further in, 18 mm further in (round the rod), 12 mm deeper along the fingers (four runs) | tab travel 0.0 mm; the two pads load the tab 42 and 40, 45 and 41, 52 and 31, 46 and 41 N (largest single point force); round the rod the hand also pressed the trim up to 87 N without moving the tab | 0.0, four times |
| push on the neighbour | the push hand driven 8 cm into the 60% seatback | 0.0 |
| two trials in one session | `full` and then `none` after the simulator's own reset | 1.0, then 0.0 (the seatback back at the latch, knob at 0.0 mm) |

## Mode runs

| Mode | What the robot does | Score (GPU) |
|---|---|---|
| full | pull and hold the knob, push the seatback past upright, let go | 1.0 at step 216 (10.8 s), on an RTX 3090 Ti and again on an RTX 4070 Ti SUPER; off the striker (8.3 degrees) at step 151, past 103 degrees at step 196 (2/3 until 215), at rest at 113.0 degrees from step 197 |
| none | nothing (100 steps) | 0.0 |
| pushonly | push at the push point, without touching the knob, to the 1,200-step limit | 0.0 |
| pullonly | pull the knob 20 mm and hold it, no push (212 steps) | 0.0 |
| letgo | one hand at a time: pull the knob, let go, then push, to the 1,200-step limit | 0.0 (the latch has caught again) |
| shove | no knob; the push hand driven 8 cm into the latched seatback at the push point (190 steps) | 0.0 |

## What the cameras show

What is inside an image is from projection and ray tests on the scene's own meshes; what is lit well enough to be seen is read from the film. A part inside the overhead image but behind a drawn bar is not seen.

- Left wrist camera: the PULL tab whole at rest before the grip (steps 43 and 70), between the fingertips and drawn (steps 87 to 151), and back in the trim after the release (from step 168); from step 156 the seatback's edge at the image's right rim.
- Right wrist camera: black at 5 to 10 degrees of swing (steps 145 and 151); the seatback's ribs, dim, from step 156; the ribbed face and its side edge at steps 180 to 186; the folded seatback's top end and the headrest block at rest (steps 196 to 215).
- Overhead camera, seatback latched, arms out of the way: the trunk face up to about 55 cm is inside the image, below the bar; in the film the latched face is a black outline until about step 148. The top edge on the trunk side is inside, by 17.9 px at 640 x 360, but is hidden by the drawn shelf edge, so it is not seen; the top edge on the cabin side and most of the rounded top are outside.
- Overhead camera, seatback swinging: the trunk-side top edge leaves the image at 2.4 degrees; the lower part of the seatback stays between and beyond the bars. At 5 to 10 degrees the film shows only an outline, the seatback's dark edge against the lit strip beside and below it, with the hand on it. The push point, 33 cm up, is inside the image to 34.4 degrees and again from 103.9; at 20 to 31 degrees it is behind the drawn shelf edge. Folded, 65% of the trunk face is inside the image, and the folded seat is seen (steps 196 to 215).
- Overhead camera, PULL tab: inside the image at rest and when pulled 20 mm, by 18.2 px and 13.1 px at 640 x 360 (9.1 and 6.5 at 320 x 176); in the film it is small, at the upper left edge.
- In no view: the angle at which the seatback passes upright; the seatback past upright with the hand on it; the latched top edge, the opening, the sill, the flanges, the striker, the right wall and the right arch. The first picture shows the arms, the shelf's two edges and the floor; the seatback stands in shadow behind them (the setup picture, before the arms move).
- The shelf, a difference from the real trial: it collides whole and is drawn as its two edges, two bars, so that the overhead camera sees the seatback as a dark outline until it swings. In the real trial an opaque board lies there. In one render of the demo with the board drawn whole, the overhead view has no seat, tab, wheel arch or left hand, the left wrist camera still sees the tab and the right wrist camera the folded seat, and no camera sees the swing at 7 to 8 degrees.
