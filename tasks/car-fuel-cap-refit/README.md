# Refit a car's ratcheting fuel cap: start it square in a recessed neck, turn to the click, push the sprung door to latch

A Volkswagen Golf VII is parked with its right rear quarter beside the arms' table, its fuel flap at the real height of 0.87 m above the ground and its fuel door standing open 90 degrees towards the arms. The fuel cap rests in a cradle on the back of the door. The left hand takes the cap by its grip bar, turns it to face the car, and starts its plug square in the filler neck at the back of the recessed pocket, its axis level. In one continuous turn of the wrist it turns the cap clockwise about a third of a turn, following it in as the thread draws it on, until the ratchet clicks. It lets go and backs away. The right hand then pushes the sprung fuel door round its hinge to flush and presses it in, against the latch's catch (8 N where it pushes), a little past flush until the latch catches, and backs away.

**Why it matters (Societal relatability).** Refitting the fuel cap is the last step of a fill-up on cars with a screw cap, and a cap left short of its click lets fuel vapour out. About 240 million people held a US driving licence in 2024, and 56% of drivers surveyed filled up at least once a week before 2020, so most of them meet this cap often. 92,530 automotive and watercraft service attendants (O*NET 53-6031.00) did it for a living in May 2023. Sources: licensed drivers ("The Nation reached 240 million licensed drivers in 2024"), US Federal Highway Administration, Highways report 2026, drivers page: https://www.fhwa.dot.gov/policyinformation/pubs/our_nations_highways_2026/drivers.cfm ; fill-up frequency ("56% of survey respondents filled up at least once a week", HONK Technologies survey, May to June 2020), NACS Daily, 24 Jun 2020: https://www.convenience.org/Media/Daily/2020/Jun/24/5-Survey-45-US-Drivers-Fill-Up-Once-Month_Fuels ; attendants, BLS Occupational Employment and Wage Statistics, May 2023, 53-6031: https://www.bls.gov/oes/2023/May/oes536031.htm .

**Ability tested:** starting a coarse thread square at the back of a recess, with the cap's axis level and the hand's housing kept clear of the open door, then turning the cap on in one wrist turn while following it in, and stopping at the click; then pushing a light sprung door round its hinge and pressing it past flush, firmly enough to work its push latch (a 2 N press leaves it unlatched).

**Arms.** The demo uses both: the left fits the cap, the right shuts the door. Both arms are not forced: the two steps come one after the other.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `taskspace.py` are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 1200 \
  --instruction "Put the fuel cap back on: start it square on the filler neck, turn it clockwise until it clicks, push the fuel door shut until it latches, let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                                         # full score, with a video
python demokit.py task.py OUT --no-record --mode none,push_only,no_door,undo,door_only,drop,light_door,weak_wrist   # the routes below
```

What the demo does:
1. The left hand comes in from the arms' left, fingers level, and closes across the cap's grip bar as it rests in the cradle.
2. It lifts the cap 2 cm out of the cradle, carries it 7 cm clear of the door, and turns it 90 degrees so that its plug faces the car. It winds its wrist roll back to -115 degrees, ready for one long turn.
3. It brings the cap level in front of the neck and squares it by feedback on the cap's observed pose, then moves it in towards 8.8 mm into the bore; the thread catches as the plug passes 8 mm.
4. It turns the cap clockwise in 18-degree steps of its wrist roll, following it in 1 mm a step (the thread's 20 mm a turn), until the ratchet clicks (eight steps; the cap turns about 126 degrees and 7.0 mm in). It never lets go mid-turn.
5. It opens its fingers, backs 10 cm out and moves away.
6. The right hand, its fingers pointing forward and 30 degrees down, comes to the open door's outer face and pushes it round its hinge to flush, its fingertips placed on the door's face about 12 degrees ahead of the door's current angle.
7. At flush it presses the door straight in with about 12 N, 10 cm from the hinge, against the latch's catch (0.8 N m, 8 N there), until the latch catches. It backs away.

The demo reads the cap's pose, the door's angle and the arms' joints; it reads no contact force and nothing from the scorer (its last push is set by its own joints' spring stretch). Nothing in the scene is random: `object_jitter=0`.

## The scene and what is simplified

Positions are (x forward from the arm bases, y left) in m from the midpoint between the bases; heights z above the ground. The arm bases stand at 0.75.

| Part | In the scene | Basis |
|---|---|---|
| Arms' table | the station's table taken out and replaced by a strip under the two arm bases, x -0.19 to 0.14, y -0.65 to 0.65, top at 0.7475; its front edge 0.23 m from the car's side | the arms keep their station poses; the car is parked beyond the table, as it would be in a real trial |
| Car | the right rear quarter of a parked Volkswagen Golf VII (2013 to 2015), fixed: the body side from the sill (0.20) to the roof (1.44), y -1.30 to the tail at 0.36, with the rear wheel in its arch (centre y -0.184, 0.30 up; tyre 205/55 R16), side window, C-pillar and rear lamp; its face at the pocket is the plane x 0.370; colliders are boxes 0.075 deep behind that face from the sill to the belt line (0.98), y -0.60 to 0.22, leaving the pocket open | the flap's centre height (0.87 m), its size and the rear wheel's place, measured on a near-orthographic side photograph of a Golf VII scaled by its 2,637 mm wheelbase (Volkswagen's data sheet); the body's curvature is an estimate |
| Fuel pocket | 238 x 145 mm opening, centred (y -0.030, z 0.870), floor 68 mm in | the flap's opening, measured on the same photograph; the depth is an estimate |
| Filler neck | 44 mm bore, 52 mm outside, axis along x at (y -0.010, z 0.870), mouth 50 mm in, 4 mm flared lead-in | estimate: a common 1.75 in neck |
| Fuel door | 232 x 140 x 3 mm, 150 g, vertical hinge at the pocket's right edge (y -0.147), light spring 0.06 N m/rad (at most 0.2 N m) holding it open | hinged at the pocket's edge towards the car's front, as on the Golf VII; size measured on the same photograph; mass and spring are estimates |
| Push latch | from flush its catch pushes back with up to 0.8 N m about the hinge (8 N at 10 cm from it, 3.4 N at the free edge); pushed 0.8 degrees past flush against it, it catches and holds the door flush | the catch of the Golf VII's push-to-open flap, as code; the force is an estimate |
| Cap cradle | a V cradle 45 mm off the door's back, low, so it sits under the fitted cap when shut | the cap holder on the back of the Golf VII's flap |
| Fuel cap | 67 x 20 mm shell, raised grip bar 50 x 14 x 12 mm, 40 x 16 mm plug, 80 g, free | the Golf VII's screw cap (Volkswagen 5Q0201550A), its retaining strap left out; sizes are estimates |
| Thread | catches with the plug 8 to 9 mm into the bore, coaxial within 1.5 mm, square within 4 degrees; 20 mm of travel a turn; seats 6.5 to 7.5 mm (about a third of a turn) on | a multi-start fuel-cap thread, as code |
| Turn | drag 0.15 N m, rising to 1.5 N m over the last sixth of a turn; past the click the ratchet slips and frees backwards (running drag only); backed off 0.8 mm past the catch, the cap is free | estimates, not measured: running drag 0.15 N m, click 1.5 N m, about 4.5 N and 45 N at the 33.5 mm rim; a real ratchet may slip somewhat higher or lower. Real-trial check: a torque screwdriver or a spring scale on the cap's rim at the click; a real ratchet that clicks harder makes the turn harder, not easier |

What is simplified: the thread's flanks are not geometry. The neck's bore and lip collide with the cap's plug and shell, and the thread is a rule that ties the cap's travel to its own turn once it is started square. The click and the door's push latch are code. The torques, the latch's force (0.8 N m; the scene's 3.4 N at the flap's free edge, to be checked on the car with a spring scale there) and the door's spring are estimates. The cap's retaining strap is left out. The cap's tilt joint stops at 80 degrees. The car's body is a visual mesh; it collides only through plain boxes behind its face around the pocket, and nothing in the task touches it elsewhere.

## Scoring

Read from the state at the end of the run; only the drop latches.

| Score | Condition |
|---|---|
| 0 | The cap fell below the pocket's lower edge (its centre under 0.77 m above the ground, 10 cm under the neck's axis; dropped), or its thread is not caught at the end |
| 0.25 | Started: the thread caught (the plug 8 to 9 mm into the bore, coaxial and square) and turned on at least 1 mm (about 18 degrees), not backed off |
| 0.5 | Turned on at least a quarter turn (5 mm of travel) |
| 0.75 | Turned on to the seat and the ratchet's click (turned back about 5 degrees, it is off the click) |
| 1.0 | Clicked; the door latched and within 2 degrees of flush; every part of both grippers (fingers, pads, palm, housing and the wrist link's end) 2 cm or more from the cap and the door with its cradle; held 3 s |

## Results

| Run | What it does | Score (GPU) |
|---|---|---|
| `full` | The whole task, both arms (RTX 3090 Ti, filmed) | 1.0 at step 575 (28.75 s) |
| `push_only` | The cap pushed onto the neck square and let go unturned; the door shut | 0.0 (caught, never turned) |
| `undo` | The cap turned to the click, turned back 30 degrees and let go; the door shut | 0.5 (off the click) |
| `no_door` | The cap fitted to the click; the door left open | 0.75 (door not latched) |
| `door_only` | The door shut with the cap left in its cradle | 0.0 (the cap in its cradle stops the door at 88.6 degrees, short of the latch; no cap on the neck) |
| `drop` | The cap lifted and let go | 0.0 (dropped) |
| `light_door` | As `full`, but the last push into the latch is 2 N | 0.75 (the door at flush, 90.12 degrees, not latched) |
| `weak_wrist` | As `full`, with the wrist roll's effort limit written down to 0.75 N m (half the click torque) while it turns | 0.5 (the wrist stalls at a quarter turn, 6.0 mm, short of the click; the cap is let go unclicked) |
| `none` | The arms wait | 0.0 |
| Dream-1 | The command above, the model driving both arms (seed 1, 20 rows per request) | 0.0 (1200 steps) |

## Runtime and seeds

The full demo takes 28.75 s of simulated time (575 of the 1,200-step limit). There is no randomness at `object_jitter=0`.

## A real trial

The real trial uses the car itself: a 2013 to 2015 Volkswagen Golf VII, whose right rear quarter carries this push-to-open fuel flap, the cap holder on its back and a strapped screw cap. Park it with that quarter beside the arms' table, its side square to the arms and the table's front edge 23 cm from it, so that the pocket's centre is 37 cm in front of the arm bases; with the bases 0.75 m up, the flap's centre (0.87 m above the ground) is 12 cm above them. Open the flap out square to the body side and rest the cap in the holder on its back. Reset: turn the cap off (counter-clockwise), rest it in the holder, press the flap's free edge and swing it open; about 20 s.

Kit: the car (borrowed for the session; no part of it is modified); the arms on a wheeled table; a spare cap in case its strap tears (Volkswagen 5Q0201550A, cap with retaining strap, list price 36.05 USD: https://oemwolf.com/oem-parts/5q0201550a.html ); a fine marker and a 30 cm straightedge (about 5 USD).

Measure the turn T once, before the session: with the cap started square on the neck, turn it clockwise from the catch to the ratchet click and read the angle off the rim mark (about 126 degrees is the scene's figure; the real cap's is its own). After each run, open the flap if it is shut and turn the cap on by hand, clockwise, against the mark on the pocket, and read what is left: the cap ratchets within 5 degrees, 0.75; the turn still left is T minus 90 degrees or less, 0.5; T minus 18 degrees or less and the cap still on the neck, 0.25; otherwise 0. A cap that falls below the pocket's lower edge (on its strap or to the ground), judged from the video, scores 0. The click is the ratchet slipping, with no further travel in. The latch is checked by the flap staying flush when let go, flush by the straightedge across the body side, and the hands by the video: 1.0 needs the latch holding, the flap flush, and both hands visibly off the cap and flap for 3 s.

## Real items

| Item | Maker and model | Link |
|---|---|---|
| The car, its fuel flap and filler | Volkswagen Golf VII (Mk7), 2013 to 2015 | https://www.volkswagen-newsroom.com/en/vehicle-data-golf-7-profile-20039 |
| Fuel cap | Volkswagen 5Q0201550A, cap with retaining strap | https://oemwolf.com/oem-parts/5q0201550a.html |

## Assets

Every mesh is made in code in `task.py` (boxes, prisms, lathed profiles and the body side's sheet); no third-party meshes, textures or materials. The car's dimensions come from a measured photograph, which does not ship: "VW Golf VII GTi CS Seite.JPG" by KarleHorn, Wikimedia Commons, CC BY-SA 4.0 (https://commons.wikimedia.org/wiki/File:VW_Golf_VII_GTi_CS_Seite.JPG), scaled by the 2,637 mm wheelbase in Volkswagen's data sheet (https://www.volkswagen-newsroom.com/en/vehicle-data-golf-7-profile-20039); on it the flap's centre is 555 mm above the rear wheel's centre (about 0.87 m above the ground) and 154 mm behind it. The robot and the station are the harness's own (`yam-isaac-lab-arena`).

## Licence

MIT
