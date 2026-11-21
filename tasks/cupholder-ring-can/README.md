# Set a soda can in a seat-back fold-down cup holder: fold down its tray, swing out its ring, set the can in upright

A seat-back panel section stands on the bench facing the arms, with a fold-down drink holder (Sea-Dog 588210-1) screwed to its face. The holder stows flat: its ring hangs in front of the plate and its tray stands folded up over it. One hand takes the tray by its sides and folds it 90 degrees down to its stop, then takes the hanging ring by its sides and swings it 90 degrees up and out to its stop. The other hand picks up a full 12 oz soda can high on its body, centres it over the ring, lowers it straight through (2 mm of play a side on the real holder, about 1.3 mm in the model) until it stands on the tray, and lets go. Both hands back away; the can stands upright on the tray inside the ring.

**Why it matters (Societal relatability).** Fold-down cup holders are on car and van seat backs, minivan doors, buses and cinema and stadium seats, and almost everyone has set a drink in one. North American cinemas alone sold about 760 to 780 million tickets in 2025. The US fleet is about 289 million light vehicles (2025). Sources: cinema tickets, Axios, "The 2025 box office comeback that wasn't" (2 Jan 2026): https://www.axios.com/2026/01/02/domestic-box-office-2025-movie-theaters (about 760 to 780 million admissions in the US and Canada, trackers differing slightly); vehicles, S&P Global Mobility, vehicles in operation, May 2025: https://www.spglobal.com/automotive-insights/en/blogs/2025/05/average-age-of-vehicle-in-us .

**Ability tested:** working a stowed two-part folding mechanism to its stops in the order it allows (the tray covers the ring), then seating a tall, top-heavy 66 mm can straight down through a 70 mm ring onto the tray floor 84 mm below it: a can lowered at an angle catches the rim and tips.

**Arms.** The demo uses both: the left opens the holder, the right carries the can. Both arms are not forced. The real holder has no published spring or detent, so each part stays where it is put, and one arm alone can open both, then fetch and set the can (`onearm` below scores 1.0).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 1200 \
  --instruction "Put the soda can in the seat-back cup holder: fold down its tray, swing out its ring, set the can upright through the ring onto the tray, and let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                                  # full score, with a video
python demokit.py task.py OUT --no-record --mode onearm,ringonly,ringfirst,beside,hold,drop,fastheld,none   # the routes below
```

What the demo does:
1. The left hand comes in with its fingers 60 degrees from straight down, toward the panel, and pinches the stowed tray across its side lips (84 mm) at 15 N a finger.
2. It carries the pinch round the tray's hinge line, the grip held; the tray turns between the pads and folds about 54 degrees down. The hand lets go, points its closed fingertips straight down and presses the tray's floor, 50 mm out from the hinge, down onto its stop.
3. It pinches the hanging ring across the band's widest point (80 mm), fingers near level, their tips 1 mm short of the plate and their upper edge under the side arms, and carries the ring rigidly round its pivot line to 60 degrees, the hand turning with it. It lets go, pinches the band again with the fingers square to it, and carries it the same way to its stop, the fingers ending straight down; it lets go and moves clear.
4. Meanwhile the right hand picks the can 110 mm up its body, fingers tilted 60 degrees from straight down, jaws across the can, and lifts it over the ring's height.
5. It centres the can's base over the ring by feedback on the can's observed pose, stands it straight if it leans, and lowers it straight through at 20 mm/s until its base is on the tray. Its fingers stay above the ring.
6. The right hand lets go, rises 10 cm and backs off.

The demo reads the can's pose, the holder's joint angles and the arms' joints; it reads no contact force and nothing from the scorer. Nothing in the scene is random: `object_jitter=0`.

## The scene and what is simplified

Positions are (x forward from the arm bases, y left) in cm from the midpoint between the bases; heights above the bench.

| Part | In the scene | Published | Source |
|---|---|---|---|
| Holder plate | 98 x 95 x 4 mm | B 3-7/8 in, A 3-3/4 in | Sea-Dog 588210-1 drawing |
| Ring inside | 70 mm | C 2-3/4 in (69.9 mm): holds drinks up to 2-3/4 in | Sea-Dog 588210-1 drawing and listing |
| Ring band, arms | 80 mm outside, 8 mm tall; two side arms to pivots at the plate's top corners | not published | product photo, estimate |
| Tray | 72 deep x 84 wide, 3 mm floor, 7 mm lips on front and sides; hinged at the plate's bottom | not published | product photo, estimate |
| Ring top over tray floor | 84 mm | not published | product photo, estimate |
| Projection open | ring front 95 mm from the panel face | D 3-7/16 in (87 mm) | Sea-Dog 588210-1 drawing; the open ring stands 8 mm farther out than D, from a side-arm length estimated off the photo; no hand uses the extra room |
| Depth stowed | 24 mm from the panel face (the stowed ring stands 1 mm off the plate) | E 15/16 in (24 mm) | Sea-Dog 588210-1 drawing |
| Can | 66 mm x 122.2 mm, 370 g | 12 oz / 355 ml standard 66 x 122.2 mm; 355 g drink + about 15 g can | Crown; Gamut Packaging |
| Seat-back panel section | 45 x 40 x 5 cm | not published | estimate |

- **Holder:** a Sea-Dog 588210-1 fold-up drink holder, modelled to its drawing and product photo, black ABS, screwed to the panel's face (plate face at x = 35.6). The ring pivots along y at (34.7, 0), 16 cm above the bench; open, its centre is at (30.5, 0) with its top face 16 cm up. The tray hinges along y at (33.9, 0), 84 mm below the ring's pivot; open, its floor's top is level with its hinge. Each part is one link with hard stops at stowed (joint 0) and open (90 degrees). No spring or detent; joint damping 0.05 N m s/rad holds each where it is put (estimate). Their weight (about 20 g each) is left out, so neither falls open by itself; the seller's "store in up position" says the stowed holder stays up. The two links collide with each other, so the stowed tray blocks the ring until it is folded down (`ringfirst` below); they do not collide with the plate they are jointed to, and the joint stops keep them in range. Stowed, the ring's band stands 1 mm off the plate and the tray's lips 1 mm off the ring, 24 mm deep in all, as on the real unit: no gap behind either part takes a fingertip.
- **Panel:** a rigid board, face at x = 36.0 cm, centred on y = 0, standing on the bench. A real seat-back holder sits about 60 to 70 cm above the floor; the panel is lowered to bench height for the arms' reach, at true scale.
- **Can:** one rigid body, the full can's mass (no fluid), standing on a 10 cm square mat at (22, -17).

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. The tiers are read from the state at the end; only the void conditions latch.

| What happened | Score |
|---|---|
| The tray and the ring each within 10 degrees of fully open | 0.33 |
| Also the can inside the ring: its axis within 10 degrees of vertical and of the ring's axis, crossing the ring's top face within 3 mm of its centre (2 mm a side on the real holder, about 1.3 mm in the model: its 16-flat ring is tighter, not easier), its base 3 mm or more below that face | 0.67 |
| Also the can's base within 2 mm of the tray floor, every part of both grippers 50 mm or more from the can and from the holder, the can still (under 0.01 m/s and 0.1 rad/s), for 2 s running | 1.0 |
| Void: once off its mat (its base risen over 10 mm), the can tilts past 30 degrees, or moves down faster than 0.3 m/s while no hand holds it (a free fall of about 5 mm; read after every physics step). A hand holds the can while its jaws are open no more than 1 mm a side past the can (68 mm across), a point of each finger is within 8 mm of the can, and the can moves down no faster than 0.1 m/s relative to the hand; a held descent at any speed does not void | 0 |

The hands-clear test reads a lattice of points 10 mm apart over each whole gripper (fingers, pads, palm, housing and the wrist link 12 cm back), plus the fingertip points, against the can's exact envelope and the holder's ring band and arms, tray, plate and lugs.

A real trial checks the same end state: both parts against their stops, the can's base on the tray floor, both grippers 5 cm or more from the can and the holder with nothing touching the can for 2 s, and after the trial a phone level app laid on the can's top reading under 10 degrees. A phone film from the side judges the voids: the can tipping past 30 degrees once off its mat, or falling free more than about 5 mm at any time once off its mat (the fingers open, or it slips out of the hand), read against a tape mark on the board; at release over the holder the can top must be within 5 mm of its seated height (38 mm over the ring's top face).

## Results

| Run | What it does | Score (GPU) |
|---|---|---|
| `full` | The whole task, both arms (RTX 3090 Ti, filmed) | 1.0 at step 914 (45.7 s) |
| `onearm` | The right arm alone opens the tray and the ring, then fetches and sets the can | 1.0 at step 1063 |
| `ringonly` | The tray and the ring opened; nothing else | 0.33 |
| `ringfirst` | The ring swung out with the tray still stowed over it | 0.0 (the stowed tray blocks the ring, which ends 1 degree open) |
| `hold` | As `full`, but the right hand keeps hold of the can on the tray | 0.67 (hands not clear) |
| `beside` | The can stood on the bench by the panel instead | 0.33 (holder open; can not in the ring) |
| `drop` | The can let go 35 mm over the ring | 0.0 (moves down at up to 1.06 m/s with no hand on it: void) |
| `fastheld` | As `full`, but the right hand, holding the can, lifts it 12 cm and brings it straight back down in 0.3 s first | 1.0 at step 958 (held, down at up to 0.64 m/s: no void) |
| `none` | The arms wait | 0.0 |
| Dream-1 | The command above, the model driving both arms (seed 1, 20 rows per request) | 0.0 (hit the 1,200-step limit) |

## Runtime and seeds

The full demo takes 45.7 s of simulated time (914 of the 1,200-step limit). There is no randomness at `object_jitter=0`.

## A real trial

Screw a Sea-Dog 588210-1 fold-up drink holder (about 10 USD from marine and RV suppliers) to a board standing upright on the bench, its ring pivots 16 cm above the bench, and stand a full 12 oz can on a mat 22 cm out and 17 cm to the right. Stow the holder: ring down flat, tray folded up over it. Reset: lift the can out, put it back on its mat, fold the ring down and the tray up; about 30 s.

## Real items

- **12 oz / 355 ml standard aluminium can**, 66 x 122.2 mm: https://crowncork.com/beverage-packaging/products/beverage-cans/12oz-355ml-standard .
- **Sea-Dog 588210-1 fold-up drink holder** (listed as 588210-1 / 588211-1), black ABS: a 98 x 95 mm plate, a ring 70 mm inside on two side arms pivoted at the plate's top corners, a tray hinged at its bottom edge; 15/16 in (24 mm) deep stowed. Maker: https://sea-dog.com/groups/1055-flush-mount-drink-holder ; drawing and listing: https://www.wholesalemarine.com/sea-dog-abs-folding-drink-holder/ .

## Assets

Every mesh is made in code in `task.py` (boxes, prisms and lathed profiles); no third-party meshes, textures or materials. The robot and the station are the harness's own (`yam-isaac-lab-arena`).

## Licence

MIT
