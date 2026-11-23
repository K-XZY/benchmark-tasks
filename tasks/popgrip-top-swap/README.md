# Swap a PopGrip top on a phone lying loose: twist the old top off, lock the new one on, then pull it open

About one in nine US adults keep a PopSockets-style grip on their phone, and on the swappable PopGrip the top comes off and goes on by a press and a quarter turn: a 38.8 mm disc standing 7.7 mm proud of a phone that lies loose on the desk, so one hand holds the phone while the other presses and twists.

## The act

An iPhone 15 lies face down on the desk. A PopSockets PopGrip base is stuck on its back, and the old top sits locked on it, collapsed. A trinket dish beside the phone holds the new top, face up. The left hand pinches the phone's two long sides from its top end. The right hand pinches the old top's rim, presses it down, turns it a quarter turn to its open slot, lifts it off and drops it in the dish. It then picks up the new top, sets it on the base at the slot, presses it, turns it a quarter turn the other way until it locks, and pulls the disc up to the grip's expanded height. Both hands let go and move clear.

Each step fails in a way a viewer can see:

- Turned without the press, the top does not move: the lugs hold it, and the gripper slips first.
- Pulled straight up without the turn, the top stays on the base.
- Without the left hand's hold, the twist turns the whole phone on the desk, and the lock never turns.
- Set on the base and let go unturned, the new top is seated but not locked.

## Real items

| Item | Maker and model | Price | Link |
|---|---|---|---|
| Phone grip with a swappable top: an adhesive base and a top that locks on it by a press and a quarter turn, with an accordion that pulls out | PopSockets PopGrip (swappable, Gen 2) | about US$15 | https://www.popsockets.com/pages/swappability.html |
| Phone | Apple iPhone 15, or a non-working display dummy of the same size with 171 g of ballast | dummy about US$20 | https://support.apple.com/en-us/111831 |

The maker's own steps for this act (https://www.popsockets.com/pages/swappability.html): close the grip flat, press down and twist 90 degrees to remove the top; to fit one, line it up, press down and twist until it clicks. A review shows the same steps on the real part (https://www.androidcentral.com/swappable-popsockets-review).

## Sizes

| Part | Sim | Real | Source |
|---|---|---|---|
| iPhone 15 | 147.6 x 71.6 x 7.8 mm, 171 g | 147.6 x 71.6 x 7.8 mm, 171 g | Apple, https://support.apple.com/en-us/111831 |
| PopGrip top | 38.8 mm across | 38.8 mm | retailer listing, https://www.jbhifi.com.au/products/popsockets-universal-popgrip-gen-2-translucent-clear |
| Grip height on the phone, collapsed / expanded | 7.7 / 24.3 mm (base 2.0, collapsed accordion with head 2.2, disc 3.5) | 7.7 / 24.3 mm | same listing; the split of the 7.7 mm stack is an estimate from product photos |
| Base | 36 mm across, 2 mm | not published | estimate from product photos; calipers on the bought part |
| Accordion travel | 16.6 mm | 24.3 - 7.7 = 16.6 mm | calculation from the listing |
| Top mass | 12 g | not published | estimate; kitchen scale (0.1 g) on the bought top |
| Trinket dish | 90 x 130 mm inside, 2 mm floor, walls 6 mm high in the sim (the real tray's are 25 mm: the 12 g top lies flat on the floor in both, and the lower sim wall leaves the gripper more room, so the real drop-in is slightly harder) | IKEA SKOGSVIKEN tray, 150 x 100 x 25 mm outside, polypropylene, about US$1 (https://www.ikea.com/gb/en/p/skogsviken-tray-black-90473486/); measure its inside and walls on arrival | the tray's real inside is within a few mm of the sim dish |

## Estimates the act depends on, their bounds, and how a real trial measures them

The quarter-turn lock is stated in code (below); its numbers are estimates. Each row gives the bound, the bench check, and the runs that show the demo and the lazy routes do not depend on the value inside the bound. The "low corner" run sets TURN_T 0.03 N m, a 2.8 N press (spring 4.67 N/mm) and accordion friction 0.5 N; the "high corner" run sets TURN_T 0.3 N m, an 11 N press (spring 18.3 N/mm), accordion friction 3 N, phone turn friction 0.09 N m, a 10 degree slot window, and lugs at 30 N. The "lazy low corner" run sets the unpressed stop at 1.5 N m, the lugs at 30 N, a 2.8 N press and TURN_T 0.03 N m.

| Constant | Sim | Bound | Measure on the real part | Difficulty | Runs across the range |
|---|---|---|---|---|---|
| Torque to turn the pressed top, TURN_T | 0.08 N m | 0.03 to 0.3 N m. Upper: 0.3 N m is 7.7 N tangential at each of two fingertips on the 19.4 mm rim, the most the maker's one-handed "press down and twist" asks of a thumb and finger; lower: 0.03 N m, any less and the turn would need no hold on the phone at all (its desk friction is 0.02 N m) | lever-and-cup rig (Real trial) on the top, pressed flat: weigh the cup at the turn, torque = weight x 0.1 m | higher is harder (the phone must be held harder) | full 1.0 at 0.03 (low corner) and 0.3 (high corner) |
| Press to free the turn: head 1 mm up on a spring, pressed when within 0.4 mm of its seat (SPRING_K 10 N/mm, PRESSED 0.4 mm) | 6 N | 2.8 to 11 N, the fingertip push-button resistance range of MIL-STD-1472H (push buttons, fingertip operation) | phone face down on the kitchen scale, tare, press the disc's centre with a fingertip, and read the scale at the moment the top turns freely | higher is harder | the demo presses 14 N; full 1.0 at 2.8 N (low) and 11 N (high) |
| Unpressed stop, HOLD_T | 3 N m | at least 1.5 N m: real lugs do not turn unpressed short of breaking; a 40 N pinch on the rim slips at about 1.4 N m (2 x 0.9 x 40 N x 19.4 mm) | lever-and-cup rig on the locked, unpressed top loaded to 1.5 N m (15 N at 100 mm, 1.5 kg): no turn; the lever is held to the disc by a G-clamp with rubber pads, checked by hand not to slip on the disc before loading (named HOLD_T lever clamp) | lower would let an unpressed twist pass | no_press 0 at 3 N m and at 1.5 N m (lazy low corner); the full demo turns only while pressed |
| Lug hold-down once turned, LUG_F | 60 N | at least 30 N: the expanded grip carries the 171 g phone in use with margin | tug test: phone held face down, a cup hung by string from the expanded disc, water added until the scale reads 3.06 kg (30 N) on the weighed cup: the top stays on; the 2 N scoring tug (204 g of water) is the same method | lower would let a straight pull lift it off | no_turn 0 at 60 N and at 30 N (lazy low corner); full 1.0 at 30 N (high corner) |
| Seat capture: coaxial, level, at a slot (CAPTURE_R, CAPTURE_TILT, CAPTURE_SLOT) | 1.5 mm, 4 degrees, 20 degrees | at most 1.5 mm a side (fit clearance); slot window 10 to 20 degrees either side, the lugs' own play: no maker or published value, an estimate bounded by the 90 degree stop's click, which a person can place within about 10 degrees by eye | index marks (below) and a feeler gauge at the seated head | wider is easier | full 1.0 with a 10 degree slot window (high corner); the radius is the 1.5 mm fit clearance, and the demo sets the top within 0.1 mm of the axis |
| Locked: turned to within LOCKED_WITHIN of the lock; lugs clear within OPEN_BAND of the slot | 10 degrees; 10 degrees | the real top clicks at its 90 degree stop; no published tolerance, an estimate; 5 to 10 degrees (the demo ends 4.5 degrees from the lock, so the score holds down to 5) | overhead photo of the index marks | narrower is harder | the demo run ends 4.5 degrees from the lock |
| Free once lifted FREE_UP at the slot | 3 mm | the head clears the base's lip within its 2 mm height | ruler at the side camera | none on the score | no_turn 0 (lugs hold) |
| Accordion pull-out friction | 1 N | 0.5 to 3 N | a cup hung by string under the disc's rim, phone held: add water until the accordion starts to move, weigh the cup on the scale | higher is harder | full 1.0 at 0.5 N (low) and 3 N (high) |
| Phone's desk friction: slide, turn | 0.5 N, 0.02 N m | slide: mu 0.2 to 0.5 on a desk; turn: 0.02 N m unpressed, about 0.09 N m under a 6 N press (0.3 x 7.7 N x a 40 mm mean radius); this one value, 0.09 N m, is used throughout | a cup of water hung by string over the pulley from the phone's side, water added until it slides, the cup weighed on the scale; the lever-and-cup rig on the top with the phone free | lower is harder (the sim keeps the unpressed value, so the phone turns more easily than on a real desk) | full 1.0 at 0.09 N m (high corner) |
| Top mass | 12 g | 8 to 20 g | kitchen scale | none: the weight differs by 0.1 N over the range, against a 14 N press | n/a |

## Departures

- The phone slides and turns on the desk but does not lift or tip: its frame is held at the table's height. A real 171 g phone lifts under about 1.7 N of upward pull. In the demo the upward loads on it are the accordion's pull-out (1 N, 3 N at the high corner: the pull ends at the stop), which ends at the accordion's travel stop (the hand is commanded 3 mm past the end of travel; the stop, a rigid limit with no modelled force value, takes the rest, as a real accordion's full extension does when a person stops pulling), and the 3 N that frees the top's head at the slot; the left hand pinches the phone's sides throughout, as a person's hand does. This makes the act slightly easier; a one-handed run still scores 0.
- The phone's turn friction stays at its unpressed 0.02 N m while the top is pressed; a real one rises to about 0.09 N m. This makes the sim harder, and the demo passes with 0.09 N m (high corner). The lazy-route table's one_hand stress run sets POPGRIP_PHONE_TURN_T=0.19 N m, twice that estimate, to show the zero does not hang on it.
- The quarter-turn lock (lugs, spring, click) is code acting on the top's joints, stated above; the top, base, phone, dish and fingers are colliding rigid bodies.
- The left hand pinches the phone's 71.6 mm width, inside the YAM gripper's 96 mm full opening, with its fingertips 4 mm above the desk on the phone's 7.8 mm sides; it opens there (TCP 19 mm above the desk) and rises straight up, clear of the desk.

## Scoring

Read at the end of the run; each tier is a check a person can make on the real phone.

| Score | Sim rule | Real check |
|---|---|---|
| 0 | the phone shifted more than 30 mm or turned more than 30 degrees on the desk | overhead camera against a grid under the phone |
| 0.33 | the old top off the base and in the dish | overhead photo |
| 0.67 | and the new top seated on the base and turned to within 10 degrees of its lock | overhead photo of the index marks; tug test (2 N straight up on the collapsed disc): it stays |
| 1.0 | and pulled up at least 15 of its 16.6 mm, still locked, both whole grippers (fingers, palm, housing) at least 1 cm from it, held 3 s | side camera with a ruler in view: disc top at least 22.7 mm above the phone's back (24.3 mm open, less the 1.6 mm the 15 mm rule allows); video: hands clear for 3 s |

## Real trial

Bought items: one iPhone 15 display dummy (147.6 x 71.6 x 7.8 mm) ballasted to 171 g, or a real iPhone 15, two PopSockets PopGrip swappable tops and one base, one IKEA SKOGSVIKEN tray as the trinket dish (about US$1; https://www.ikea.com/gb/en/p/skogsviken-tray-black-90473486/). Tools: an overhead camera and a side camera (phone cameras, 30 fps), a printed 10 mm grid under the phone, a ruler in the side camera's view, a printed paper protractor (1 degree marks) laid over the overhead photo, and one digital kitchen scale (Amazon Basics, 11 lb / 5 kg, 1 g steps, about US$12, https://www.desertcart.co/products/50479461) that does every force and torque check. Force: press on the scale (phone face down on it, tare, press the disc's centre, read the load at the free turn) or weigh a hung cup of water; 2.8 N is 286 g and 2 N is 204 g, so 1 g steps (0.01 N) resolve both. Torque: a 100 mm rigid lever (a ruler) clamped across the disc, a string at its end over a pulley (Screwfix Essentials steel single wheel swivel pulley, 25 mm, about US$3, https://www.screwfix.com/p/essentials-steel-single-wheel-swivel-pulley-25mm/125fe) to a cup; add water to the cup until the top turns, weigh the cup with the water on the scale; the empty cup hangs on the string from the start, so its weight is inside the weighed load and no offset is subtracted (the pulley's friction, a few percent at most on a 25 mm wheel, makes the reading slightly high; hang the cup with no top on the lever to check it hangs free); 0.03 N m is 0.3 N at 100 mm, 31 g, resolved to 3%. Tug method (a platform scale cannot pull, so no scale hook is used anywhere: every pull is a hung cup of water): hold the phone face down, hang a cup by string from the expanded disc, add water, and weigh the cup on the scale: 204 g of water is the 2 N tug, 3.06 kg the 30 N tug, within the scale's 5 kg range. The sim has no tug check: the 2 N tug is implied by the 60 N lug hold-down once turned. Cost of the whole rig and tools: about US$60 with a ballasted dummy phone (a real iPhone 15 adds the phone's price), including two tops and a base (about US$20), the dummy (about US$15), the scale, a dish (about US$1), the pulley (about US$3), a cup and printed paper; the cameras are phone cameras the lab already owns.

Index marks: draw a radial line on each top's face at its lug, and a matching line on the phone's back at the locked angle and at the open slot (90 degrees apart). The overhead photo reads the top's angle against them.

Bench setup, robot base frame, table top at z 0: the phone face down, centre at x 0.38, y -0.05 m, long side along y, camera plateau at the +y end; the base stuck on its back 40 mm from the -y end, as owners stick it; the old top locked on it, collapsed; the phone and the dish are loose on the desk by design, nothing holds them (the phone's only restraint is its friction); the dish centred at x 0.30, y -0.22, the new top in it at x 0.30, y -0.19, collapsed, face up, its lug mark along +x.

Steps of one trial: the robot holds the phone, presses and twists the old top off, puts it in the dish, sets, presses and twists the new top to its lock, pulls it up, and lets go; the run ends at 60 s.

Scoring by hand: 0 if the grid shows the phone moved more than 30 mm or turned more than 30 degrees; 0.33 if the old top is in the dish; 0.67 if the new top's mark is within 10 degrees of the lock mark (protractor on the overhead photo) and it stays on under a 2 N tug (cup of 204 g of water hung from the disc); 1.0 if the side camera shows the disc pulled up to its open height, still locked, with both hands at least 1 cm clear for 3 s.

One-hand check on the bought parts: with the left hand idle the phone is free on the desk; press and twist the top by hand with one hand. The score-0 rule for one hand is the shove limit: with the left hand idle the phone slides more than 30 mm or turns more than 30 degrees. Two effects can move the phone: it turns with the top when the pressed lock torque is above the phone's pressed turn friction (about 0.09 N m), and it slides when the single hand's push exceeds its 0.5 N slide friction. Either one scores 0. In the sim the slide decides at every bound (phone slides 139 mm even at lock 0.03 N m). If the bench pressed torque reads under 0.09 N m, record the torque and, from the grid, whether the phone turned and by how many degrees and mm it slid in the one-hand trial; the score is 0 only if the grid shows over 30 mm or 30 degrees. Bench checks before trials, on the bought parts: the pressed turn torque (0.03 to 0.3 N m), the press force (2.8 to 11 N), the unpressed stop (no turn at 1.5 N m), the tug test (3 kg, top stays), the accordion pull (0.5 to 3 N), each as in the estimates table. Reset: press and twist the new top off, return both tops to their start, wipe the base.

## Run

    python demokit.py task.py OUT --mode full

Demo: 1.0 (both arms, success at step 607, 30.35 s; RTX 3090 Ti). Dream-1 run: 0.0, valid single session of 1,200 steps (60 s); the film is a three-camera view of the run.

## Lazy routes

| Route | Score |
|---|---|
| none: the arms wait | 0 |
| one_hand: as the demo, the left hand idle | 0 (the phone is pushed and turns with the top; also 0 at the low-lock corner, lock torque 0.03 N m with pressed phone friction 0.19 N m (stress value POPGRIP_PHONE_TURN_T=0.19, twice the 0.09 estimate): the lock turns easily there, but the single hand's push still slides the phone 139 mm and turns it 180 degrees, over the 30 mm shove limit, so the zero is from sliding, not torque. The sim does not claim a one-handed pass anywhere in the bounds) |
| no_turn: the old top pulled straight up | 0 (the lugs hold it) |
| no_press: the turns without the press | 0 (the unpressed stop holds) |
| set_only: the new top set on the base and let go unturned | 0.33 |

## Sources

- YouGov, November 2023, 1,104 US adults: 11% use a phone grip such as a PopSocket, https://yougov.com/en-us/articles/48154-american-consumers-and-cell-phone-accessories-spending-patterns-and-preferences
- PopSockets PopGrip listing: close flat, press down and twist 90 degrees to swap the top; easy removal for wireless charging, https://att.com/buy/accessories/specialty-items/popsocket-popgrip-bubbly.html
- PopSockets: over 275 million grips sold in 75 countries, https://www.popsockets.com/pages/about-us
- MIL-STD-1472H, Department of Defense Design Criteria Standard: Human Engineering, push buttons, fingertip resistance 2.8 to 11 N.
