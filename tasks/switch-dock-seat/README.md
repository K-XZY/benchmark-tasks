# Dock a Nintendo Switch: lower the console straight down its dock's slot until its port seats on the hidden USB-C plug

Most people with a Nintendo Switch put it back in its dock to play on the TV or to charge it. The console goes straight down the dock's slot until the USB-C plug at the bottom, out of sight, enters the port on the console's lower edge. The slot is about 1 mm wider than the console on each side and the plug is an 8.25 x 2.4 mm tongue, so the console has to come down upright and centred, its screen facing the way the dock's front panel does (Nintendo's own step), until the port is fully on the plug.

**What this task adds.** Plug-in tasks push a held plug into a visible port. Here the port is on the 242 mm wide, 420 g console and the plug is fixed and hidden at the bottom of a slot. The console starts screen down on a staging rest of two blocks, one under each Joy-Con, so its screen glass touches nothing, with its top edge overhanging; each hand closes on the 13.9 mm top edge at a Joy-Con end, and both hands stand it up with one 90 degree wrist pitch and lower it.

**How the demo does it.** Both hands take the top edge at the Joy-Con ends, 104 mm either side of the middle (jaws up and down across 13.9 mm, 35 N a finger, each hand's fingers turned 30 degrees outward, past the console's end), at points read from the console's pose on the rest, lift, carry the console level over the dock, and pitch it 90 degrees so it hangs upright with its screen toward the robot. From there every move is closed on the observed poses of the console and the dock, read from the simulator (a policy gets only the cameras). The middle of the console's lower edge, where the port is, is brought over the plug to within 0.3 mm across and along the slot, and the console is squared to the slot to within 0.23 degrees, before any descent; the pocket clears the plug by 0.5 mm a side, and at 0.3 degrees of yaw the console's ends move 0.63 mm across the slot, inside its 1.05 mm clearance a side. It then goes down the slot's axis in short steps, 4 mm above the slot's rim, 3 mm inside it and 1 mm over the last 8 mm, and at each step both hands correct the edge's place and the console's squareness before the next. The last 6.2 mm is a push until the port is on the plug.

**Protecting the screen and the pads.** In a real trial the screen is covered by its own film or a microfibre sleeve, the rest blocks are topped with a microfibre cloth, and the gripper pads wear nitrile finger cots or a soft pad cover. In the simulator the pads are plain and nothing is soft, so the demo is built not to touch the glass at all: the console lies on two blocks under its Joy-Con (screen face free), and both hands grip only the frame at the Joy-Con ends, where there is no glass (the glass is 154.9 x 87.2 mm, so it ends 77 mm from the middle; the pads centre 104 mm out and are 36 mm wide). The scorer counts every step in which any of the 24 points on the fingers is over the glass (printed as `screen_steps`; not scored): 0 in the full run.

**In the slot.** The scorer counts a console as in the slot only when its lower edge, its middle and its lowest corner, is at least 4 mm below the dock's rim and the middle of that edge, where the port is, is over the opening: within the slot's 16 mm width and within 34 mm of the dock's middle along its length, so the 242 mm console spans the whole 174 mm dock with both Joy-Con past its ends. This is checked at every step the tiers are read. A console left beside the dock, stood off to one end of the slot, or with its port past either end, scores 0; a console leaning more than 15 degrees scores 0. The plug's friction acts while the plug is in the port and the console is within 8 degrees of upright.

**Lowered, not dropped.** The plug's resistance and the console's arrival speed are read every physics step (240 Hz), not only at the 20 Hz control steps. While the plug is in the port, it resists as friction along the console's axis, up to 8 N either way, applied as a force on the console and its reaction on the dock (the plug clears the pocket's walls by 0.5 mm, so the friction is set by code, not by contact); the console's own 4.1 N weight does not overcome it, so the hands push the rest. The resistance holds whether or not the port is home, going in or coming out, so a seated console stays seated and a half-seated one stays half-seated. The arrival speed is the console's speed into the plug at each step its lower edge reaches the plug's tip, onto the port or anywhere beside it along the edge; only a console lifted 4 mm or more off the plug is judged afresh, so a seat-lift-drop is judged on its last arrival and a strike beside the port counts as a drop. Upright and square are read against the dock's own frame. Hands clear means every finger point, 9 points through each gripper's housing and every arm link's origin 3 cm from the console.

**Result** (scripted, GPU physics, 20 Hz, the upright dock at ChargerLAB's measured size): `python demokit.py task.py OUT` gives 1.0 in 611 steps (30.55 s), filmed at 640 x 360 with no blank frame. Port 6.5 mm onto the plug (stop at 6.6), tilt 0.08 degree, `screen_steps` 0. Every route below was run on GPU with the scorer as below, scored by the same function (`--no-record`). No scripted route ends at 0.25: the tilted jams we ran lean further once let go and move the dock (0).

| Check | What it does | Steps | Score |
|---|---|---|---|
| `full` | the demo | 611 | 1.0 |
| `none` | the arms wait | 100 | 0 |
| `drop` | let go over the plug from the hover height, the port 64 mm above it | 328 | 0.5 (it arrives at 1.1 m/s and seats: the stated tier for a console dropped in) |
| `drop10` | aligned, lowered inside the slot and let go 10 mm above the plug, hands lifted clear | 610 | 0.5 (it arrives at 0.45 m/s and seats: dropped in, not lowered) |
| `rest` | aligned, lowered inside the slot and let go 1.2 mm above the plug, hands lifted clear | 647 | 0.5 (the plug's friction holds it with the port 2.0 mm on the plug: its own weight does not seat it; upright in the slot, not seated) |
| `redrop` | seated gently, lifted 3 cm off the plug in the hands, let go | 713 | 0.5 (judged on its last arrival, at 0.47 m/s; it stops on the plug's tip) |
| `crooked` | aligned, then lowered with a 10 degree roll across the width | 935 | 0 (it jams tilted; let go, it leans over and pushes the dock off its mark; at a 7 degree roll the same, 777 steps) |
| `beside` | stood upright on the bench 8 cm beyond the dock (away from the robot), let go | 440 | 0 (it stands upright and still, its port 88 mm from the slot) |
| `pastleft` | lowered with the port 30 mm past the dock's left end, let go | 563 | 0 (it topples off the dock's end and moves the dock; held there, its port is outside the opening, which also scores 0) |
| `pastright` | lowered with the port 30 mm past the dock's right end, let go | 589 | 0 (the same; the right hand ends up to 20 mm off its target at the edge of its reach) |
| `undo` | the drop, left to settle 2 s (0.5 by then), then gripped again and lifted 10 cm back out and held | 449 | 0 (every tier is read from the state at the end, held still for 1 s; a tier reached earlier and undone does not count) |

**The seating force across its sourced range.** The plug's 8 N is an estimate inside USB Type-C's 5 to 20 N insertion range. Run at both ends (`SW_DETENT=5` and `SW_DETENT=20`), the demo scores 1.0 in 609 and 637 steps; at the value used, 8 N, the hands push about 4 N beyond the console's weight. The force the task hinges on is the alignment, not the push.

## Size check

`python -c "import task; print(task.size_check())"` prints the scene's key sizes against the published ones:

```
console height x width x thickness, mm: scene (102.0, 242.0, 13.9), published (102.0, 242.0, 13.9)
console mass, g: scene (420.0,), published (420.0,)
dock height x length x depth, mm (ChargerLAB, measured): scene (103.7, 174.0, 47.4), published (103.7, 174.0, 47.4)
dock mass, g (ChargerLAB, weighed): scene (286.0,), published (286.0,)
dock height x length x depth, mm (Nintendo, about): scene (103.7, 174.0, 47.4), published (104.1, 175.3, 50.8)
dock mass, g (Nintendo, about): scene (286.0,), published (313.0,)
screen diagonal, in: scene (7.0,), published (7.0,)
plug width x thickness, mm: scene (8.2, 2.4), published (8.2, 2.4)
seating resistance, N: scene (8.0,), published (5.0, 20.0)
slot width against the console's thickness, mm: scene (16.0,), published (13.9,)
```

The seating row gives the USB Type-C range's two ends; the slot row gives the console's thickness, since the slot has no published width (its range is in the measurement table below).

**Run record.** On the final code: the README command three times on one GPU type (RTX 4070 Ti SUPER), 1.0 each in 611 steps with no blank frame, and once on another (RTX 5090), 1.0 in 611 steps; four seeds with the console and dock shifted up to 5 mm (`--seed 1` to `4 --jitter 0.005`), 1.0 each in 609, 645, 652 and 711 steps; the routes and the force ends as in the tables above.

## Earlier feedback

- Construct validity, "The simulator can award partial credit for a console left on the bench beside the dock, without performing the docking task." The scorer's slot test now needs the middle of the console's lower edge, where the port is, over the dock's opening in both width and length (section "In the slot" above). The `beside` route scores 0.
- Scoring, "The in_slot check omits the console's position along the dock, allowing a console entirely beyond either end to score." Position along the dock is now part of the test; `pastleft` and `pastright` (the port 30 mm past either end) score 0.
- Flag, "protect the console screen and gripper pads when gripping and resting the device." Film or a microfibre sleeve on the screen, nitrile finger cots or a soft pad cover on the pads, a cloth-topped rest under the Joy-Con only; the demo never touches the glass (`screen_steps` 0).
- Partial credit is read from the end state only: the `undo` route (dropped in at 0.5, then lifted back out) scores 0.
- Drops are read every physics step and each arrival is judged afresh: `drop10` and `redrop` score 0.5, the dropped-in tier (section "Lowered, not dropped").

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `taskspace.py` are in this folder.

## Run a trial

```bash
python demokit.py task.py OUT --mode full                                    # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode none,drop,drop10,rest,redrop,crooked,beside,pastleft,pastright,undo   # the other routes
python demokit.py task.py OUT --no-record --mode full --seed 1 --jitter 0.005                       # console and dock shifted up to 5 mm (seeds 1 to 8)
```

## Building it for a real trial

- **Console:** a Nintendo Switch OLED model with both Joy-Con attached, 102 x 242 x 13.9 mm, 420 g.
- **Dock:** the OLED model dock with LAN port, 103.7 mm high, 174 mm long and 47.4 mm deep, 286 g (ChargerLAB's caliper and scale measurement; Nintendo rounds it to about 4.1 x 6.9 x 2.0 in, 0.69 lb), standing upright on its base as sold, the slot open in its top, with its AC adapter plugged in, an HDMI cable to a display, and the console awake; "seated" is then read off the display showing the console's picture, or the dock's LED.
- **Start:** the console screen down on two fixed rest blocks 150 mm high, a staging rest standing in for the hand or shelf the console comes from, at a height where both hands reach its top edge over the upright dock (any firm blocks, tops covered with a microfibre cloth), 66 x 30 mm, one under each Joy-Con, its top edge overhanging them by 30 mm toward the robot; the dock on the bench 16 cm closer to the robot, its front toward the robot.
- **Protection:** the screen under its own film or a microfibre sleeve; the gripper pads in nitrile finger cots or with a soft pad cover.
- **Kit and cost:** the Switch OLED model comes with its dock, AC adapter and HDMI cable (about $350 at its US launch in October 2021; a used or refurbished unit serves the same); a display with an HDMI input; two firm blocks 150 mm high and a microfibre cloth; nitrile finger cots (a few dollars a box); masking tape for the dock's outline; a steel ruler (a few dollars) for the outline and, stood behind the console's top edge in a side phone video, for how far it falls once let go (no more than 3 mm onto the plug is lowered, matching the simulator's 0.25 m/s). Any phone's level app reads the console's tilt.
- **Reset:** lay the console back screen down on its rest and the dock on its tape outline, about 30 seconds (estimate); nothing is consumed.
- **Forces:** the console weighs 420 g, shared by both hands (a 35 N grip per finger in the demo, within the YAM's roughly 40 N); the plug resists with 8 N in the simulator, inside USB Type-C's 5 to 20 N mating range (see the table); the console's 4.1 N weight gives about half and the hands push the rest, within the arms' roughly 20 N push each.

Each value is marked sourced, or estimate with its honest range; every estimate has a cheap real check.

| Value | In the simulator | Sourced or estimate | How to measure it on the real setup |
|---|---|---|---|
| Console size and mass | 102 x 242 x 13.9 mm, 420 g | Sourced: Nintendo's specs | A tape and a kitchen scale |
| Dock size and mass | 103.7 x 174 x 47.4 mm, 286 g | Sourced: ChargerLAB's teardown, measured (Nintendo: about 4.1 x 6.9 x 2.0 in, 0.69 lb) | A tape and a kitchen scale |
| Slot width | 16 mm, so 1.05 mm a side round the console's 13.9 mm | Estimate, range 14.5 to 17 mm: no published figure or teardown measurement (iFixit tore down the OLED console, not its dock; ChargerLAB's dock teardown gives none). It must clear the 13.9 mm console | Digital calipers across the empty slot (about $20) |
| Slot depth | 45 mm | Estimate, range 35 to 55 mm, from photos of docked consoles | The calipers' depth rod to the slot's floor |
| Plug | 8.25 x 2.4 mm, 6.5 mm tall, rigid | Sourced width and thickness (USB Type-C); height an estimate (the receptacle it enters is 6.2 mm deep). The real plug sits on springs (ChargerLAB: "the inside of the USB-C connector is supported by springs"), so it floats a little; the float is not published | Calipers; the plug's sideways play with a feeler gauge (about $10) |
| Port pocket | 9.25 x 3.4 x 6.6 mm: 0.5 mm a side round the plug | Estimate, range 0.1 to 0.7 mm a side. The USB-C receptacle's opening is 8.34 x 2.56 mm (Wikipedia, USB-C), under 0.1 mm a side; the simulator's rigid plug gets 0.5 mm instead, standing for the real plug's spring float plus the lead-in round the console's port | Feeler gauge on the plug's play, calipers on the port's lead-in |
| Seating force | 8 N of friction while the plug is in the port | Sourced range, value estimated: USB Type-C insertion force 5 to 20 N (EIA-364-13, as makers' datasheets give the spec's figure, e.g. Amphenol, Lumberg); extraction 8 to 20 N new and 6 to 20 N after 10,000 cycles (USB-IF ECN to spec section 3.8.1.2), a limit the spec waives "when the connectors are used in a mechanical docking application". 8 N is inside the range; both ends are run above | A 50 N digital push-pull gauge (about $90 to $150) on the console's top edge as it seats, then pulled back out |
| Friction, console in slot, on bench | 0.4 | Estimate, range 0.2 to 0.6 (plastic on plastic) | A tilt test |

## Modelled, not sourced from CAD

No public CAD of the OLED console or its dock under a redistributable licence was found (searched Printables, Thingiverse, MakerWorld, CGTrader, Sketchfab and Thangs: the free ones are replacement or compact docks of other sizes, the console models are for sale or restricted). Both are built from Nintendo's published sizes as boxes with the slot, plug and port pocket; the screen glass (7-inch 16:9, 154.9 x 87.2 mm) is drawn on the console's face at its published size.

## Safety for a real trial

Real trial: 1.0 is the TV showing the console's picture (the dock's AC adapter plugged in, an HDMI cable to a display, the console awake) or the dock's LED lit, the console upright by eye, the dock not moved; the speed (0.25 m/s) and tilt (2 degrees) limits are simulator checks. To protect the plug, carry the console down inside the slot onto the plug and push it home; never let it fall onto the plug (the simulator's 0.25 m/s limit is the speed of a 3.2 mm fall). Check the plug's sideways play with the feeler gauge every 10 trials; a bent or loose plug means a replacement dock, which is sold on its own.

Cover the screen and the pad faces as above before any grip.

Keep people clear of the arms while the robot runs and keep an emergency stop in reach. A console dropped from a height may damage the dock's plug.

## Sources

- Screen: 7.0-inch OLED, 16:9 (same Nintendo page).
- Nintendo Switch OLED model specs (102 x 242 x 13.9 mm with Joy-Con): https://www.nintendo.com/sg/hardware/switch/modal/specs/body_oled.html
- Console weight, 0.93 lb with Joy-Con: https://bulbapedia.bulbagarden.net/wiki/Nintendo_Switch_(OLED_model)
- Dock about 4.1 x 6.9 x 2.0 in, about 0.69 lb: https://www.nintendo.com/en-ca/switch/tech-specs/
- USB Type-C plug 8.25 x 2.4 mm: USB Type-C Cable and Connector Specification, USB-IF.
- ChargerLAB, "Teardown of Original Nintendo Switch OLED Charging Dock", https://chargerlab.com/teardown-of-original-nintendo-switch-oled-charging-dock: length 17.4 cm, height 103.69 mm, thickness 47.43 mm, weight 286 g; "There is a USB-C port in the center of the slot"; "The inside of the USB-C connector is supported by springs". Checked on 19 Nov 2026.
- USB-IF, USB Type-C ECN "Extraction force reduction" (to Release 1.0, section 3.8.1.2): "within the range of 8 N to 20 N before up to 10,000 mating cycles and within the range of 6 N to 20 N after ... This requirement does not apply when the connectors are used in a mechanical docking application." https://docs.ampnuts.ru/eevblog.docs/Standards/USB%203.1/usb_31_030215/USB%20Type-C/USB%20Type-C%20ECNs/USB%20Type-C%20ECN%20Extraction%20force.pdf
- USB Type-C insertion force 5 to 20 N (EIA-364-13, 12.5 mm a minute), as the spec's figure in makers' datasheets: Amphenol USB Type-C connector system, https://www.mouser.bg/new/amphenol/amphenol-usb-type-c-connector-system; Lumberg 2436 02, https://lumberg.com/en/products/product/2436+02.
- USB-C receptacle opening "8.34 mm x 2.56 mm, 6.20 mm deep": Wikipedia, USB-C, https://en.wikipedia.org/wiki/USB-C. Checked on 19 Nov 2026.
- iFixit, Nintendo Switch OLED Model teardown (console only, no dock measurements): https://www.ifixit.com/News/53272.
- Push-pull gauge price: 50 N digital gauges listed at about $90 to $150 (e.g. eBay listing 296834577499, $91.50, 19 Nov 2026).
- Nintendo, "How to Connect Nintendo Switch to a TV", https://en-americas-support.nintendo.com/app/answers/detail/a_id/22372/~/how-to-connect-nintendo-switch-to-a-tv: "Position the console so that the console screen faces in the same direction as the front panel on the dock", then insert the console into the dock. Checked on 19 Nov 2026.
- Nintendo IR, Dedicated Video Game Sales Units, https://www.nintendo.co.jp/ir/en/finance/hard_soft/index.html: Nintendo Switch hardware 157.20 million units life to date, as of 30 Sep 2026.
- Tom's Hardware, "Nintendo Announces OLED Switch, $349 in October", https://www.tomshardware.com/news/nintendo-switch-oled-model-price-specs-release-date: US launch price.

Nintendo Switch is a trademark of Nintendo. This task is not made, endorsed or sponsored by Nintendo.

## Licence

MIT
