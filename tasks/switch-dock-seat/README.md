# Dock a Nintendo Switch: lower the console straight down its dock's slot until its port seats on the hidden USB-C plug

Most people with a Nintendo Switch put it back in its dock to play on the TV or to charge it. The console goes straight down the dock's slot until the USB-C plug at the bottom, out of sight, enters the port on the console's lower edge. The slot is about 1 mm wider than the console on each side and the plug is an 8.25 x 2.4 mm tongue, so the console has to come down upright and centred, then take a firm push over the last 6 mm.

**What this task adds.** Plug-in tasks push a held plug into a visible port. Here the port is on the 242 mm wide, 420 g console and the plug is fixed and hidden at the bottom of a slot. The console starts screen down on a rest with its top edge overhanging, so one jaw closes on its 13.9 mm top edge; it is stood up by one 90 degree wrist pitch and lowered.

**How the demo does it.** The right hand takes the top edge (jaws up and down across 13.9 mm, 35 N a finger), lifts, carries the console level over the dock, pitches it 90 degrees so it hangs upright with its screen toward the robot, then steers it by its own pose: the demo reads the console's pose from the simulator (a policy gets only the cameras), levels it, centres it on the plug and lowers it at 10 mm/s, then pushes the last millimetres.

**Result** (scripted, GPU physics, RTX 4070 Ti SUPER, 20 Hz): 1.0 in 500 steps (25 s): port 6.4 mm onto the plug (stop at 6.6), tilt under 1 degree, plug within 0.3 mm of the port's axis. Lazy-route checks, scored by the same function:

| Check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 |
| `drop` | let go 8 cm above the plug | 0 (it fell and did not stay upright in the slot) |
| `rest` | let go 3 mm above the plug | 0 (the hands stayed within 3 cm of the console; resting unseated is capped at 0.5 by the scorer) |
| `crooked` | lowered with a 10 degree roll across the width | 0: the console jammed tilted (10.6 degrees), the push shifted the dock more than 2 cm; the tilt gate (2 degrees) is in the scorer |

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `taskspace.py` are in this folder.

## Run a trial

```bash
python demokit.py task.py OUT --mode full                                    # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode none,drop,rest,crooked       # lazy-route checks
```

## Building it for a real trial

- **Console:** a Nintendo Switch OLED model with both Joy-Con attached, 102 x 242 x 13.9 mm, 420 g.
- **Dock:** the OLED model dock with LAN port, about 104 x 175 x 51 mm, 313 g, with its AC adapter plugged in, an HDMI cable to a display, and the console awake; "seated" is then read off the display showing the console's picture, or the dock's LED.
- **Start:** the console screen down on a fixed rest 120 mm high (any firm block), its top edge overhanging the rest by 30 mm toward the robot; the dock on the bench 16 cm closer to the robot, its front toward the robot.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Console size and mass | 102 x 242 x 13.9 mm, 420 g | A tape and a kitchen scale |
| Slot width, depth | 16 mm, 45 mm (estimates) | Calipers across the empty slot and a depth gauge to its floor |
| Plug size | 8.25 x 2.4 x 6.5 mm (USB Type-C plug size; height estimated) | Calipers |
| Port pocket | 9.25 x 3.4 x 6.6 mm (estimates) | Calipers on the console's lower edge |
| Seating force | 8 N over the last 6 mm (estimate) | A push gauge on the console's top edge |
| Friction, console in slot, on bench | 0.4 (estimates) | A tilt test |

## Modelled, not sourced from CAD

No public CAD of the OLED console or its dock under a redistributable licence was found (searched Printables, Thingiverse, MakerWorld, CGTrader, Sketchfab and Thangs: the free ones are replacement or compact docks of other sizes, the console models are for sale or restricted). Both are built from Nintendo's published sizes as boxes with the slot, plug and port pocket.

## Safety for a real trial

Real trial: 1.0 is the TV showing the console's picture (the dock's AC adapter plugged in, an HDMI cable to a display, the console awake) or the dock's LED lit, the console upright by eye, the dock not moved; the speed (0.25 m/s) and tilt (2 degrees) limits are simulator checks. To protect the plug, lower the console slowly and never let go above the slot.

Keep people clear of the arms while the robot runs and keep an emergency stop in reach. A console dropped from a height may damage the dock's plug.

## Sources

- Nintendo Switch OLED model specs (102 x 242 x 13.9 mm with Joy-Con): https://www.nintendo.com/sg/hardware/switch/modal/specs/body_oled.html
- Console weight, 0.93 lb with Joy-Con: https://bulbapedia.bulbagarden.net/wiki/Nintendo_Switch_(OLED_model)
- Dock about 4.1 x 6.9 x 2.0 in, about 0.69 lb: https://www.nintendo.com/en-ca/switch/tech-specs/
- USB Type-C plug 8.25 x 2.4 mm: USB Type-C Cable and Connector Specification, USB-IF.

Nintendo Switch is a trademark of Nintendo. This task is not made, endorsed or sponsored by Nintendo.

## Licence

MIT
