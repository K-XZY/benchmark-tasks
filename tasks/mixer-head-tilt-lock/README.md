# Tilt a stand mixer's head back: hold its sprung lock lever up, lift the 4.5 kg head to its stop, let the lever lock it

A tilt-head stand mixer at the published size of the KitchenAid Classic K45SS (353 mm high, 221 wide, 358 deep; 439 mm high with the head up) stands on the bench with its head down and locked, no bowl or beater fitted. The robot tilts the head back the way the maker's manual gives it: hold the locking lever up at UNLOCK, tilt the head back, and lock it up. The lever springs back to LOCK when it is let go, and its pin drops into the hinge when the head is at either stop.

**What this task adds.** In the model the head's centre of mass (an estimate) lies ahead of its hinge through the whole tilt to the up stop, about 31 degrees (derived from the published 439 mm head-up height), so a head that is let go before it locks falls forward onto the bowl clamp; on a real mixer this follows from that estimate and is checked in the real trial below. One hand must hold the sprung lever up on the column's side until the head is off its pin, while the other lifts the 4.5 kg head by its hub, about 15 N at the start, along the hinge's arc to the stop, and keeps hold of it until the pin is in.

**Who does it, and how often.** 43% of US consumers report owning a stand mixer (AHAM, 2018), and KitchenAid alone was making two million stand mixers a year by 2012. On a tilt-head mixer the head is tipped back at every bowl or beater change: KitchenAid's K45 manual starts its "To attach bowl" and "To attach flat beater, wire whip, or dough hook" steps with "Hold the locking lever in the UNLOCK position and tilt motor head back." Sources below.

**How the demo does it.** The right hand pinches the lever tab by its two ends and raises it to its UNLOCK stop, closed on the lever's own travel. The left hand pinches the attachment hub across its sides, jaws along the hinge's axis, so the hub turns between the fingers as the head tilts. It carries the hub up the hinge's arc in short task-space presses (20 N set point), each along the arc's tangent at the head's measured angle, reading the angle again after each. Once the head is 4 degrees up, off the pin, the right hand lets the lever go; the left hand takes the head on to its stop, waits for the pin, lets go and backs away.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder, the scripted demo (the README command):

```bash
python demokit.py task.py OUT --mode full                                              # the full-score reference, filmed (640 x 360)
python demokit.py task.py OUT --no-record --mode none,nolever,leveronly,nolock,letgo,undo,restcolumn,resthead   # failure checks, below
python demokit.py task.py OUT --mode full --jitter 0.01 --seed 1                       # jittered run
python demokit.py task.py OUT --mode full --size 320x176 --stills 100                  # at the policy's camera size; setup.png (top camera, step 0) and a picture from each camera every 100 steps
python task.py --sizes                                                                 # the size check
```

`MX_TRACE=1` prints the head angle, lever travel, pin, tier and hands-clear distance every 20 steps. The demo reads the mechanism's joint angles (head hinge, lever slide) from the simulator, as `demokit` allows; a policy gets the cameras only.

**Result** (scripted, GPU physics, 20 Hz, on RTX 4070 Ti SUPER lab PCs and an RTX 5090): 391 steps (19.6 s of simulated time) to 1.0 on the lab PCs and 497 steps (24.9 s) on the RTX 5090, against a limit of 1,200.

| Run | Setting | Score |
|---|---|---|
| README command, 3 runs, RTX 4070 Ti SUPER | no jitter, 640 x 360, filmed | 1.0, 1.0, 1.0 (391 steps each) |
| README command, RTX 5090 | no jitter, 640 x 360, filmed | 1.0 (497 steps) |
| README command, RTX 5090 | no jitter, 320 x 176 (`--size 320x176`), filmed | 1.0 (497 steps) |
| Dream-1 (the model, 1,200 steps), RTX 4070 Ti SUPER | no jitter | 0 (hit the step limit) |
| Jittered, 2 runs, RTX 4070 Ti SUPER | `--jitter 0.01 --seed 1`, `--seed 2` | 1.0, 1.0 (391 steps each: the same count as unjittered, so the jitter does not appear to move the standing mixer) |

The films have no black frames on any camera (every frame of each camera read for brightness), and are written with faststart.

| Failure check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 |
| `nolever` | the left hand lifts by the hub; the lever is never touched | 0 (the pin holds the head down) |
| `leveronly` | the right hand holds the lever up; the head is not lifted | 0 |
| `nolock` | the lever is held up the whole time, and the head held at its stop by the hand to the end | 0.5 (at the stop, not locked) |
| `letgo` | the lever let go once the head is off the pin, the head let go at 20 degrees | 0 (in the model it falls onto the bowl clamp: a drop) |
| `restcolumn` | as `full`, but the right hand lets the lever go and rests its fingertips on the column's side, 5 cm above the tab | 0.5 (locked up, a fingertip within 3 cm of the mixer) |
| `resthead` | as `full`, but the left hand, off the hub, rests its fingertips on the head's front face above the hub | 0.5 (locked up, a fingertip within 3 cm of the mixer) |
| `undo` | as `full`, then (the run is not ended at the 1.0) the lever is pushed up again and the head lowered gently back down by the hub | 0 (the score and success follow the end state) |

**Force against the arm's limits.** At the hub the head needs 15.3 N square to its arc at 0 degrees, falling to 7.7 N at the stop; it is over the YAM's 1.5 kg (14.7 N) payload only in the first 3 degrees. The payload is rated at full reach (60 cm, a 8.8 N m moment at the shoulder); the hub is 40 cm (in plan) from the left arm's base at the start, where 15.3 N makes 6.1 N m. The demo lifts with task-space presses of 20 N set point (the YAM's push) along the arc's tangent. The lever needs 5 N; the right hand pinches it at 40 N a finger, within the gripper's figure.

## Sizes

Output of `python task.py --sizes`:

```
size                            scene  published  source
height, head down               353.0      353.0  listings, 13.9 in
width                           221.0      221.0  listings, 8.7 in
depth, base rear to hub         358.5      358.0  listings, 14.1 in
height, head up                 439.0      439.0  KitchenAid, tilt-head models, 17.3 in
hub diameter                     50.0       50.0  estimate from photos
up stop (deg)                    30.8        nan  derived: head-up height
hinge to hub (mm)               300.8        nan  model
head 4.5 kg, centre of mass [  0.  101.7  68. ] mm from the hinge (y ahead, z above); at the hub, lift 15.3 N at 0 deg
```

With the estimated centre of mass, the modelled head's weight pulls it forward at every angle up to the stop, so in the model it stays up only when the pin holds it.

| Size | Scene | Source |
|---|---|---|
| Height, head down | 353 mm | retailer listings for the K45SS ([Leon's](https://www.leons.ca/products/kitchenaid-white-classic-series-45-quart-stand-mixer-k45sswh), [Best Buy](https://www.bestbuy.com/product/kitchenaid-classic-series-4-5-quart-tilt-head-stand-mixer-k45ssob-onyx-black/J3KHV3FPXS)): 13.9 in |
| Width | 221 mm | listings: 8.7 in |
| Depth, base rear to hub | 358 mm | listings: 14.1 in |
| Height, head up | 439 mm | KitchenAid's figure for its 5-quart Artisan tilt-head ([listing](https://www.kitchenaid.com/p.KSM150PSPT.html), quoted at [KitchPrep](https://kitchprep.com/?p=83094)): 17.3 in |
| Up stop | 30.8 degrees | derived: the angle at which the modelled head reaches 439 mm |
| Mass | head 4.5 kg, stand 5 kg | estimates: the [5K45SS product sheet](https://cdn.shopify.com/s/files/1/0716/9140/3576/files/5K45SS.pdf) gives 10.5 kg net with bowl and tools |
| Head shape | 198 mm wide, 180 mm high, 320 mm long, rounded | from maker photos at the listed outer size |
| Hinge pin | 115 mm behind the head's middle, 68 mm below its axis | estimate from photos |
| Head centre of mass | 102 mm ahead of the hinge, 68 mm above | estimate: the centroid of the modelled head |
| Attachment hub | 50 mm across, 40 mm proud | estimate from photos |
| Lever tab | 25 x 12 mm, 18 mm proud, 10 mm travel, 5 N spring | estimates |

## Building it for a real trial

- **Mixer:** a KitchenAid Classic 5K45SS tilt-head mixer, the model whose manual gives the up-lock (new from about USD 280, used from about USD 120: estimates from retail listings). Check before buying, because the up-lock is not on every tilt-head: KitchenAid's current K45 manual says locking the head up is "available on select models", and KitchenAid's help page for some tilt-head models says "The tilt-head mixer only locks in the down position". The 5K45 manual describes the lever the task models: "Once lifted, the lever will automatically go back in LOCK position to keep the head lifted." A model whose lever stays where it is slid is worked the same way, with the lever moved back to LOCK by hand at the top.
- **Bench:** the mixer stands on its rubber feet; no clamp.

| Item | Use | Price | Resolution |
|---|---|---|---|
| Wixey WR300 digital angle gauge ([Trick-Tools](https://www.trick-tools.com/WR300-T2-Wixey-Digital-Angle-Gauge-Inclinometer-Level-7997)) | head angle: up, at the stop, the push check | USD 29.99 | 0.1 degree ([Woodcraft](https://www.woodcraft.com/products/wixey-digital-angle-gauge-with-backlight-type-2-wr300)) |
| Steel ruler | lever tab at LOCK (under 2 mm) | about USD 5 (estimate) | 1 mm |
| 1 kg weight on a loop of string (a bag of sugar checked on a kitchen scale) | the 10 N push check at the hub | about USD 2 (estimate) | |
| 30 mm wooden block | hands-clear gauge | scrap (estimate) | |
| Phone with 240 fps slow motion | a head meeting a stop faster than 120 degrees a second (0.5 degree a frame) | owned | 1/240 s |
| Masking tape | marks round the feet (slide over 2 cm, tip over 5 degrees) | about USD 3 (estimate) | 1 mm with the ruler |
| Luggage scale, spring scale | optional: head mass and lever spring, the estimates below | about USD 10 each (estimate) | 0.01 kg |

The tools cost under USD 60 besides the mixer and the phone.

**Reset, about 30 s (estimate), nothing consumed.** (1) Hold the lever at UNLOCK. (2) Lower the head gently by the hub until it is fully down ("Push the locking lever to UNLOCK and gently bring the head down", 5K45 manual). (3) Let the lever return to LOCK and check that the head will not lift. (4) Set the feet back on their tape marks.

| Value | In the simulator | Real-trial check |
|---|---|---|
| Head mass | 4.5 kg | a luggage scale on the hub, head unlocked, read at level: hub force x 0.30 m / (0.10 m x 9.81) |
| Lift at the hub | 15.3 N at 0 degrees, 7.7 N at the stop | the luggage scale on the hub as the head is raised |
| Lever spring | 5 N at 10 mm | a spring scale on the tab |
| Up stop | 30.8 degrees | the digital angle gauge on the head's top, head down and head up |

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. The scorer reads the head's hinge angle and speed, the lever's travel, the mixer's place and tilt, and every fingertip of both hands; the push check is a force it applies at the hub.

## Simplifications, stated

- The lock is a rule, not a modelled pin: it holds the hinge where it is while the lever is under 7 mm up and the head is within 1 degree of the down stop or 0.5 degree of the up stop, and frees it when the robot's pinch raises the lever 7 mm or more.
- The mixer and its head are built from the published outer sizes and maker photos (no vendor CAD is used or distributed). The head is a rounded solid of revolution, its hub a plain cylinder, its centre of mass the centroid of that solid (an estimate).
- No bowl or beater is fitted: tilting the head is how a bowl or beater is fitted.
- Whether a real unlocked head, let go at its up stop, falls forward or rests up on its own is not sourced and not yet measured: it follows from the estimated centre of mass. The manuals lock the head up "to keep the head lifted" (5K45) and give a separate "To lock motor head in raised position" step (K45), which suggests it does not stay up unaided. Check on the bought mixer before the first trial: unlock it at the stop with a hand under the hub, and let go slowly. If it rests up, letting go before the pin is in costs nothing on that unit, and the 1.0 condition (lever at LOCK, pin in, the 10 N push held) is unchanged.

## Safety for a real trial

A dropped head lands hard on the bowl clamp and can pinch fingers between the head and the column. Keep people clear of the arms' reach while the robot runs, unplug the mixer, and keep an emergency stop in reach. No food, liquid or powder is used, so nothing reaches the grippers; the fingers grip the chrome hub and the plastic lever tab only.

## Trademarks and sources

KitchenAid and K45SS are trademarks of Whirlpool Corporation, used here only to name the product whose published dimensions and tilting steps the task follows; this task is not affiliated with or endorsed by Whirlpool. Sizes: K45SS retailer listings, 13.9 x 8.7 x 14.1 in (https://www.leons.ca/products/kitchenaid-white-classic-series-45-quart-stand-mixer-k45sswh, https://www.bestbuy.com/product/kitchenaid-classic-series-4-5-quart-tilt-head-stand-mixer-k45ssob-onyx-black/J3KHV3FPXS); the 5K45SS product sheet, 10.5 kg net (https://cdn.shopify.com/s/files/1/0716/9140/3576/files/5K45SS.pdf); KitchenAid's 5-quart Artisan tilt-head listing, 17.3 in with the head raised (https://www.kitchenaid.com/p.KSM150PSPT.html, as quoted at https://kitchprep.com/?p=83094). Steps: KitchenAid's K45 / KSM45 instructions, "To lock motor head in raised position" ("Push locking lever up into UNLOCK position and hold. Tilt motor head back. Move locking lever into LOCK position.") and "To attach bowl" ("Hold the locking lever in the UNLOCK position and tilt motor head back."), https://www.caterkwik.co.uk/shop/downloads/K45_kitchenaid%20ksm45en%20instruction%20manual.pdf. The lever's return: the 5K45 manual ("Once lifted, the lever will automatically go back in LOCK position to keep the head lifted"; "Push the locking lever to UNLOCK and gently bring the head down"), https://manualslib.com/manual/2802035/Kitchenaid-5k45.html?page=9. The up-lock on select models: KitchenAid's current K45SS use-and-care guide ("Locking option is available on select models"), https://api.whirlpoolcentral.ca/_resources/documents/K45SSWH_use_and_care_en.pdf, and KitchenAid's help page ("The tilt-head mixer only locks in the down position"), https://producthelp.kitchenaid.com/Countertop_Appliances/Stand_Mixers/Tilt_Head_Mixer/Operation/Head_Will_Not_Lock_or_Unlock_-_Tilt-head_Mixer. Ownership: AHAM, "All Mixed Up: 6 Facts about Stand Mixers", 13 August 2018 ("Today, 43% of consumers report owning a stand mixer"), https://blog.aham.org/?p=4025. Production: Whirlpool, "KitchenAid Announces Two Million Stand Mixer Milestone", 12 December 2012 (an annual production of two million stand mixers, all built in Greenville, Ohio), https://whirlpool.mediaroom.com/2012-12-12-KitchenAid-Announces-Two-Million-Stand-Mixer-Milestone. Tools: Wixey WR300 at USD 29.99 (https://www.trick-tools.com/WR300-T2-Wixey-Digital-Angle-Gauge-Inclinometer-Level-7997), 0.1 degree resolution (https://www.woodcraft.com/products/wixey-digital-angle-gauge-with-backlight-type-2-wr300).

## Licence

MIT. Copyright (c) 2026 the task's authors.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: the above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
