# Slide a turner under one egg inside a 65 mm deep holding tray, cover held clear, set it on the muffin and re-cover

A round cooked egg lies in a polycarbonate holding tray 65 mm deep, with only about 30 mm between its rim and the wall. The left hand lifts the tray's cover off and holds it clear; the right hand brings a stainless turner down steeply over the wall, flattens it to lie on the floor, slides it under the egg's 3 mm rounded edge, carries the 55 g egg about 32 cm level on the blade and slides it onto the centre of a toasted muffin heel. The cover then goes back onto the tray, seated, with the second egg still in it.

## Run

From this folder, with the harness's Python:

```
python demokit.py task.py OUT --mode full                 # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,grab_egg,pinch_fast,drop_high,lid_counter,slide_lid,lid_off
```

Set `FFMPEG` to the venv's imageio_ffmpeg binary first (the system ffmpeg has no libx264).

**Result** (scripted, GPU physics, 20 Hz): task_score 1.0, 631 steps (31.55 s), RTX 3090 Ti; egg 8.2 mm off the heel's centre, flat; cover seated (2.8 mm off its mark across against the 8 mm limit, height within 2 mm, under 1 degree); tray moved 0.0 mm.

**Dream-1 run**: task_score 0.0 (1,200 steps, 60 s, one session, 20 rows per request, one 32 GB GPU).

## Scoring, and the routes that score less

One tier per key substep (egg out of the tray on the blade, egg on the heel, cover back on the tray), plus failures that void the run. Tiers read the end state only.

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | the left hand lifts the cover by its grip bar and holds it level clear of the tray; the right hand takes the turner by its handle, slides the blade under the near egg, flattens and lifts, carries it back over the wall and over the heel, tips the blade so the egg slides onto the heel, draws the blade out; the cover is lined up over the tray by its seen place and lowered | 1.0 |
| none | nothing | 0.0 |
| grab_egg | cover held clear as in full; the egg pinched out by the fingers and set on the heel; cover put back | 0.0 (the fingers cannot take it out: the tray lifts with it and the egg never leaves) |
| pinch_fast | same, with the carry sped up | 0.0 |
| drop_high | full, but the egg tipped off the blade 10 cm above the heel | 0.0 (slid off from more than 3 cm above the heel's top) |
| lid_counter | cover set down on the counter | 0.0 (void) |
| slide_lid | cover slid half off along the rim, never lifted | 0.0 (egg never comes out) |
| lid_off | egg served as in full, cover never put back | 0.6 |

Egg out of the tray with the cover not held clear, the cover set down on the counter, an egg touching the counter and the other egg leaving the tray void the run. The egg must go tray, blade, heel: an egg that skips the blade, is off all three for more than 0.5 s, or slides off the blade from more than 3 cm above the heel's top scores 0. All checks are made in the tray's own frame, because the tray is a free 0.36 kg body on the counter.

Real trial: the same rules, read from a fixed overhead phone camera (a few tens of dollars) over a printed target mark under the muffin for the 12 mm centre figure, and a second phone camera at counter height beside the muffin and tray, filming a vertical ruler tape, for the 3 cm slide-off height and the cover's 10 mm lift; the cover's 2 mm seat is checked with a 2 mm feeler gauge along the rim (about $10), and the egg's flatness with a phone level app laid on a card on the egg (resolution 0.1 degree).

## Why it matters

Fast-food kitchens hold cooked egg in covered trays and serve it onto sandwiches one at a time. 641,070 people work as Cooks, Fast Food and 3,854,050 as Fast Food and Counter Workers (BLS Occupational Employment and Wage Statistics, May 2025, https://www.bls.gov/oes/current/oes352011.htm and https://www.bls.gov/oes/current/oes353023.htm), a total of 4,495,120. O*NET lists 'Cook and package batches of food, such as hamburgers or fried chicken, prepared to order or kept warm until sold' as a Core task of Cooks, Fast Food (35-2011.00, https://www.onetonline.org/link/summary/35-2011.00).

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Tray | 325 x 176 mm at the rim, 296 x 150 mm inside, 65 mm deep, 3 mm walls and floor, 0.36 kg | 1/3-size pan, 2.5 in (65 mm) deep | Cambro 32CW135 product page (burkett.com/products/cambro-1-3-size-2-1-2-deep-clear-food-storage-pan); mass estimated |
| Cover | 325 x 176 x 3 mm, 6 mm skirt, grip bar 60 x 25 x 30 mm, 0.22 kg | flat lift-off cover with moulded centre grip | Vollrath 31300 product page (webstaurantstore.com/vollrath-31300-1-3-size-clear-solid-cover/92231300.html); mass from the maker |
| Egg | 89 mm across, 12 mm thick, 3 mm rounded edge, 55 g, friction 0.25 | round cooked egg patty | estimate |
| Turner | blade 102 x 127 mm, 2.5 mm thick bevelled to 0.6 mm at the tip, handle 130 x 22 x 14 mm 65 mm above the blade, 0.17 kg, friction 0.25 | 5 x 4 in blade solid offset turner | Mercer M18480 product page (webstaurantstore.com/turner-5x4-turner-rosewoo/470M18480.html); mass estimated |
| Muffin heel | 95 mm across, 22 mm high, 30 g, friction 0.9, on an 18 x 18 cm wrapper | toasted English muffin half | estimate |
| Layout | tray centred 40 cm in front of the base midpoint, muffin heel 36 cm in front and 30 cm to the right, about 32 cm from the egg's start | | scene choice, within the arms' reach |

## Colliders

| Body | Colliders | Free or fixed |
|---|---|---|
| Tray | floor and four wall boxes | free body |
| Cover | top plate, skirt strips, grip bar | free body |
| Eggs | one rounded disc each | free, friction 0.25 |
| Turner | blade, neck and handle boxes | free |
| Muffin | cylinder on the wrapper | free |
| Fingers | the harness's YAM finger colliders | the robot |

## What is modelled, and what is scripted

Everything the act touches is a free, colliding body held by friction and gravity: the tray, cover, eggs, turner and muffin. The egg is rigid (a real egg disc is firm enough to ride a blade); heat and steam are not modelled. The motion is scripted from the bodies' observed poses.

## Reset and consumables

Reset takes about 3 minutes: set the tray on the counter at its mark, lay the two egg patties on its floor at the far end (centres 41.0 and 50.2 cm in front of the base midpoint), seat the cover, lay the turner on the counter on its blade's back edge with its tip toward the tray, and set a fresh toasted muffin heel on the wrapper. Egg patty: a round cooked egg 89 mm across and 12 mm thick, about 55 g, cooled to room temperature (a food-replica disc of the same size is an equivalent); muffin heel: a plain English muffin half toasted to a dry crust, about 95 mm across and 30 g. The tray is wiped dry between trials.

## Robot figures the demo keeps within

Grip 30 N on the cover bar and 40 N on the turner handle (the YAM's figure is about 40 N a finger); payloads under 0.25 kg; both work points within about 0.45 m of the bases. Run: 631 of 1,200 steps.

## Real trial

A 1/3-size polycarbonate pan with its cover, a 5 x 4 in stainless offset turner and a toasted English muffin are common restaurant-supply items (about $40 in all). The egg patty is a cooked round egg or a food-replica disc of the same size. Pad covers are not needed: the objects are light and none can shatter.
