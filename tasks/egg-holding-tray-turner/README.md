# Slide a turner under an egg patty inside a 65 mm deep tray, set it on a muffin, and re-cover the tray

A cooked egg patty lies in a polycarbonate holding pan 65 mm deep whose walls draw in to a 267 x 121 mm floor, with 68 mm of flat floor free beside it. One hand takes the pan's cover off and holds it clear or sets it down off the pan; the right hand brings a stainless turner in over the wall tipped about 42 degrees, sets its tip on the floor at the patty's 3 mm rounded edge and slides it under, flattening the blade as its back end clears the near wall, carries the 42.5 g patty about 32 cm level on the blade and slides it onto the centre of a toasted muffin heel. The cover then goes back onto the pan, seated, with the second patty still in it.

## Run

From this folder, with the harness's Python:

```
python demokit.py task.py OUT --mode full                 # final film, 640 x 360, GPU to yourself
for M in lid_counter none grab_egg pinch_fast pinch_to_blade near_outline egg_edge_counter drop_high slide_lid lid_off blade_then_pinch fric_lo fric_hi fric_pan_lo fric_pan_hi fric_muf_lo fric_muf_hi shallow_wells deep_wells big_round thin_patty soft_patty; do
  python demokit.py task.py OUT --no-record --mode $M     # each mode in its own session: each scores one episode from a fresh scene, as a policy run does
done
```

Set `FFMPEG` to the venv's imageio_ffmpeg binary first (the system ffmpeg has no libx264).

**Result** (scripted, GPU physics, 20 Hz): task_score 1.0, 716 steps (35.8 s), RTX 3090 Ti; patty 6.3 mm off the heel's centre, flat; cover seated (height within 2 mm, under 1 degree, centre within 8 mm); pan moved 1.9 mm.

**Dream-1 run**: 0.0 over the 1,200-step (60 s) limit; the model reached none of the scored stages.

## Scoring, and the routes that score less

One tier per key substep (0.3 the patty on the blade, its centre carried 2 cm or more past the pan's 325 x 176 mm outline; 0.6 the patty on the heel; 1.0 the cover back on the pan), plus failures that void the run. Tiers read the end state only.

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | the left hand lifts the cover by the land between its two finger wells and holds it level beside the pan; the right hand takes the turner by its handle, brings the blade in tipped about 42 degrees with its tip at the near patty's edge, slides it under while flattening it, lifts, carries it back over the wall and over the heel, tips the blade so the patty slides onto the heel, draws the blade out; the cover is lined up over the tray by its seen place and lowered, and the closed fingers press its land down onto the seat | 1.0 |
| none | nothing | 0.0 |
| grab_egg | cover held clear as in full; the patty pinched out by the fingers and set on the heel; cover put back | 0.0 (in this run the fingers do not get the patty out: the pan lifts with it; a softer real patty might be pinched out, and the path rule voids that: a patty off pan, blade and heel for more than 0.5 s) |
| pinch_fast | same, with the carry sped up | 0.0 |
| drop_high | full, but the patty tipped off the blade 10 cm above the heel | 0.0 (slid off from more than 20 mm above the heel's top) |
| lid_counter | full, but the cover is set down flat on the counter beside the pan while the patty is served, then taken up and seated | 1.0 (746 steps) |
| pinch_to_blade | cover set on the counter; the blade held level just past the pan; the free hand pinches the patty out of the pan and lays it on the blade, which serves it | 0.0 (in this run the fingers do not get the patty out, as in grab_egg; check_score.py's case of the patty pinched out, laid on the blade and served scores 0: it left the pan not resting on the blade) |
| near_outline | patty scooped as in full and held on the blade with its centre about 1 cm past the pan's outline | 0.0 (not 2 cm past the outline) |
| shallow_wells | full with the cover's finger wells at the low end of their accepted depth, 12 mm | 1.0 |
| big_round | full with a 12 mm floor round, the top of its accepted range | 1.0 |
| deep_wells | full with the finger wells at the high end of their accepted depth, 20 mm | 1.0 |
| fric_pan_lo | full with the patty's friction on the pan floor at the low end (pair 0.20) | 1.0 |
| fric_pan_hi | full with the patty's friction on the pan floor at the high end (pair 0.50) | 1.0 |
| thin_patty | full with the patty at the thin end of its range, 7 mm | 1.0 |
| soft_patty | full with a bending patty (five strips on sprung hinges, a plate of modulus 27 kPa that sags about 20 mm half over a ruler's edge), the blade tipped to 20 degrees at the heel | 1.0 |
| fric_muf_lo | full with the patty's friction on the heel at the low end (pair 0.35), the blade tipped to 18 degrees at the heel | 1.0 |
| fric_muf_hi | full with the patty's friction on the heel at the high end (pair 1.0) | 1.0 |
| egg_edge_counter | full, but the blade set 7 cm further past the heel, so the patty slides off beside it onto the counter | 0.0 (a patty touching the counter) |
| slide_lid | cover slid half off along the rim, never lifted | 0.0 (the patty never comes out) |
| lid_off | patty served as in full, cover never put back | 0.6 |
| blade_then_pinch | cover set on the counter; the blade carries the patty over the heel, then the free hand reaches to lift it off the blade and set it on the heel | 0.0 (the heel is out of the free hand's reach, 16 cm short, so the patty falls off the drawn-back blade onto the counter; a lift off the blade over the heel by any hand is the check_score case "the fingers lift the egg 2 cm off it", 0) |
| fric_lo | full with the patty's friction at the low end of the accepted range (pair with steel 0.20) | 1.0 (715 steps) |
| fric_hi | full with the patty's friction at the high end (pair with steel 0.50), the blade tipped to 30 degrees at the heel (26 in full, 16 at the low end, where the patty slides more readily) | 1.0 (716 steps) |

The patty coming out of the pan while the cover is on the pan or resting on it, a patty touching the counter outside the pan (its lowest point within 4 mm of it) and the other patty leaving the pan void the run. The cover may be held anywhere with its lowest point (a plate corner or the bottom of a finger well) 10 mm or more above the rim, or held or set down anywhere wholly off the pan's outline, such as flat on the clean counter; it must be seated again at the end for 1.0. The patty must go pan, blade, heel: a patty that leaves the pan other than resting on the blade (its centre within 10 mm of the blade's outline and within 4 mm of the height of a patty lying flat on the blade's face), skips the blade, is off all three for more than 0.5 s, rises more than 4 mm once it has left the blade (lifted off it rather than slid down off it), or slides off the blade with its lowest point more than 20 mm above the heel's top scores 0. All checks are made in the pan's own frame, because the pan is a free 0.31 kg body on the counter. `python check_score.py` runs the scorer on 23 hand-built cases with no simulator: the cover held beside, held 15 cm over the pan, stood on its edge, set flat on the counter beside or in front (each 1.0 when seated after), left off (0.6), slid along the rim, leaned on the rim or lifted only 5 mm (each void), put back 8.5 mm off on the diagonal (0.6), and the path cases: carried by the fingers, pinched out and laid on the blade, held on the blade only 10 mm past the outline, lifted off the blade over the heel, slid off from 2.5 cm and from 8 cm (each 0).

Real trial: the same rules, read from a fixed overhead phone camera (a few tens of dollars) over a printed target mark under the muffin for the 12 mm centre figure and over a tape line on the counter 2 cm outside the pan's outline for the 0.3 tier (the patty's centre past the line), and a second phone camera at counter height beside the muffin and pan, filming a vertical ruler tape, for the 20 mm slide-off height and the cover's 10 mm lift; the cover's 2 mm seat is checked with a 2 mm feeler gauge along the rim (about $10), and the patty's flatness with a phone level app laid on a card on the egg (resolution 0.1 degree).

## Why it matters

Lifting a cooked egg patty onto a muffin with a turner is the familiar step at the heart of an egg breakfast sandwich, made at home and behind every breakfast counter. The breakfast sandwich is the top food at US fast-food breakfast: 37% of all fast-food breakfast orders in August 2022 included one (Circana, https://www.circana.com/post/breakfast-was-u-s-restaurant-industry-bright-spot-in-august). Americans spent more than $2.4 billion on breakfast sandwiches in the year ending September 2023 (NielsenIQ, reported by NACS, https://www.convenience.org/Media/Daily/2023/October/25/2-Breakfast-Sandwiches-Are-a-Cereal-Killer_FS). Kitchens keep cooked patties in a covered pan so that they stay moist, so the cover comes off for each patty and goes back on.

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Tray | 325 x 176 mm at the rim, 296 x 150 mm inside at the rim, walls drawn in 14.5 mm a side to a 267 x 121 mm floor, an 8 mm round at the floor, 65 mm deep, 3 mm walls and floor, 0.31 kg | 1/3-size pan, 2.5 in (65 mm) deep, 2.5 qt, 0.682 lb | Cambro 32CW135 product page (burkett.com/products/cambro-1-3-size-2-1-2-deep-clear-food-storage-pan; capacity from webstaurantstore.com/cambro-32cw135-camwear-1-3-size-clear-food-pan-2-1-2-deep/21432CWCL.html); draw-in from the capacity (Fidelity), floor round estimated |
| Cover | 325 x 176 x 3 mm, 6 mm skirt 1.5 mm clear of the walls, two finger wells 68 x 39 mm sunk 15 mm with a 15 mm land between them, 0.22 kg | flat lift-off cover with two moulded finger wells at its centre | Vollrath 31300 product page (webstaurantstore.com/vollrath-31300-1-3-size-clear-solid-cover/92231300.html); mass from the maker; wells and land measured on the maker's photo, scaled by the 324 mm cover length; well depth estimated |
| Egg patty | 89 mm across, 9 mm thick, 3 mm rounded edge, 42.5 g, friction 0.25 | 3 1/2 in round, 1.5 oz | Papetti's 3 1/2 in round fully cooked scrambled egg patty, 1.5 oz (webstaurantstore.com/papettis-3-1-2-round-fully-cooked-scrambled-egg-patty-1-5-oz-case/873MIC570225.html); thickness estimated |
| Turner | blade 127 mm long (tip to back) x 102 mm wide, 2.5 mm thick bevelled to 0.6 mm at the tip, handle 130 x 22 x 14 mm 65 mm above the blade, 0.34 kg, friction 0.25 | solid offset turner, blade 5 in long x 4 in wide, 12 1/2 in long, shipping weight 0.75 lb | Mercer M18480 product page (webstaurantstore.com/turner-5x4-turner-rosewoo/470M18480.html); mass set to the shipping weight, an upper bound |
| Muffin heel | 95 mm across, 22 mm high, 30 g, friction 0.9 (pair with the patty 0.575), on an 18 x 18 cm wrapper | toasted English muffin half; one Thomas' Original English Muffin is 13 oz for 6 (61 g), so a half is about 31 g | Thomas' Original English Muffins, 6 count, 13 oz (walmart.com/ip/16419108616); diameter and height estimated. Rigid stand-in if no muffins: a 95 mm x 22 mm disc of closed-cell foam or cork with a dry, rough face |
| Layout | tray centred 40 cm in front of the base midpoint, muffin heel 36 cm in front and 30 cm to the right, about 32 cm from the patty's start | | scene choice, within the arms' reach |

## Colliders

| Body | Colliders | Free or fixed |
|---|---|---|
| Tray | floor box, four drawn-in walls and four floor rounds (convex hulls), the flange | free body |
| Cover | top plate round the well openings, skirt strips, each well's floor and walls, the land between the wells | free body |
| Egg patties | one rounded disc each | free, friction 0.25 |
| Turner | blade, neck and handle boxes | free |
| Muffin | cylinder on the wrapper | free |
| Fingers | the harness's YAM finger colliders | the robot |

## What is modelled, and what is scripted

Everything the act touches is a free, colliding body held by friction and gravity: the pan, cover, patties, turner and muffin. The patty is rigid, except in soft_patty, where it bends on sprung hinges (see the stiffness row under Fidelity); heat and steam are not modelled. The motion is scripted from the bodies' observed poses.

## Fidelity

| Number or rule | Sim | Real | Source | Effect on difficulty |
|---|---|---|---|---|
| Pan depth | 65 mm | 2 1/2 in (65 mm) | Cambro 32CW135 | same |
| Pan inside | 296 x 150 mm at the rim, walls drawn in 14.5 mm a side to a 267 x 121 mm floor | 1/3 size, 325 x 176 mm outside, 2.5 qt (2.37 L) | bound, a shown calculation: a frustum 62 mm deep from 296 x 150 mm at the rim holds 2.37 L when the walls draw in 14.5 mm a side (13.3 mm for the 2.4 L of a GN 1/3 x 65 mm pan, araven.com); a pan whose listed capacity stops short of the brim draws in less | the sim is at the tightest end: less draw-in leaves more floor. Real check: a ruler across the floor, accepted 267 to 290 mm and 121 to 140 mm |
| Floor round | 8 mm (a 45 degree chamfer) on all four sides | rounded floor edge, no published figure (the maker's sheets call it a large-radius bottom) | estimate, bounded by the blade: the 102 mm blade must lie on the flat floor across the 121 mm floor, so a round much over 10 mm lifts its side edges onto the rounds and its tip off the floor | a larger round narrows and shortens the flat floor. big_round runs full with a 12 mm round and scores 1.0; at 15 mm the blade rides up the side rounds, pushes the patty along the floor and scores 0. Real check: a radius gauge in the floor corners, accepted 0 to 12 mm; a pan with a larger round is not used. A vertical corner round up to 25 mm stays clear of the patties and the blade |
| Free flat floor before the patty | 68 mm | | from the rows above and the patties at the far end, 3 mm apart | the blade, 127 mm long, cannot lie flat there: it comes in tipped about 42 degrees, the least that clears the near wall by 4 mm, and flattens as it slides under |
| Patty size and mass | 89 mm, 42.5 g | 3 1/2 in, 1.5 oz | Papetti's 1.5 oz patty | same |
| Patty thickness | 9 mm | not published; 42.5 g over an 89 mm disc (62 cm2) at a cooked-egg density of 0.75 to 1.0 g/cm3 gives 7 to 9 mm | estimate, bounded by that calculation | the blade goes under the edge at the floor; a thinner patty sits lower on it. Both ends score 1.0 (full at 9 mm, thin_patty at 7 mm). Real check: a ruler across the patty's edge, accepted 7 to 9 mm |
| Patty friction on steel | 0.25 (PhysX averages a pair: patty 0.25 and steel 0.25; patty on the pan 0.30) | 0.20 to 0.50 accepted | bound: a fat-filmed contact (steel on steel with lard 0.11, with light mineral oil 0.23) to a dry soft solid on steel (polymers 0.2 to 0.5), engineeringtoolbox.com/friction-coefficients-d_778.html | the real trial lays the patty on the turner's blade and tips the turner until it slides, reading the angle with a phone level app: tan of 11.3 to 26.6 degrees is 0.20 to 0.50; a patty that slides below 11.3 degrees (a heavy fat film) is blotted with a paper towel and tested again. The demo scores 1.0 at both ends (the blade tipped at the heel to 16 degrees for fric_lo and 30 for fric_hi, against 26 in full, as a cook tips it less for a slippery patty and further for a sticky one) |
| Patty friction on the pan floor | pair 0.30 (patty 0.25, polycarbonate 0.35) | 0.20 to 0.50 accepted | bound as for steel above: a fat-filmed contact to a dry soft solid on a hard plastic | the floor holds the patty while the blade's tip goes under its edge: a grippier floor holds it better, a slicker one lets it slide ahead of the tip. Both ends score 1.0 (fric_pan_lo, pair 0.20; fric_pan_hi, pair 0.50). Real check: the patty laid on the pan floor and the pan tilted until it slides, read with a phone level app, accepted 11.3 to 26.6 degrees |
| Patty friction on the heel | pair 0.575 (patty 0.25, toasted heel 0.90) | 0.35 to 1.0 accepted | estimate (no published value for egg on toasted crumb; the tilt test follows the sliding-angle method of Davison et al., Tribology International 2021, doi 10.1016/j.triboint.2020.106615): a toasted crumb is rougher than the blade's polished steel, so the range starts at the middle of the steel pair's range and runs to 1.0, where the patty sticks | both ends score 1.0 (fric_muf_lo, pair 0.35, with the blade tipped to 18 degrees at the heel; fric_muf_hi, pair 1.0). Real check: the patty laid on the heel and the heel tilted until the patty slides, read with a phone level app, accepted 19 to 45 degrees (tan 0.35 to 1.0) |
| Patty stiffness | rigid in every run but soft_patty; in soft_patty the patty is five strips across the blade's path on sprung hinges, each hinge E I / spacing for a plate of modulus E = 27 kPa | a cooked patty bends | bound, a shown calculation: a patty held level half over a ruler's edge sags at its free edge by 1.5 rho g L^4 / (E t^2) (a uniform strip bending under its own weight; rho = 0.76 g/cm3 from 42.5 g in an 89 x 9 mm disc, L = 44.5 mm, t = 9 mm), that is 540 Pa m / E: 20 mm at E = 27 kPa, 5 mm at 100 kPa | the rigid patty and the 27 kPa patty both score 1.0 (soft_patty, with the blade tipped to 20 degrees at the heel, as a bending patty lies closer on the blade and slides more readily), so the act holds from no sag to 20 mm. Real check before each session: the patty laid half over a ruler's edge, the drop of its free edge read on a second ruler held upright, accepted 0 to 20 mm; a patty that sags more is replaced |
| Turner blade | 127 mm long x 102 mm wide, 2.5 mm, 0.6 mm at the tip | 5 in long x 4 in wide | Mercer M18480 (size); thickness estimated, bounded by comparable stainless servers and spreaders listed at 1.0 to 2.0 mm (Browne catalogue); 2.5 mm sits above that range | thicker than the real blade: the patty climbs a taller step onto it, so the sim is harder. Real check: calipers on the blade at the tip bevel's end, accepted 1.0 to 2.5 mm (the 2.0 mm listed top, plus the 2.5 mm simulated); the scorer reads end state only and passes across that range |
| Cover | 0.22 kg, 6 mm skirt 1.5 mm clear of the walls | 0.22 kg; a drop-on cover that fits loosely over the rim | Vollrath 31300 (mass); skirt clearance estimated | a tighter real fit asks the cover to be lined up more closely before it drops in, so the real seat is somewhat harder. The 1.5 mm is a fit clearance for the contact solver: at 0.75 mm the pan lifts with the cover in simulation. Real check: feeler gauges between the skirt and the wall on each side with the cover centred; the real gap is recorded with each result; accepted 1.5 mm or more a side (up to 4 mm), the seat tolerance of 8 mm across being larger than that range |
| Cover grip | two finger wells 68 x 39 mm, 15 mm deep, a 15 mm land between | wells and land as measured | maker's photo scaled by the 324 mm cover; depth not published, estimated from the maker's photo and a side view of a 1/3-size polycarbonate lid (webstaurantstore.com, Choice 176FP13LHCL), accepted 12 to 20 mm and run at both ends | the fingers reach into the wells and pinch the land, as a hand does. Real check: a ruler on the bought cover, accepted 60 to 75 mm, 35 to 45 mm, land 12 to 20 mm, depth 12 to 20 mm; the demo scores 1.0 with 12 mm wells (shallow_wells) and with 20 mm wells (deep_wells) |
| Masses | pan 0.31 kg, turner 0.34 kg | 0.682 lb pan; 0.75 lb turner shipped | maker listings | the turner is no lighter than the real one |
| Patty on the blade | the patty resting on the blade face (within 4 mm of lying on it, inside its outline) | the patty rides the blade | contact rule, no credit for nearness | same |
| Cover clear | lowest point 10 mm or more above the rim, or wholly off the pan's outline | lifted or set aside | rule | same |
| Out on the blade | the patty leaves the pan resting on the blade; 0.3 needs its centre 2 cm past the 325 x 176 mm outline | | rule | same |
| Landing | centre within 12 mm, flat within 10 degrees, slid off from at most 20 mm, no rise after leaving the blade | | rule | same |
| Cover seated | within 2 mm in height, centre within 8 mm, under 1 degree | 2 mm feeler gauge | rule | same |

## Reset and consumables

Reset takes about 3 minutes: set the pan on the counter at its mark, lay the two patties on its flat floor at the far end, 3 mm apart (centres 38.7 and 47.9 cm in front of the base midpoint), seat the cover, lay the turner on the counter on its blade's back edge with its tip toward the pan, and set the muffin heel on the wrapper. The patties are Papetti's 3 1/2 in 1.5 oz patties ($59.49 a case of 160, $0.37 each), thawed and at room temperature. The act neither cuts nor eats them, so the same two serve every trial of a session: reset means sliding the served patty back into the pan with the turner. Replace them each session, or sooner if one tears. The muffin heel (a Thomas' Original English Muffin half toasted to a dry crust, about 95 mm across and 30 g) is reused the same way. A non-food stand-in for the patty is an 89 mm disc cut from 5/16 in (8 mm) food-grade silicone sheet; check with calipers that it measures 7-9 mm thick, and for the muffin heel a 95 mm x 22 mm disc of closed-cell foam or cork with a dry, rough face; check each against the friction ranges above before use. The pan is wiped dry between trials.

## Robot figures the demo keeps within

Grip 30 N on the cover's land, then a short press of the closed fingers on it and 40 N on the turner handle (the YAM's figure is about 40 N a finger); payloads under 0.35 kg; both work points within about 0.45 m of the bases. Run: 716 of 1,200 steps.

## Real trial

The kit is the Cambro 32CW135 pan, the Vollrath 31300 cover, the Mercer M18480 turner (about $40 in all), a case of Papetti's patties ($59.49) and a pack of Thomas' Original English Muffins (6 count), all from restaurant suppliers and grocers. Pad covers are not needed: the objects are light and none can shatter.

## Real items

- Egg patty: Papetti's 3 1/2 in Round Fully Cooked Scrambled Egg Patty, 1.5 oz, 160 a case (Michael Foods), https://www.webstaurantstore.com/papettis-3-1-2-round-fully-cooked-scrambled-egg-patty-1-5-oz-case/873MIC570225.html
- Tray: Cambro Camwear 1/3-size clear food pan, 2 1/2 in deep, 32CW135, https://www.burkett.com/products/cambro-1-3-size-2-1-2-deep-clear-food-storage-pan
- Cover: Vollrath 31300 1/3-size clear solid cover, https://www.webstaurantstore.com/vollrath-31300-1-3-size-clear-solid-cover/92231300.html
- Turner: Mercer M18480 solid stainless offset turner, 5 x 4 in blade, https://www.webstaurantstore.com/turner-5x4-turner-rosewoo/470M18480.html
- Muffin heel: Thomas' Original English Muffins, 6 count, 13 oz, https://www.walmart.com/ip/16419108616
