# Hold a locked keeper box flat on the checkout magnet while the other hand swings its lid open, then bag the product

A clear anti-theft keeper box with a razor-cartridge pack locked inside lies on a checkout counter, its lock end 3 cm to one side of the magnet of a detacher set flush into the counter. Its lock opens only while the box's lock end lies flat on the detacher's magnet. One gripper slides the box sideways until its lock end sits over the magnet and holds it down there. The other pinches the lid's lip and swings the lid open about its far-end hinge. Then the first lifts the pack out above the low tray walls and sets it in the bagging area. In the simulation, pulled with one hand, the lid's catch tips the box off the magnet, and the lock holds.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,onehand,pry,none,freefall,slip_release,held_fast
```

The harness's instruction (Dream-1 run): `--instruction "Free the razor cartridges from the locked keeper: slide its lock end onto the detacher's magnet, hold it flat, open the lid, bag the pack."`

## Scoring, and the routes that score less

Scored at the end of the run, or once full score has held for 1 s (20 steps); a pack set in the bagging area less than 1 s before the end scores 0.5. Every tier is read from the current state, so nothing done earlier and later undone keeps its credit; only a dropped part latches, and it voids the run for good, at any moment.

- **1.0**: the pack at rest (under 0.05 m/s) on the counter, any side down (its lowest corner within 3 mm of the counter top), every corner of it over the 20 x 16 cm bagging area, and its centre outside the keeper's outline.
- **0.5**: the lid is open 70 degrees or more.
- **0.25**: the keeper is seated on the detacher with its lid shut: the point under the middle of its lock block within 10 mm of the magnet's centre across the counter, at most 3 mm above the counter, tilt under 5 degrees.
- **0**: none of these, or a dropped part at any moment, which voids the run for good: the keeper or the pack 5 cm below the counter top, or the pack falling faster than 0.4 m/s relative to the gripper that holds it (a free fall of about 8 mm), whether released, slipping from the fingers or tumbling from the tray. In a real trial: the pack falls freely more than about 1 cm at any moment, judged from the side video.

Run on an RTX 3090 Ti; scores are the harness's task_score.

| Mode | What it does | Score |
|---|---|---|
| full | left hand slides the keeper onto the detacher and presses its lock block down; right hand swings the lid open by its lip; left lifts the pack out and sets it in the bagging area | 1.0 (709 steps, 35.45 s) |
| onehand | left hand slides the keeper onto the detacher and leaves; the right alone pulls the lip | 0.0 (the box is pulled off the magnet and the lock holds; the lid stays shut, 0 degrees) |
| pry | the keeper is left where it starts, 3 cm off the magnet; the left presses its lock block down and the right pulls the lip | 0.0 (the lock holds 40 N at the lip; the lid stays shut, 0 degrees) |
| none | both arms wait | 0.0 |
| freefall | as full, but the pack is let go in the air after the lift | 0.0 |
| slip_release | as full, but the pack is held 3 cm above the mat and the fingers loosen 3 mm a side so it slides out | 0.0 |
| held_fast | as full, but the pack is lowered in 0.25 s while held | 1.0 |

Tipping the box out after the lid is open is a valid way to get the pack out, as long as it lands gently: a pack falling faster than 0.4 m/s relative to the hand holding it voids the run.

## Real trial

A clear keeper (about 19 x 11.5 x 5 cm) with a boxed razor-cartridge pack inside, locked, on a counter 3 cm off the magnet of a detacher set into a cutout in the counter, its face flush (the real S3MDKEYST is a counter-top housing, so the trial cuts a flush opening for it); a bagging area marked 20 x 16 cm with tape beside it. Judge the end state from the trial video: pack out and any side down in the bagging area, lid angle, and whether the keeper's lock end lies on the detacher.

Checks, all cheap: a phone filming from the side and one from above (the lid angle against a printed 70 degree protractor card, the pack's corners against the taped bagging area, the 10 mm seating limit against a steel rule laid on the counter, with tick marks on the pad 10 mm either side of the magnet centre, read from the top phone; the 3 mm lift read against a steel rule stood upright beside the lock end in the side phone's frame, and the 5 degree tilt from the side phone); a 10 N spring scale (about US$10) hooked under the lid lip to read the catch while the box lies on the magnet, and a 50 N spring scale (about US$10) to read the lock's hold off the magnet; a digital caliper (0 to 150 mm, about US$15) for the lip and the tray wall. Each of these is an estimate in the simulation, with the range it was run across in the table below; a reading outside that range is entered in the task's setting before the simulated and real runs are compared. Protect the grippers' pads and the clear lid with a thin foam sleeve on the finger faces; the keeper is a rigid polycarbonate shell, no part is sharp, and the lid's lip is the only part the gripper pinches against. Reset: put the pack on its mark in the keeper, close the lid until it locks, set the keeper on its mark, its lock end 3 cm left of the magnet: about 20 s, nothing is consumed.

## Real items

- Keeper box: Alpha (Checkpoint Systems) Classic Keepers, Multi-Use Keepers-406, order code AVM406BNDL (AVM406BRFL with a radio tag), https://www.alphaworld.com/solutions/keepers/Classic-Keepers. Listed size 5.33 x 1.80 x 7.93 in (135 x 46 x 201 mm) outside, inside at least 5.05 x 1.28 x 7.23 in (128 x 33 x 184 mm), which holds the 100 x 70 x 30 mm pack. It lies on its back on the counter.
- Detacher: Alpha S3 counter detacher (Multi-Detacher), order code S3MDKEYST, listed 1.75 x 1.75 x 5.75 in (44 x 44 x 146 mm), https://www.alphaworld.com/solutions/keys-detachers/multi-detacher: a dark housing with a round magnet face and a label, set into a cutout in the counter so its face is flush.
- Departures from these sizes: the scene's keeper is 190 x 115 x 50 mm (listed 201 x 135 x 46 mm; 11 mm shorter, 20 mm narrower, 4 mm taller), a rounded estimate; its walls and lock block are likewise estimates, and its lid catch (5 N) and slider hold (40 N) are estimates, because the maker publishes no forces. The scene's detacher is the listed 146 x 44 x 44 mm housing set into the counter, its face flush (a 32 mm magnet disc and a label on it, a 3 mm rim round it), so the box rests on the counter as at a real checkout; the magnet's pull is not modelled. The smaller footprint and inside width leave the hands less room than the real keeper, never more, and the 4 mm extra height is not a grip feature. None of these gives the gripper a hold or reach the real parts lack: the pack and the lid lip are the only parts the hands grip. The pack is modelled at its real size. The maker publishes no size for the lid's edge: the model's lid lip is its top plate overhanging the base by 5 mm (3 mm thick) along the lid's whole side edge (16 cm), the few-millimetre edge of a clear rigid lid, an estimate not measured on the real box. The hand pinches it with one fingertip above and one below, jaws vertical, a plain edge with no pocket or grip block added, though its size is an estimate of the real edge, not a measurement; the lid's pull-off needs only the 5 N catch force at the lip.

## Modelling

- The keeper is one free rigid body (clear base tray 190 g: 4 mm floor, 1.6 cm walls, a grey ridged lock cap 2.6 cm long and full height at the near end) carrying a clear lid (60 g) on a revolute hinge along its far top edge, 0 to 110 degrees, with 0.12 N m of joint friction so the lid stays where it is left.
- The lock is code on the lid joint, since the simulator has no magnets. While the lid is shut and the keeper is off the magnet, the lock's slider holds the lid: a joint drive capped at 6.0 N m, 40 N at the lip 15 cm from the hinge. While the keeper is seated on the detacher (the seating test above), the magnet has pulled the slider back and only the lid's snap catch holds it: a drive capped at 0.75 N m, 5 N at the lip. Past 8 degrees the lid is off its catch and swings free. Shut again off the detacher, it relocks.
- The magnet's pull on the keeper body is not modelled. The detacher's face and the bagging area are flush with the counter and have no collider; the counter is their face.
- The pack is a free rigid box (10 x 7 x 3 cm, 80 g).
- The demo reads the pack's pose before the hand takes it (it is in plain view through the open lid). The lid's joint angle is read only for the log.

## Sizes

| Part | In the scene | Source |
|---|---|---|
| keeper outside, length x width x height | 190 x 115 x 50 mm (real 201 x 135 x 46 mm) | Alpha AVM406BNDL listing, the nearest model; rounded estimate |
| keeper inside width | 109 mm (real at least 128 mm) | 3 mm walls; the real inside is larger |
| keeper mass | 250 g (base 190 g, lid 60 g) | estimate |
| razor-cartridge pack | 100 x 70 x 30 mm, 80 g | estimate for an 8-cartridge retail pack |
| detacher housing | 146 x 44 x 44 mm, set flush into the counter, 32 mm magnet disc | Alpha S3MDKEYST listing; disc size estimate |
| lid lip | 5 mm overhang, 3 mm thick, along the lid's whole 16 cm side edge (real: unpublished, a few mm) | estimate: the few-mm edge of a clear lid |
| lid catch, lock hold | 5 N and 40 N at the lip | estimates |
| detacher seating window | 10 mm across, 3 mm lift, 5 degrees tilt | estimates |

## Estimates and the range each was run across

The maker publishes none of these. Each has a bound, a real-trial reading, and full-score runs at both ends of its range: the low end as one combined run (catch 10 N, lip 3 x 2 mm, wall 25 mm, seating window 5 mm), the high end as another (catch 2 N, lip 6 x 3 mm, wall 10 mm, window 15 mm).

| Estimate | In the scene | Range and bound | Real-trial reading | Runs at the ends |
|---|---|---|---|---|
| lid catch, keeper on the magnet | 5 N at the lip | 2 to 10 N. A polycarbonate snap hook (E 2.4 GPa) 10 mm wide, 10 mm long, deflected 0.5 mm needs P = 3EIδ/L³ (snap-fit guide below) = 1.5 N to 6.6 N for hook thicknesses 0.8 to 1.3 mm, and with a 30 degree ramp and friction 0.3 about 1.6 to 7 N to pull off | 10 N spring scale under the lip | full 1.0 at 2 N (708 steps) and 10 N (710 steps); onehand 0.0 at 2 N |
| lock hold, keeper off the magnet | 40 N at the lip | 20 N and up: at least twice the catch range's top, so a hand that opens the catch cannot open the lock | 50 N spring scale under the lip | the lock is released while seated, so full does not use it; onehand 0.0 and pry 0.0 at 20 N |
| lid lip | 5 mm overhang, 3 mm thick | 3 to 6 mm overhang, 2 to 3 mm thick: polycarbonate walls are moulded 1 to 3.8 mm thick (Fictiv design guide), and a lid's edge overhangs its base by one or two wall thicknesses | caliper | full 1.0 at 3 x 2 mm and 6 x 3 mm; with the lip at 1.5 x 2 mm (below the bounded range) full 1.0 in 707 steps, onehand 0.0 |
| magnet pull on the seated keeper | not modelled (0 N) | no maker value. The pull can only help hold the box down, so leaving it out makes the act no easier. The one-hand result holds only while the pull on the lock end is under 7.5 N: the hand lifts the lip 4 cm from the lock end, the hinge is 9.5 cm past the box centre, and the catch's 5 N at the lip with the box's 2.5 N weight tips the box about its far edge with 2.9 N at the lock end, and about its near side edge with 7.5 N, so 7.5 N holds it. A 32 mm disc magnet through about 3 mm of plastic onto a small steel slider is expected to pull well under that | 50 N spring scale hooked under the lock end, lifting straight up while the box lies on the magnet: the setting's claim that a one-hand pull fails is made for a reading under 7.5 N; a reading of 7.5 N or more is entered in the task's setting and the claim is limited to the simulation, which carries no magnet pull | onehand 0.0 at the modelled 0 N; the one-hand result is claimed for a real pull under 7.5 N |
| base tray wall height | 16 mm | 10 to 25 mm: under the listed 33 mm inside depth | caliper | full 1.0 at 10 and 25 mm |
| seating window across the counter | 10 mm | 5 to 15 mm: the slider's magnet must stay over the 32 mm disc (16 mm radius) | steel rule with tick marks, top phone | full 1.0 at 5 and 15 mm (lock end within 1 to 5 mm of the magnet during the act) |
| seating lift and tilt | 3 mm, 5 degrees | the keeper lies flat on the counter throughout (0 mm, 0 degrees in every run), so neither limit is reached | upright steel rule and side phone | not binding |

## Run record

`python demokit.py task.py OUT --mode full` and `python demokit.py task.py OUT --no-record --mode onehand,pry,none,freefall,slip_release,held_fast` on an RTX 3090 Ti: full 1.0 in 709 steps (35.45 s), the lid swung to its 110 degree stop, 0 blank frames; onehand 0.0; pry 0.0; none 0.0; freefall (the pack let go in the air) 0.0 and slip_release (the pack held 3 cm up, the fingers loosened 3 mm a side) 0.0, both voided as a dropped pack; held_fast (the pack lowered in 0.25 s while held) 1.0 in 692 steps. The range runs are in the table above.

Dream-1 (one run, seed 1, 2,400 steps, 16 rows per request, one uninterrupted session): task_score 0.0.

## Sources

- U.S. Bureau of Labor Statistics, Occupational Employment and Wage Statistics, May 2024, 41-2011 Cashiers, industries with the highest levels of employment, https://www.bls.gov/oes/2024/may/oes412011.htm: health and personal care retailers 152,360; general merchandise retailers 554,230.
- Retail Brew, "Consumers wait an average of 7.7 minutes for products to be unlocked", 16 Dec 2024, https://retailbrew.com/stories/2024/12/16/consumers-wait-an-average-of-7-7-minutes-for-products-to-be-unlocked: of 626 stores checked, items were locked up in 100% of drug stores and 88% of mass-market stores.
- O*NET OnLine, 41-2011.00 Cashiers, Tasks, https://www.onetonline.org/link/details/41-2011.00: Core task (importance 86): "Establish or identify prices of goods, services, or admission, and tabulate bills, using calculators, cash registers, or optical price scanners." Supplemental task: "Bag, box, wrap, or gift-wrap merchandise, and prepare packages for shipment."
- Checkpoint Systems (Alpha), Classic Keepers, https://www.alphaworld.com/solutions/keepers/Classic-Keepers, and Multi-Detacher, https://www.alphaworld.com/solutions/keys-detachers/multi-detacher: the keeper's clear lid, grey base and lock end, and the counter detacher that opens it.
- Fictiv, Injection Molding Design Guide, https://www.fictiv.com/wp-content/uploads/2021/07/IM-Design-Guide-ebook.pdf: polycarbonate wall thickness 0.040 to 0.150 in (1 to 3.8 mm).
- Bayer MaterialScience, Snap-Fit Joints for Plastics, a design guide, https://fab.cba.mit.edu/classes/S62.12/people/vernelle.noel/Plastic_Snap_fit_design.pdf: the cantilever snap's deflection force and its mating force over a ramp with friction.
- All meshes are built in `task.py` from boxes and cylinders; no third-party meshes are used.

Checkpoint and Alpha are trademarks of their owners, named for identification only.

## Licence

MIT
