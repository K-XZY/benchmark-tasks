# Prop a door open with a rubber wedge: push it under the door's bottom edge until its slope bites, then let go

Prop a door open with an Everbilt rubber wedge: one gripper holds the door's free edge, the other pushes the wedge, thin end first, under the door's bottom edge until its slope bites. Then both let go and the scorer pulls the free edge toward closing at 20 N for 2 s.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,one_hand,no_wedge,beside,short,loose,drop,none
```

The harness's instruction (Dream-1 run): `--instruction "Prop the door open with the rubber wedge: push it under the door's bottom edge so the door stays open when pushed, then let go with both hands."`

`DW_DX` (default 0.08 m) sets the whole rig that far out in x, so the right wrist stays inside its joint limits.

## Scoring, and the routes that score less

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | left holds the door, right pushes the wedge in, both let go | 1.0 |
| one_hand | right alone pushes the wedge in against the unheld door | 0.0 (the heavy leaf is shoved, never judged) |
| no_wedge | left holds the door, no wedge | 0 (door closes 177 mm under the pull) |
| beside | wedge set down beside the door | 0 (door closes 265 mm) |
| short | wedge pushed in only 10 mm | 0 (door moves 21 mm) |
| loose | wedge laid on its side and let go | 0 |
| drop | one hand sets the wedge down 29 mm under the edge, door unheld | 0 (the door blocks the wedge from above; it lands on the edge and is never judged) |
| none | nothing | 0 |

The judging starts only after a hand has handled the door or the wedge, and every fingertip of both hands is 3 cm clear and all still for 1 s.

## Real trial: the door stub

Cut the lower 250 mm (10 in) off the bottom of a 30 x 80 in hollow-core interior door with a fine saw; glue a 6 mm plywood cap on the cut edge and a solid pine stile (35 x 35 mm) up the free edge, where a screw eye for the luggage scale goes in. Hang the stub on two 3-1/2 in butt hinges screwed to a 50 x 100 mm (2x4) post, bolted to the bench and braced; set the leaf's bottom edge 13 mm above a vinyl floor tile fixed to the bench with double-sided tape. Hook the luggage scale in the screw eye 10 cm above the tile and pull square to the leaf toward closing to 20 N for 2 s, with a ruler at the edge. The simulated leaf has the whole 762 mm width; the stub keeps it. The stub alone (about 1.5 kg, a 250 mm slice of a 12 kg hollow-core leaf) swings far lighter than a full door: about 0.3 kg m^2 about the hinge. Bolt about 4 kg of steel flat bar to its face 0.70 m from the hinge axis, near the free edge, which adds about 2.0 kg m^2 (4.1 x 0.70^2) for the sim's 2.3 kg m^2 (estimate). Or hang the stub's hinges on a full leaf.

## Sources

- Everbilt 4-3/4 in heavy-duty rubber door stop, Home Depot Canada 1000110437, listed 4.69 x 2.19 x 1.4 in: https://homedepot.ca/product/everbilt-4-3-4-inch-heavy-duty-off-white-rubber-door-stop/1000110437. No maker drawing or STL was found; the wedge is modelled as a prism of the listed size with a 5 mm tip.
- Standard 30 x 80 in, 1-3/8 in hollow-core interior door and standard 3-1/2 in butt hinges: stated sizes, modelled as boxes (the door cut to its lower 250 mm).
- The stub's swing inertia is that of a full 12 kg, 762 mm leaf (12 x 0.762^2 / 3 = 2.3 kg m2 about the hinge), an estimate; the visible stub is 250 mm tall.
- Mass, friction, contact stiffness, hinge friction and tip are estimates, as makers do not publish them.
- All meshes are built in `task.py` from boxes and a prism; no third-party meshes are used.

## Licence

MIT
