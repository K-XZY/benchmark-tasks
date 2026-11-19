# Put a Ken doll's head back on: hold the body in one hand and push the head onto its neck ball until it snaps

Most people who have owned a fashion doll know its head pops off. To put it back, people hold the body in one hand and push the head onto the neck with the other. The neck hole is inside the head, so it goes out of sight over the post and has to find the 7 mm ball within about 1.5 mm before a firm push, about 8 N, snaps the hole's lip over the ball. Off centre or crooked, the rim lands on the side of the ball and the head tilts instead of going on; pushed with the body left lying loose, the light doll slides away from the push.

**Instruction:** "Put the doll's head back on its neck."

**What this task adds.** Snap-fit tasks press a part onto a fixed hub or post. Here both parts are free: the 90 g body must be held off the bench by one hand while the other carries the 12 g head, turns its hole to the post, lines it up on the post's axis and pushes it over the ball.

**How the demo does it.** The left hand lifts the body 3 cm by its hips (jaws across the 30 mm hips, 35 N a finger). The right hand picks the head across its middle (35 N a finger), lifts it, turns it a quarter turn so the hole faces the post, then steers it by its own pose: the demo reads the head's and the body's poses from the simulator (a policy gets only the cameras), turns the head's axis onto the post's, centres the hole 12 mm off the ball's tip, and presses along the post at 13 N, keeping the head's axis on the post's as it goes, until the head is home and the push has built up. Both hands let go and back away; the body is set down first.

**Result** (scripted, GPU physics, RTX 4070 Ti SUPER, 20 Hz, 640 x 360): 1.0 in 848 steps (42.4 s), three runs of the command below. The head goes home with the arm's push read at 11 to 12 N, sits within 0.12 mm and 0 degrees of the post's axis once the hands are clear, and holds through the 3 N pull. Lazy-route checks, scored by the same function:

| Check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 |
| `light` | the whole demo at a 4 N push, under the lip's 8 N | 0.3 in each of 3 runs (home, but not snapped: the head rests on the ball) |
| `unseated` | the head pushed 3 mm onto the ball and let go | 0 (it falls off) |
| `unheld` | the right hand does it all, the body left lying | 0 or 0.3 in two runs (the head meets the bench beside the post; it ends off the ball or resting on it, never snapped) |
| `crooked` | the head pushed on 20 degrees off the post's axis, not steered | 0 here; the lead-in can straighten such a head, so this shows only that a head left more than 14 degrees off the axis scores 0 |

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `taskspace.py` are in this folder.

## Run a trial

```bash
python demokit.py task.py OUT --mode full                                     # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode none,light,unseated,unheld,crooked  # lazy-route checks
```

## Building it for a real trial

- **Doll:** a Barbie Fashionistas Ken with sculpted hair (#227 was used for the sizes; any Fashionistas Ken with the same ball-ended neck post will do), about 12 in (30 cm) tall, its head popped off.
- **Start:** the body face up on the bench, feet away from the robot's right arm, neck post toward it; the head on its side beside the shoulders, its neck hole facing away from the robot. Tape marks for both.
- **Replace the head** (or the doll) when it no longer keeps the body in the lift test below: the vinyl hole loosens with repeated fitting.

| Value | In the simulator | How to measure it on the real doll |
|---|---|---|
| Height | 30 cm (listed 12 in) | A tape |
| Body | 26 cm without the head, 90 g; hips 30 mm, shoulders 52 x 22 mm, chest 34 mm across and 20 mm deep (estimates) | Calipers, a kitchen scale |
| Head | 40 x 30 x 34 mm, 12 g (estimates) | Calipers, a kitchen scale |
| Neck post | 5 mm stem, 7 mm ball, 9 mm long (estimates) | Calipers |
| Neck hole | 5 mm with a 2 mm lip (estimates) | Calipers |
| Push-on force | 8 N peak (estimate) | A push gauge on the crown while the body is held |
| Frictions | 0.5 on the bench, 0.8 gripper on vinyl (estimates) | A tilt test |

## Modelled, not sourced from CAD

No CAD of the doll is published, and no third-party mesh is used: the doll is modelled in `task.py` from the listing's height and estimates from listing photos. What is seen is a slim lathed body in a T-shirt and shorts with arms, hands, legs and shoes, and a lathed head with sculpted hair, ears and a painted face; what collides is simpler (boxes for the body, a hull and a ring of wedges for the head), within about 2 mm of the visual at the neck, shoulders and hips. The real head is soft vinyl whose 5 mm hole stretches over the ball. Here the head is rigid, with a 7.5 mm bore and a 10 mm lead-in round the 7 mm ball, and the snap is a rule on the robot's real push: the head counts as snapped once it is fully on (within 2 mm and 14 degrees of the post's axis) while the hand holding it pushes it at least 8 N along the neck for 3 control steps, read from that arm's joint springs (the stretch between its joint targets and its joints, over its compliance). A snapped head is then held on by a 4 N force along the neck in code, which the scorer's 3 N pull tests. No published size exists for the doll except its 30 cm height, so there is no size check beyond that.

## Safety for a real trial

Keep people clear of the arms while the robot runs and keep an emergency stop in reach. The doll is light plastic and vinyl; nothing sharp.

## Sources

- Barbie Fashionistas Ken #227, sculpted brunette hair: https://www.maziply.com/a/s/products/barbie-fashionistas-ken-227-brunette-fashion-doll
- Height about 12 in (30 cm): https://www.mcgrocer.ae/products/barbie-fashionistas-ken-doll-assortment-12inch-30cm and Target Fashionistas Ken #226 (12 in) https://www.target.com/p/-/A-89979815
- Every size except the height, the masses, the snap force and the frictions: estimates from listing photos, as Mattel publishes none.

Barbie and Ken are trademarks of Mattel, named for identification only; this task is not affiliated with Mattel.
