# Brake pad change: push two pads flush into their retainers each side of the disc, then swing the caliper down over them

A front brake corner of a small hatchback stands on a bench stand, its caliper pivoted up on its upper guide pin. The robot pushes two new pads into their sprung retainers either side of the disc, one at a time, until each seats flush, then swings the caliper down over them until it shuts.

**Ability tested:** a sequence of fits with tight clearances. Each pad is carried under the raised caliper and its ears pushed 12 mm into the two retainers on its side of the disc until its backing plate seats flush: the inner pad, with its shim and wear indicator, on the piston's side (the arms' left), the outer pad on the fingers' side (the arms' right); both go in from the front. The 2 kg caliper is then swung down about its upper pin, so that its piston and its outer fingers pass either side of the 21 mm disc and the pads with 2 mm to spare, and its bridge closes over both pads' edges with 1 mm: a pad standing more than 1 mm proud stops the caliper short. In the demo one hand fits the pads and the other swings the caliper; the score does not check which hand does what.

**Why it matters:** replacing brakes is a core task of automotive service technicians and mechanics (O*NET 49-3023: "Repair, reline, replace, and adjust brakes"), and the pad change is the most common brake job. The order is the maker's: pivot the caliper up out of the way, install the pads with the wear indicators on the inside, pivot the caliper down into position.

**The real geometry.** The corner has a round vented disc at its real thickness, a caliper body with its piston and fingers on its guide pin, the bracket with sprung pad retainers, and pads with ears and backing plates, all at the scale of a Honda Jazz front brake. The dimensions and clearances are taken from that car's parts; the retainers' spring drag is not modelled and the push into them is not measured (see the calibration table below).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Fit the new brake pads, then swing the caliper down over them."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                    # full score, with a video
python demokit.py task.py OUT --no-record --mode noflush,onepad,noshut,none   # the failure modes below
```

The demo is a scripted reference. It reads the poses of the pads and of the caliper, which the cameras see; a policy gets only the cameras. The right arm takes each pad from the parts box by its edge, 35 mm below its centre (a level grip, the jaws across its 15 mm), carries it under the raised caliper, lines it up with its retainers clear of the disc, and pushes it straight in 12 mm until flush. The left arm then pushes the caliper's bridge round its pin with closed fingertips until the caliper shuts over both pads, and both hands back away.

## Building it for a real trial

- **Brake corner:** a front corner of a Honda Jazz (2002 to 2008, 1.3) or the same class, from a breaker's yard, bolted by its knuckle to a bench stand (a 40 x 40 mm steel post on a 220 x 160 mm foot plate, clamped to the bench) with the disc's centre about 25 cm above the bench: a vented disc 262 mm across and 21 mm thick, the caliper bracket bolted to the knuckle, four pad retainers fitted and greased.
- **Caliper:** the single-piston floating caliper on its upper guide pin, its piston pushed back, its lower guide bolt out, pivoted up out of the way (100 degrees from shut). Weigh it: about 2 kg is our estimate. If it does not stay at 100 degrees, add friction at the pin, for example an O-ring on the pin under its rubber sleeve, until the luggage-scale test in the table below reads about 1.6 N m. The brake hose may stay connected; it hangs slack.
- **Pads:** a new pair for that car (131.4 x 54.7 x 15.3 mm with ears, 0.2 kg each), the inner one with its shim and wear indicator, standing on one ear, linings towards the disc's side they belong to, in a slotted wooden parts box (70 x 100 mm, 13 cm tall) beside the stand.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Caliper mass | 2.0 kg | **Not measured.** Weigh it |
| The guide pin's hold | 1.6 N m of dry friction, above the caliper's 1.4 N m weight moment, so it stays where it is left | **Not measured.** With a luggage scale at the caliper's lower end, read the pull at which it starts to swing, times the distance from the pin |
| The pads' push into the retainers | greased channels, friction 0.3; no spring drag or snap | **Not measured.** Push a pad into its retainers with a push gauge and read the drag and the snap at flush |

## Safety for a real trial

The corner is heavy and the caliper swings on its pin: clamp the stand to the bench, keep hands out of the caliper's swing while the robot works, and keep an emergency stop in reach. Brake dust: do not blow it; wipe the bracket with brake cleaner before the trial.

## Scoring

See `scoring` in `task.json`, and `score`, `success` and `pad_now` in `task.py`. In a real trial:

- **Pads flush:** each pad in its own retainers, the inner pad (shim and wear indicator) on the piston's side and the outer on the fingers' side, both linings facing the disc, and pushed the full 12 mm in; a 1 mm feeler gauge must not enter between any ear and its retainer's end. A pad on the wrong side or turned round does not count.
- **Caliper shut:** swung down onto its bracket, its lower bolt hole lined up with the bracket's thread (within 1 degree in the simulator).
- **Hands clear:** no gripper within 5 cm of the pads or the caliper's bridge, all of this for 2 s: 1.0.
- **0.3:** both pads flush, without the 1.0 result. **0.1:** one pad flush. **0:** otherwise. A pad dropped off the bench caps the score at 0.1.

In the simulator, a pad counts as flush when its centre is within 1 mm of its seated position along the retainers, within 5 mm across and 3 mm up or down, and turned less than 4 degrees from its seated pose. The lower guide bolt is left out of this version: the task ends with the caliper shut over the pads. Forcing the caliper against a pad is not scored separately: a pad standing proud stops the caliper short, and the score stays at 0.3 or below.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | Both pads pushed in 12 of 12 mm, flush (the outer turned 2.4 degrees, the inner 0.9); the caliper swung shut to -0.1 degrees; held 2 s | 1.0, at step 1327 (66 s) |
| `noflush` | The inner pad pushed only 6 mm of its 12: the caliper stops short of shut | 0.1 |
| `onepad` | The inner pad left out: swung shut, the caliper knocks the lone outer pad out of its retainers and off the bench | 0.0 |
| `noshut` | Both pads flush, the caliper left up | 0.3 |
| `none` | The arms wait | 0.0 |
| Dream-1 | The right hand reaches into the parts box and touches the new pads there, but never carries one to the retainers; it then draws back, the caliper still up | 0 |

No seed is needed: nothing in the scene is random. Runtime per trial, on one GPU: the demo's 1,327 steps took 5 to 10 minutes while recording three cameras (2.3 to 4.1 steps per second); a run to the 2,400-step limit takes 10 to 18 minutes.

## Assets

All meshes are made by the task's own code from plain shapes. No third-party assets, no textures.

## Licence

MIT
