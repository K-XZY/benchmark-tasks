# Lock a free-rolling utility cart in place: press both locking casters' brake tabs fully down so it no longer rolls

A rolling cart, stand or trolley is locked before it is loaded or worked at, by pressing down the brake tab on each of its locking wheels. Here the cart is a stock 3-tier metal utility cart at its real size (Mind Reader LIDCART3T-BLK, 15.5 x 11 x 28.25 in, 7.85 lb) on four swivel casters, two of them locking. The robot presses each tab past its latch, which locks that caster's wheel and swivel, and moves its hands clear. The score is judged by effect: once both casters are locked, a push that rolls the free cart does not move it.

**Instruction.** Lock the cart in place: press down the brake tabs on both of its locking wheels until each locks, then move both hands clear.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. The cart, its casters and tabs are built in `task.py` from the listing's sizes; there are no mesh files.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --seed 1 \
  --instruction "Lock the cart in place: press down the brake tabs on both of its locking wheels until each locks, then move both hands clear."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full   # full score, with a video
python check_score.py                       # the scorer on hand-built states, no simulator
```

The demo reads the cart's pose to plan its moves; it reads no contact force and moves nothing except through the grippers. The left hand holds the left front post while the right hand presses the right tab straight down until it latches; then the right hand holds the right post while the left hand presses the left tab; then both let go and move clear, and the scorer pushes the cart. `check_score.py` feeds hand-built runs through the scorer itself: each rung, the zeros, the hands-clear and stillness gates, and the lazy routes (a tab pressed part way, a tab knocked down by an arm link rather than a fingertip, a hand back on the cart during the pushes, a shove after the hold).

Other modes, for checks: `onehand`, `nosteady` (both tabs, nothing steadying the cart), `together` (both tabs at once), `half` (each tab pressed only part way), `g_roll` (no hands: the push that starts the free cart rolling).

## The scene and what is simplified

- The table top stands for the floor, because the arms cannot reach floor level, and a hand presses the tab where a person would use a foot. These two stagings are the only changes from real use; the brake's force and travel are the same either way.
- Rigid bodies at 240 Hz on the GPU. The cart is one free articulation: a frame, four caster forks on vertical swivel axes, four wheels on their axles, two sprung brake tabs.
- Each wheel is drawn as a 50 mm by 18 mm tyre and meets the floor as a ball of its radius: a round tread that rolls and locks the same at every angle of the wheel, of rubber (friction 0.8, 0.7 sliding, on the table top) with a 5 mm contact patch that gives it turning (scrub) friction.
- Rolling and swivel resistance are dry friction held by drives: 0.5 N at each wheel's rim (the free cart starts rolling at about 2.5 N) and 0.03 N m at each swivel.
- Each tab is a sprung slide with 8 mm of travel and 10 N at full travel. Pressed past 7 mm (about 9 N) it latches down, and its caster is then a total lock: the wheel and the swivel are held by stiff drives where they are. Pressed less, it springs back and the caster stays free. The latch is coded, not modelled as a part.
- The tab size, travel and force, the wheel size and the rolling resistance are assumed, with ranges in the setup text, because the listing gives none. The task is scored by effect, so any value in those ranges gives the same task.

## Building it for a real trial

Stand the cart on the station table's smooth hard top, with stops along the table's edges so a free cart cannot roll or topple off, and mark the feet of its four corner posts with tape. With a spring gauge, measure the force that starts the free cart rolling (pushing the bottom shelf's front edge, 10 cm up) and the force that latches each tab; the setup text gives the assumed ranges. Score with the video, a ruler against the marks and the spring gauge: pull it along +x at the bottom shelf's front edge, then along +y at the right front post, 10 cm up, each time raising it to 8 N over about half a second and holding 1 s. Reset: lift both tabs to unlock, set all four wheels trailing straight toward +x and put the cart back on its marks.

## Scoring

See `task.json`. In short: 0.25 with one tab latched, 0.5 with both, 0.75 with both and the cart's middle (the mean of its four post feet) never more than 2 cm from its start, 1.0 with that and the cart moving less than 5 mm under each test push (raised to 8 N over 0.5 s, held 1 s) with the hands and arms 3 cm clear, then standing still for 1 s. A tip over 10 degrees, a lift of 2 cm off the table, or the cart's middle moving over 5 cm scores 0.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps: it pressed neither tab | 0 |
| `full` | The whole task: each tab pressed while the other hand steadies the cart by its front post (the film) | 1.0 |
| `nosteady` | Both tabs pressed one at a time with nothing steadying the cart: the right press rolls it 22 mm, past the 2 cm limit; the test pushes then move it 3 mm | 0.5 |
| `together` | Both tabs pressed at once, nothing steadying the cart: it rolls 27 mm | 0.5 |
| `onehand` | Only the left tab pressed (the cart rolls 21 mm) | 0.25 |
| `half` | Each tab pressed only about 4 mm: both spring back and the casters stay free | 0 |
| `g_roll` | No hands: a push at the bottom shelf's front edge, rising 0.1 N a step, starts the free cart rolling at 2.5 N | (gate) |

A press pushes the free cart forward on its wheels, so a policy that presses with nothing steadying the cart lets it roll about 2 cm; holding a front post with the other hand keeps it to a few millimetres.

## Runtime and seeds

The demo takes 388 steps (19.4 s at 20 Hz); the run limit is 2,400 steps. Nothing is random (`object_jitter=0`).

## Sources

Cart: Mind Reader LIDCART3T-BLK, Quill listing (dimensions, 7.85 lb, two locking casters, two drawers and one covered compartment, perforated shelves; read 11-16): https://www.quill.com/mind-reader-3-tier-metal-cart-covered-compartment-black-lidcart3t-blk/cbs/55486199.html. Caster brakes in general (a total lock locks wheel and swivel): TENTE, 'How total locks work': https://www.tente.com/en-us/know-how/total-lock.

No third-party assets: the cart, its casters and tabs are built in `task.py` from the listing's sizes.

Mind Reader, Quill and TENTE are named for identification only; the names are their owners' trademarks.

## Licence

MIT
