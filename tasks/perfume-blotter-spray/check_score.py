"""The scorer on hand-built states, with no simulator: each rung, each rule and a case that fails it.

    python check_score.py

Every case is a run of per-step states fed to task.py's own Trial. Exits non-zero if any case scores other than the scoring text says.
"""

from __future__ import annotations

import importlib.util
import os
import pathlib
import sys
import types

import numpy as np

os.environ["PERFUME_TRACE"] = "0"
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gatekit as gk  # noqa: E402

# stand-ins for the simulator side of the kit: task.py builds its props at import, and the scorer reads states through m.state and m.fingertips
dk = sys.modules["demokit"] = types.ModuleType("demokit")
dk.TABLE_TOP = 0.0
mk = sys.modules["mechkit"] = types.ModuleType("mechkit")
mk.Look = lambda *a, **kw: None
mk.rgb = lambda *a: a
mk.ENV_PATH = "/W"
mk.set_visible = lambda *a, **kw: None
for fn in ("prism", "box", "cylinder", "sphere", "join", "col_hull", "col_cylinder", "col_box", "slider", "mechanism", "free", "fixed", "on_step", "on_reset"):
    setattr(mk, fn, lambda *a, **kw: None)
mk.state = lambda env, name: env[name]
mk.fingertips = lambda env: env["tips"]
spec = importlib.util.spec_from_file_location("perfume_task", HERE / "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

P = gk.Pose
R_TRAY = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])  # a strip lying flat, its tip towards -y
R_ALONG_X = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # a strip lying flat, its tip towards +x, its width along y


def strip(i, dz=0.0):
    return P((t.STRIP_X[i], t.STAND_Y, t.STRIP_Z0 + dz))


def tester(yaw=-90.0, act=0.0, pos=None, R=None):
    R = gk.rot((0.0, 0.0, 1.0), yaw) if R is None else R
    return P(t.B_AT if pos is None else pos, R, joints={"act": act})


T0 = tester()  # nozzle facing +x, away from the robot
TA = tester(180.0)  # turned: nozzle facing -y, towards the strips' side
TP = tester(180.0, act=-t.STROKE)  # pressed through its stroke


def held_at(tip_above_axis=0.010, dist=0.15, tst=TA):
    """Strip 0 tip up in the spray's line, `dist` out from the orifice, its tip `tip_above_axis` above the spray's axis."""
    o, a = t.nozzle(tst)
    tip = o + dist * a + np.array([0.0, 0.0, tip_above_axis])
    return P(tip - np.array([0.0, 0.0, t.S_L / 2]))


S_AIM = held_at()
S_UP = strip(0, 0.045)  # lifted 4.5 cm out of its slot
S_TRAY = P((t.TRAY[0], t.TRAY[1], t.TRAY_FLOOR + t.S_T / 2), R_TRAY)
S_COUNTER = P((0.20, 0.15, t.TOP + t.S_T / 2), R_TRAY)  # lying on the glass counter, clear of the tray and the block
S_ON_STAND = P((t.STRIP_X[2], t.STAND_Y - 0.015 - 0.0012, t.TOP + 0.005 + t.S_L / 2))  # upright, its face against the stand's -y side, 5 mm off the counter
S_ON_BLOCK = P((t.BLOCK[0], t.BLOCK[1] - 0.04, t.BLOCK_TOP + t.S_T / 2), R_ALONG_X)  # lying on the display block, 1.5 cm from the tester
S_FLOOR = P((0.2, 0.5, t.TABLE - 0.7))


def S_ON_NOZZLE():
    """Strip 0 level, pointing at the turned tester's nozzle, its tip 1 mm from the actuator."""
    o, _ = t.nozzle(TA)
    return P(np.array([o[0], o[1] - 0.001 - t.S_L / 2, o[2]]), np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]))


S_BY_STRIP1 = P((t.STRIP_X[1], t.STAND_Y + 0.0015, t.STRIP_Z0 + 0.030))  # strip 0 held 1.5 mm from strip 1's face, 3 cm up, clear of the stand


G_FACES = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # in the strip's frame: jaws closing through it, fingers across its width


def grasp(pose):
    """The right hand's jaws shut across strip `pose`'s faces at the demo's pinch, 80 mm below its tip."""
    return gk.tips(right=gk.hand(pose.to_world(np.array([0.0, 0.0, t.PINCH]))[0], pose.R @ G_FACES, opening=t.S_T / 2))


def env(s0, tst, others=None, tips=None):
    d = {"strip0": s0, "tester": tst, "tips": gk.tips() if tips is None else tips}
    for i in range(1, t.N_STRIPS):
        d[f"strip{i}"] = (others or {}).get(i, strip(i))
    return d


def run(phases):
    """phases: (steps, state) in order; returns the score at the end."""
    tr = t.TRIAL
    tr.reset()
    for n, e in phases:
        for _ in range(n):
            tr.step(e)
    return tr.score()


def full(detour=(), press=None, s_aim=S_AIM, t_aim=TA, end=None, others_from_press=None, tips_end=None, t_end=TA, presses=1):
    """The whole act, with one part changed: strip 0 lifted, held in the line, the tester turned and pressed, the strip laid on the tray."""
    o = others_from_press
    ph = [(3, env(strip(0), T0)), (2, env(S_UP, T0, tips=grasp(S_UP))), (2, env(s_aim, t_aim, tips=grasp(s_aim)))]
    for _ in range(presses):
        ph += [(1, env(s_aim, press or TP, o, grasp(s_aim))), (2, env(s_aim, t_aim, o, grasp(s_aim)))]
    ph += [(3, env(s, t_aim, o, grasp(s))) for s in detour]
    ph += [(25, env(end or S_TRAY, t_end, o, tips_end))]
    return ph


bad = 0


def check(label, got, want):
    global bad
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {label}")


# the rungs, in order
check("doing nothing", run([(40, env(strip(0), T0))]), 0.0)
check("0.5: the mark across the top 40 mm, the strip still held at the end", run(full(end=S_AIM)), 0.5)
check("0.75: that strip on the tray, but it touched the counter on its way", run(full(detour=[S_COUNTER])), 0.75)
check("1.0: on the tray, touched nothing else, others standing, tester on its block, hands clear, still 1 s", run(full()), 1.0)

# rule 0.5, the mark: each case fails it
check("an unsprayed strip on the tray (no press)", run([(3, env(strip(0), T0)), (2, env(S_UP, T0)), (2, env(S_AIM, TA)), (25, env(S_TRAY, TA))]), 0.0)
check("the spray misses: tester not turned", run(full(t_aim=T0, press=tester(-90.0, act=-t.STROKE), t_end=T0)), 0.0)
check("the mark only below the top 40 mm (tip 9 cm above the axis)", run(full(s_aim=held_at(0.090))), 0.0)

# rule 0.75, lying on the tray
check("the sprayed strip held 3 cm over the tray at the end", run(full(end=P((t.TRAY[0], t.TRAY[1], t.TRAY_FLOOR + 0.03), R_TRAY))), 0.5)
check("the sprayed strip laid on the counter instead of the tray", run(full(end=S_COUNTER)), 0.5)
S_OVER = P((t.TRAY[0], t.TRAY[1], t.TRAY_FLOOR + 0.015), R_TRAY)
edge_hand = gk.tips(right=gk.hand(S_OVER.pos, opening=t.S_W / 2))  # jaws shut across the strip's 12 mm, by its long edges
check("the sprayed strip pinched by its edges, flat, 1.5 cm over the tray at the end", run(full(end=S_OVER, tips_end=edge_hand)), 0.5)
edge_hand_on = gk.tips(right=gk.hand(S_TRAY.pos, opening=t.S_W / 2))
check("the sprayed strip on the tray floor, a hand still gripping it by its edges", run(full(tips_end=edge_hand_on)), 0.5)

# rule 1.0, touched nothing but the grippers and the tray after leaving the stand
check("the strip set down on the counter on its way to the tray", run(full(detour=[S_COUNTER])), 0.75)
check("the strip brushed against the stand's side on its way", run(full(detour=[S_ON_STAND])), 0.75)
check("the strip laid on the display block on its way", run(full(detour=[S_ON_BLOCK])), 0.75)
check("the strip's tip against the tester's nozzle", run(full(detour=[S_ON_NOZZLE()])), 0.75)
check("the strip brushing another strip's face", run(full(detour=[S_BY_STRIP1])), 0.75)

# rule 1.0, the other strips still standing in their slots
check("a second strip knocked out of the stand onto the counter", run(full(others_from_press={2: S_COUNTER})), 0.75)
check("a second strip tipped over in its slot (40 degrees)", run(full(others_from_press={3: P(strip(3).pos, gk.rot((1.0, 0.0, 0.0), 40.0))})), 0.75)
check("allowed: a second strip raised 1 cm in its slot (still in it)", run(full(others_from_press={1: strip(1, 0.010)})), 1.0)

# rule 0.5, the mark is made with the strip up in the air: strips sprayed in the stand, or on the tray, earn nothing
def tester_at(nozzle_at, R, act=-t.STROKE):
    """The tester posed (rotation R) so that its orifice, at `act`, is at `nozzle_at`."""
    off = tester(R=R, act=act, pos=(0.0, 0.0, 0.0))
    return tester(R=R, act=act, pos=np.asarray(nozzle_at) - t.nozzle(off)[0])


def raw_mark(i):
    """Whether strip i's card shows a full-width mark within 40 mm of its tip, counted or not."""
    return float(t.mark_ok(t.TRIAL.hits[i]))


R_TA = gk.rot((0.0, 0.0, 1.0), 180.0)
t_at_stand = tester(180.0, act=-t.STROKE, pos=((t.STRIP_X[0] + t.STRIP_X[1]) / 2, t.STAND_Y + 0.12, t.TOP + 0.05))
in_stand_spray = [(3, env(strip(0), T0)), (1, env(strip(0), t_at_stand, tips=grasp(strip(0)))), (2, env(strip(0), TA)), (2, env(S_UP, TA)), (25, env(S_TRAY, TA))]
check("strips 0 and 1 sprayed while they stand in the stand (strip 0 gripped there; tester lifted 12 cm off), then strip 0 laid on the tray", run(in_stand_spray), 0.0)
check("  (strip 0's card shows the mark, not counted)", raw_mark(0), 1.0)
tip0 = strip(0).to_world(t.TIP)[0]
t_close = tester_at(tip0 + np.array([0.0, 0.06, -0.02]), R_TA)  # lifted, its nozzle 6 cm from strip 0's face, 2 cm below its tip
end_strip = [(3, env(strip(0), T0)), (1, env(strip(0), t_close, tips=grasp(strip(0)))), (2, env(strip(0), TA)), (2, env(S_UP, TA)), (25, env(S_TRAY, TA))]
check("strip 0 alone sprayed in its slot from 6 cm (gripped there; no other strip marked), then lifted and laid on the tray", run(end_strip), 0.0)
check("  (strip 0's card shows the mark, not counted)", raw_mark(0), 1.0)
check("  (no other strip's card is marked)", float(any(np.any(t.TRIAL.hits[k] >= t.MARK_HITS) for k in range(1, t.N_STRIPS))), 0.0)
R_DOWN = gk.rot((1.0, 0.0, 0.0), -90.0)  # the tester on its back, its nozzle pointing down
spot = S_TRAY.to_world(t.TIP - np.array([0.0, 0.0, 0.02]))[0]  # 2 cm in from the tip of the strip lying on the tray
on_tray_spray = [(3, env(strip(0), T0)), (2, env(S_UP, T0)), (3, env(S_TRAY, T0)), (1, env(S_TRAY, tester_at(spot + np.array([0.0, 0.0, 0.10]), R_DOWN), tips=grasp(S_TRAY))),
                 (2, env(S_TRAY, TA)), (25, env(S_TRAY, TA))]
check("the unsprayed strip laid on the tray, then sprayed there from 10 cm above while gripped (tester lifted, set back)", run(on_tray_spray), 0.0)
check("  (its card shows the mark, not counted)", raw_mark(0), 1.0)
held_low = P((S_UP.pos[0], S_UP.pos[1], t.TOP + 0.005 + t.S_L / 2))  # held upright out of its slot beside the stand, its foot 5 mm over the counter
low_spray = [(3, env(strip(0), T0)), (2, env(S_UP, T0)), (1, env(held_low, tester_at(held_low.to_world(t.TIP)[0] + np.array([0.0, 0.08, -0.02]), R_TA), tips=grasp(held_low))),
             (2, env(S_UP, TA)), (25, env(S_TRAY, TA))]
check("strip 0 gripped with its foot 5 mm over the counter when sprayed (not in the air), then laid on the tray", run(low_spray), 0.0)
check("  (its card shows the mark, not counted)", raw_mark(0), 1.0)

# the strip resting across the tray's two 1 cm rims (it is 130 mm long, the tray 120 mm across x), sprayed there from above, then nudged onto the floor
S_RIMS = P((t.TRAY[0], t.TRAY[1], t.TOP + t.TRAY_RIM + t.S_T / 2), R_ALONG_X)
rim_spot = S_RIMS.to_world(t.TIP - np.array([0.0, 0.0, 0.02]))[0]
for h, hands in (("no hand on it", None), ("a hand gripping it", grasp(S_RIMS))):
    rims = [(3, env(strip(0), T0)), (2, env(S_UP, T0, tips=grasp(S_UP))), (3, env(S_RIMS, T0, tips=hands)),
            (1, env(S_RIMS, tester_at(rim_spot + np.array([0.0, 0.0, 0.08]), R_DOWN), tips=hands)), (2, env(S_RIMS, TA)),
            (25, env(P((t.TRAY[0], t.TRAY[1], t.TRAY_FLOOR + t.S_T / 2), R_ALONG_X), TA))]
    check(f"the strip resting across the tray's rims, {h}, sprayed from 8 cm above, then nudged onto the floor", run(rims), 0.0)
    check("  (its card shows the mark, not counted)", raw_mark(0), 1.0)
# the strip lying on the display block beside the tester, sprayed there, then laid on the tray
S_BLOCK_AIM = P((t.BLOCK[0] - 0.02, t.BLOCK[1] - 0.045, t.BLOCK_TOP + t.S_T / 2), R_ALONG_X)
block_spray = [(3, env(strip(0), T0)), (2, env(S_UP, T0, tips=grasp(S_UP))), (3, env(S_BLOCK_AIM, T0)),
               (1, env(S_BLOCK_AIM, tester_at(S_BLOCK_AIM.to_world(t.TIP - np.array([0.0, 0.0, 0.02]))[0] + np.array([0.0, 0.0, 0.08]), R_DOWN))),
               (2, env(S_BLOCK_AIM, TA)), (25, env(S_TRAY, TA))]
check("the strip lying on the display block, sprayed there from above, then laid on the tray", run(block_spray), 0.0)
check("  (its card shows the mark, not counted)", raw_mark(0), 1.0)

# effects only: spray that passes the held strip is not scored; with the spray reaching 30 cm the standing strips behind it are wetted
t.RANGE = 0.30
check("the spray reaching 30 cm, past the held strip onto the standing strips: still 1.0", run(full()), 1.0)
check("  (a standing strip's card is wetted)", float(any(np.any(t.TRIAL.hits[k] >= t.MARK_HITS) for k in range(1, t.N_STRIPS))), 1.0)
t.RANGE = 0.22

# rule 1.0, the tester on its block; the hands clear; everything still
check("the tester left lifted 3 cm off its block at the end", run(full(t_end=tester(180.0, pos=np.add(t.B_AT, (0.0, 0.0, 0.03))))), 0.75)
check("an open hand within 2 cm of the strip on the tray", run(full(tips_end=gk.tips(right=gk.hand((t.TRAY[0], t.TRAY[1], t.TRAY_FLOOR + 0.015), opening=0.01)))), 0.75)
face = TA.to_world(np.array([[0.0, t.B_D / 2 + 0.020, 0.04]]))[0]  # 2 cm off the turned tester's broad face (its nozzle side), 4 cm up
R_ALONG_FACE = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # pointing down, its jaws closing towards the face, its width along it
check("a closed hand 2 cm off the tester's broad face at the end (3.4 cm from its mid-plane)",
      run(full(tips_end=gk.tips(left=gk.hand(face, R_ALONG_FACE, opening=0.0)))), 0.75)
jit = [(1, env(P(S_TRAY.pos + [0.002 * (k % 2), 0.0, 0.0], R_TRAY), TA)) for k in range(30)]
check("the strip still moving on the tray (2 mm each step)", run(full()[:-1] + jit), 0.75)

# the zeros, latched
lying = gk.rot((1.0, 0.0, 0.0), 90.0) @ gk.rot((0.0, 0.0, 1.0), 180.0)
low = float((np.array(t.GLASS) @ lying.T)[:, 2].min())
t_fallen = tester(R=lying, pos=(t.BLOCK[0], t.BLOCK[1], t.BLOCK_TOP - low))
check("the tester fallen over on its block at the press, then stood up again", run(full(press=t_fallen)), 0.0)
check("the tester set down on the counter, then put back on its block", run(full(press=tester(180.0, act=-t.STROKE, pos=(0.35, -0.02, t.TOP)))), 0.0)
check("a strip on the floor", run(full(others_from_press={4: S_FLOOR})), 0.0)

# effects only: what is no longer scored
check("allowed: the tester lifted 3 cm off its block to press, then set back", run(full(press=tester(180.0, act=-t.STROKE, pos=np.add(t.B_AT, (0.0, 0.0, 0.03))))), 1.0)
check("allowed: two presses", run(full(presses=2)), 1.0)
check("allowed: the strip held 8 cm from the nozzle", run(full(s_aim=held_at(dist=0.08))), 1.0)
check("allowed: the strip tilted 35 degrees at the press", run(full(s_aim=P(held_at().pos, gk.rot((1.0, 0.0, 0.0), -35.0)))), 1.0)

# review sweep: the tester must be turned; a gripped strip on the block earns nothing; the 1.0 does not outlast a carried-off tester
T0P = tester(-90.0, act=-t.STROKE)
S_FRONT = P(held_at(tst=T0).pos, gk.rot((0.0, 0.0, 1.0), 90.0))  # its face turned to the nozzle
check("the tester left facing +x (not turned), the strip held 15 cm in front of its nozzle, pressed, then laid on the tray", run(full(s_aim=S_FRONT, t_aim=T0, press=T0P, t_end=T0)), 0.0)
check("  (its card shows the mark, not counted)", raw_mark(0), 1.0)
block_hand = [(3, env(strip(0), T0)), (2, env(S_UP, T0, tips=grasp(S_UP))), (3, env(S_BLOCK_AIM, T0, tips=grasp(S_BLOCK_AIM))),
              (1, env(S_BLOCK_AIM, tester_at(S_BLOCK_AIM.to_world(t.TIP - np.array([0.0, 0.0, 0.02]))[0] + np.array([0.0, 0.0, 0.08]), R_DOWN), tips=grasp(S_BLOCK_AIM))),
              (2, env(S_BLOCK_AIM, TA, tips=grasp(S_BLOCK_AIM))), (25, env(S_TRAY, TA))]
check("the strip gripped while lying on the display block, sprayed there from above, then laid on the tray", run(block_hand), 0.0)
carried = full() + [(30, env(S_TRAY, tester(180.0, pos=np.add(t.B_AT, (0.0, 0.0, 0.10)))))]
check("1.0 reached, then the tester lifted 10 cm off its block and held there", run(carried), 0.75)

print("all as the scoring text says" if not bad else f"{bad} case(s) wrong")
sys.exit(1 if bad else 0)
