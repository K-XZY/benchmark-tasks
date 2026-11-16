"""The scorer on hand-built states, with no simulator: each rung, each rule and a case that fails it.

    python check_score.py

Every case is a run of per-step states fed to task.py's own Trial (the cart's pose, the tab joints, the fingertips and every arm body).
Exits non-zero if any case scores other than the scoring text says.
"""

from __future__ import annotations

import importlib.util
import os
import pathlib
import sys
import types

import numpy as np

os.environ["CART_TRACE"] = "0"
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gatekit as gk  # noqa: E402

dk = sys.modules["demokit"] = types.ModuleType("demokit")
dk.TABLE_TOP = 0.0
dk.DT = 0.05
mk = sys.modules["mechkit"] = types.ModuleType("mechkit")
mk.Look = lambda *a, **kw: None
mk.rgb = lambda *a: a
for fn in ("cylinder", "box", "join", "col_hull", "col_cylinder", "col_box", "hinge", "slider", "mechanism", "on_step", "on_reset", "lock", "set_target", "push"):
    setattr(mk, fn, lambda *a, **kw: None)
mk.state = lambda env, name: env[name]
mk.fingertips = lambda env: env["tips"]
spec = importlib.util.spec_from_file_location("cart_task", HERE / "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
t.arm_points = lambda env: env["links"]

P = gk.Pose
FAR_LINKS = np.array([[0.0, 2.0, 2.0], [0.0, -2.0, 2.0]])
START = np.array([t.CART_AT[0], 0.0, 0.0005])


def cart(dx=0.0, dy=0.0, yaw=0.0, l=-0.009, r=-0.009, ang=0.0, tilt=0.0):
    R = gk.rot((0.0, 0.0, 1.0), yaw) @ gk.rot((1.0, 0.0, 0.0), tilt)
    j = {f"{p}_{c}": 0.0 for p in ("sw", "wh") for c in t.CASTERS}
    j.update(tab_l=l, tab_r=r)
    return P(START + [dx, dy, 0.0], R, ang_vel=(0.0, 0.0, ang), joints=j)


def pedal(side, c=None):
    a = t.axis_of(t.NEAR[side])
    return (c or cart()).to_world(np.array([[a[0] + t.TAB_X[1] + 0.01, a[1], t.TAB_TOP]]))[0]


DOWN_ON = lambda side: gk.hand(pedal(side) + [0.0, 0.0, 0.005], opening=0.0)  # noqa: E731
FINGERS = gk.tips(left=DOWN_ON("left"), right=DOWN_ON("right"))
LEFT_ONLY = gk.tips(left=DOWN_ON("left"))
CLEAR_TIPS = gk.tips()


def env(c, tips=CLEAR_TIPS, links=FAR_LINKS):
    return {"cart": c, "tips": tips, "links": links}


def run(first, after=None, n_after=150):
    """first: (steps, env) in order; then `after(trial)` gives the env for n_after steps (default: the cart still, hands clear)."""
    tr = t.TRIAL
    tr.reset()
    for n, e in first:
        for _ in range(n):
            tr.step(e)
    for _ in range(n_after):
        tr.step(after(tr) if after else env(cart()))
    return tr.score()


bad = 0


def check(label, got, want):
    global bad
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {label}")


PRESS = [(6, env(cart(), FINGERS))]  # both tabs pressed 9 mm with a fingertip on each

check("doing nothing", run([(20, env(cart(l=0.0, r=0.0)))], None), 0.0)
check("0.25: only the left tab latched", run([(6, env(cart(r=0.0), LEFT_ONLY))], lambda tr: env(cart(r=0.0))), 0.25)
check("0.5: both latched, the cart shoved 3 cm while pressing is not allowed, but 2.5 cm is only a 0.5", run(PRESS + [(2, env(cart(dx=0.025)))], lambda tr: env(cart(dx=0.025))), 0.5)
check("1.0: both latched, the cart 1 cm from its start, hands clear, the test pushes held, still 1 s", run(PRESS + [(2, env(cart(dx=0.010)))], lambda tr: env(cart(dx=0.010))), 1.0)
check("1.0: the whole act, the cart not moved", run(PRESS), 1.0)
check("0: the cart pushed 6 cm before the test", run(PRESS + [(2, env(cart(dx=0.06)))], lambda tr: env(cart(dx=0.06))), 0.0)
check("0: the cart tipped 15 degrees", run(PRESS + [(2, env(cart(tilt=15.0)))], lambda tr: env(cart(tilt=15.0))), 0.0)

# the tab: only a real press latches it
check("a tab only 4 mm down, a fingertip on it: springs back, nothing latched", run([(6, env(cart(l=-0.004, r=-0.004), FINGERS))], lambda tr: env(cart(l=0.0, r=0.0))), 0.0)
check("a tab 9 mm down with no fingertip near it (an arm link or a knock): not latched", run([(6, env(cart(), CLEAR_TIPS, np.array([pedal("left"), pedal("right")])))], lambda tr: env(cart(l=0.0, r=0.0))), 0.0)

# the test: each way the cart can move or be held while the hands read clear
def creep(per_phase):
    """The key points slide by `per_phase[phase]` metres (counted from the start of the +x push) while that phase runs."""
    return lambda tr: env(cart(dx=per_phase.get(tr.phase, 0.0)))


check("a 4 mm creep in the +x push alone passes the cart on as locked: 1.0", run(PRESS, creep({"px": 0.004, "gap": 0.004, "py": 0.004, "tested": 0.004})), 1.0)
check("the cart slides 6 mm in the +x push: held to 5 mm, so 0.75", run(PRESS, creep({"px": 0.006, "gap": 0.006, "py": 0.006, "tested": 0.006})), 0.75)
check("4 mm in the +x push, 8 mm by the end of the gap, 12 mm by the end of the +y push: creep adds up, 0.75", run(PRESS, creep({"px": 0.004, "gap": 0.008, "py": 0.012, "tested": 0.012})), 0.75)
check("the cart yaws 3 degrees in the +y push with its middle still: the right front post moves about 10 mm, 0.75", run(PRESS, lambda tr: env(cart(yaw=3.0 if tr.phase in ("py", "tested") else 0.0))), 0.75)
def turning(rate):
    """After the test pushes, the cart turns `rate` degrees a step about its middle (2 mm/s at the farthest key point for 0.02)."""
    since = {}
    def after(tr):
        if tr.phase == "tested":
            since.setdefault("n", tr.n)
        return env(cart(yaw=rate * (tr.n - since["n"]) if since else 0.0))
    return after


check("the cart's yaw still changing at the end, its middle still: not still, 0.75", run(PRESS, turning(0.02)), 0.75)
check("the cart spinning (0.2 rad/s) so the test never starts: 0.75", run(PRESS, lambda tr: env(cart(ang=0.2))), 0.75)

# hands: every fingertip and every arm body, the whole cart
far = np.array([[0.0, 2.0, 2.0]])
check("a wrist 1 cm from a front post, the fingertips far: the test never starts, 0.75", run(PRESS, lambda tr: env(cart(), CLEAR_TIPS, np.vstack([FAR_LINKS, [-t.PX + t.CART_AT[0] - 0.01 - t.POST_R, t.PY, 0.3]]))), 0.75)
pull = START + [-t.W / 2 - 0.01, 0.0, 0.40]
check("a fingertip 1 cm off the drawer pull at y = 0 (far from every post): 0.75", run(PRESS, lambda tr: env(cart(), gk.tips(left=gk.hand(pull)))), 0.75)
check("a fingertip on the tab at the end: 0.75", run(PRESS, lambda tr: env(cart(), FINGERS)), 0.75)
check("hands clear, but one forearm 2 cm from the shelf's front edge: 0.75", run(PRESS, lambda tr: env(cart(), CLEAR_TIPS, np.vstack([FAR_LINKS, START + [-t.W / 2 - 0.02, 0.05, t.SHELF_Z]]))), 0.75)

# after the end hold, the end state must still hold
def late(back_tips=None, shove=0.0):
    def after(tr):
        if tr.n > 110:
            return env(cart(dx=shove), back_tips if back_tips is not None else CLEAR_TIPS)
        return env(cart())
    return after


check("1.0 reached, then both hands come back to the cart: the score drops", run(PRESS, late(gk.tips(left=gk.hand(pull)))), 0.75)
check("1.0 reached, then the cart shoved 3 cm: the score drops to 0.5 (moved over 2 cm)", run(PRESS, late(shove=0.03)), 0.5)
check("the same run with nothing after: 1.0", run(PRESS, late()), 1.0)

print("all cases as the text says" if not bad else f"{bad} case(s) BAD")
sys.exit(1 if bad else 0)
