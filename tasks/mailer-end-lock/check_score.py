"""The scorer on hand-built states, with no simulator: each rung, each rule and a case that fails it.

    python -B check_score.py

Every case is a run of per-step Seen states fed to task.py's own Trial. Exits non-zero if any case scores other than the scoring text says.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
dk = sys.modules["demokit"] = types.ModuleType("demokit")  # the simulator side of the kit: task.py needs only its constants here
dk.TABLE_TOP, dk.DT = 0.0, 0.05
for fn in ("level", "tilted", "down", "_as_grip"):
    setattr(dk, fn, lambda *a, **kw: None)
ya = sys.modules["yam_arena"] = types.ModuleType("yam_arena")  # mechkit's geometry and USD text work without the simulator
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")
tf.Box = type("Box", (), {})
tf.ObjectState = type("ObjectState", (), {})
import dataclasses  # noqa: E402

tf.Box = dataclasses.make_dataclass("Box", [("name", str), ("pos", tuple), ("size", tuple), ("mass", float, 0.0)], frozen=True)
spec = importlib.util.spec_from_file_location("mailer_task", HERE / "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
m = t.m
m.lock = lambda *a, **kw: None
T, H = t.T, t.H
HOLE = t.HOLE_X


class Env:
    def __init__(self, seen):
        self.seen = seen


class Box:
    joints = {"tab_l_joint": 0.0, "tab_r_joint": 0.0}
    pos = np.zeros(3)

    def rotation(self):
        return np.eye(3)


m.state = lambda env, name: Box()
t.Trial._spring = lambda self, *a, **kw: None
t.Trial.read = lambda self, env: env.seen


def seen(roll=0.0, outer=-15.0, edge=(-0.05, 0.0, 0.30), tips="up", ears=(0.0, 0.0), moved=0.0, tilt=0.0, clear=0.2, yaw=0.0):
    z = 0.0005 if tips == "in" else 0.30
    tp = (np.array([HOLE, t.TAB_Y, z]), np.array([HOLE, -t.TAB_Y, z])) if tips != "one" else (np.array([HOLE, t.TAB_Y, 0.0005]), np.array([HOLE, -t.TAB_Y, 0.30]))
    return t.Seen(outer, roll, edge, tp, ears, moved, tilt, clear, yaw)


EDGE_DOWN = (0.017, 0.0, 0.006)
ROLLED = dict(roll=120.0, outer=-3.0, edge=(0.017, 0.0, 0.10))
DOWN = dict(roll=179.5, outer=-0.8, edge=EDGE_DOWN)


def run(phases):
    tr = t.TRIAL
    tr.reset()
    for n, kw in phases:
        for _ in range(n):
            tr.step(Env(seen(**kw)))
    return tr.score


bad = 0


def check(label, got, want):
    global bad
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {label}")


HOLD_SHORT = 19
N = 3 * t.HOLD_STEPS
check("doing nothing", run([(N, {})]), 0.0)
check("0.25: rolled past 100 degrees, edge inside and below the top", run([(N, ROLLED)]), 0.25)
check("0.25: rolled 120 degrees, the wall leaning", run([(N, dict(ROLLED, outer=-15.0))]), 0.25)
check("0.5: down inside, no tab in", run([(N, DOWN | {"tips": "up"})]), 0.5)
check("0.75: one tab in", run([(N, DOWN | {"tips": "one"})]), 0.75)
check("0.75: both tabs in, the wall leaning 10 degrees", run([(N, DOWN | {"tips": "in", "outer": -10.0})]), 0.75)
check("0.75: both tabs in, a tuck flap opened 20 degrees", run([(N, DOWN | {"tips": "in", "ears": (20.0, 0.0)})]), 0.75)
check("0.75: both tabs in, a fingertip 2 cm from the box", run([(N, DOWN | {"tips": "in", "clear": 0.02})]), 0.75)
check("0.75: both tabs in, then 19 steps only", run([(HOLD_SHORT, DOWN | {"tips": "in"})]), 0.75)
check("1.0: both tabs in, wall upright, hands clear, still 1 s", run([(N, DOWN | {"tips": "in"})]), 1.0)
check("0.25: the panel only half over (edge not inside)", run([(N, dict(ROLLED, edge=(-0.05, 0.0, 0.3)))]), 0.0)

# the effect rule: the box stays near its place, upright (negative cases)
check("0: the box pushed 6 cm while all else is done", run([(N, DOWN | {"tips": "in", "moved": 0.06})]), 0.0)
check("1.0: the box slid 3 cm (inside the 5 cm)", run([(N, DOWN | {"tips": "in", "moved": 0.03, "yaw": 5.0})]), 1.0)
check("0: the box turned 15 degrees about the vertical", run([(N, DOWN | {"tips": "in", "yaw": 15.0})]), 0.0)
check("0: the box tipped 12 degrees (or lifted: moved counts height)", run([(N, DOWN | {"tips": "in", "tilt": 12.0})]), 0.0)
check("0: a tuck flap swung out 40 degrees", run([(N, DOWN | {"tips": "in", "ears": (0.0, 40.0)})]), 0.0)

# failures latch: carry the box off, do the act, bring it back
check("0: box dragged 8 cm for 3 steps, then back, then the whole act", run([(3, {"moved": 0.08}), (N, DOWN | {"tips": "in"})]), 0.0)
check("0: box tipped 20 degrees for 3 steps, then back, then the act", run([(3, {"tilt": 20.0}), (N, DOWN | {"tips": "in"})]), 0.0)
check("0: box turned 30 degrees for 3 steps, then back, then the act", run([(3, {"yaw": 30.0}), (N, DOWN | {"tips": "in"})]), 0.0)
check("0: a tab poked in its hole while the box was out of place, then back and down", run([(3, DOWN | {"moved": 0.08, "tips": "in"}), (N, DOWN)]), 0.0)

# the hands: the open lid and its tuck count as the box
check("0.75: the end done, a fingertip 2 cm from the lid (clear measured to the lid too)", run([(N, DOWN | {"tips": "in", "clear": 0.02})]), 0.75)
check("1.0: a hand 5 cm clear of everything", run([(N, DOWN | {"tips": "in", "clear": 0.05})]), 1.0)

# a tab only seats with the roll: the tips cannot reach the holes with the panel unrolled
check("a tab tip in a hole with the panel not rolled (poked): 0", run([(N, {"tips": "in"})]), 0.0)

print("FAILED" if bad else "all cases as the scoring text says")
sys.exit(1 if bad else 0)
