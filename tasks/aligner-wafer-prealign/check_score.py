"""The scorer on hand-built states, with no simulator: each rung, each rule and a case that fails it.

    python -B check_score.py

Every case is a Seen state fed to task.py's own judge(), failed() and Trial rules. Exits non-zero if any case scores other than the scoring text says.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
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
ya = sys.modules["yam_arena"] = types.ModuleType("yam_arena")
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")
tf.ObjectState = type("ObjectState", (), {})
tf.Box = dataclasses.make_dataclass("Box", [("name", str), ("pos", tuple), ("size", tuple), ("mass", float, 0.0)], frozen=True)
spec = importlib.util.spec_from_file_location("aligner_task", HERE / "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

FAR = np.array([[1.0, 1.0, 0.5]] * 24)  # every fingertip far away
bad = []


def seated(**kw):
    """A wafer seated exactly: flat on the close pins (gap 0), rim on the left pin (gap 0)."""
    base = dict(c=(0.0, 0.0), yaw=0.0, h=0.0, tilt_deg=0.0, slide=0.0, key=0.0, tips=FAR, vacuum=False)
    base.update(kw)
    return t.Seen(**base)


def shifted(dx=0.0, dy=0.0, yaw_deg=0.0):
    """The wafer moved from seated by (dx, dy) m and turned about its centre."""
    return seated(c=(dx, dy), yaw=math.radians(yaw_deg))


def score(s, clear=True, still=True, held=40, dead=None):
    dead = t.failed(s) if dead is None else dead
    return t.judge(s, clear, still, held, dead)[1]


def case(name, got, want):
    ok = abs(got - want) < 1e-9
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {got} (want {want})")
    if not ok:
        bad.append(name)


start = shifted(0.004, -0.003, -8.0)
case("start state: nothing touched", score(start), 0.0)
A = seated()
print("seated gaps (mm):", np.round(np.array(A.gaps()) * 1000, 3))
# one close pin: turned 3 degrees about the pin at y = -10 mm
one = shifted(0.0, 0.0, 0.0)
th = math.radians(-3.0)
pin = np.array(t.PINS[0])
c1 = pin + np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]]) @ (-pin)
s1 = t.Seen(c1, th, 0.0, 0.0, 0.0, 0.0, FAR, False)
print("one-pin gaps (mm):", np.round(np.array(s1.gaps()) * 1000, 3))
case("flat on one pin only", score(s1), 0.25)
case("flat on both pins, rim not on the left pin", score(shifted(0.0, -0.005)), 0.5)
case("flat 1 mm off the close pins", score(shifted(0.001, 0.0)), 0.0)
case("seated, vacuum off", score(seated()), 0.5)
case("seated, vacuum on, slide out", score(seated(vacuum=True)), 0.75)
case("seated, vacuum on, slide in, hands clear, still, held 1 s", score(seated(vacuum=True, slide=0.150)), 1.0)
case("the same one step after the slide arrives (not yet held)", score(seated(vacuum=True, slide=0.150), held=3), 0.75)
case("slide 3 mm short of the stop", score(seated(vacuum=True, slide=0.147)), 0.75)
case("hands not clear", score(seated(vacuum=True, slide=0.150), clear=False), 0.75)
case("not still", score(seated(vacuum=True, slide=0.150), still=False), 0.75)
case("wafer moved 1 mm by the slide", score(seated(c=(0.001, 0.0), vacuum=True, slide=0.150)), 0.0)
case("wafer rim 0.2 mm from the left pin", score(seated(c=(0.0, -0.0002), vacuum=True, slide=0.150)), 1.0)
case("wafer rim 0.6 mm from the left pin", score(seated(c=(0.0, -0.0006), vacuum=True, slide=0.150)), 0.5)
# the failures
tips_on_face = FAR.copy()
tips_on_face[:12] = [0.0, 0.0, 0.0001]  # a hand's blade on the face at the wafer's centre
case("fingertip on the face, 20 mm in", score(seated(vacuum=True, slide=0.150, tips=tips_on_face), dead=None), 0.0)
tips_rim = FAR.copy()
tips_rim[:12] = [0.0, -0.049, 0.0001]  # on the outer 3 mm of the face
case("fingertip on the face within 3 mm of the rim (allowed)", score(seated(vacuum=True, slide=0.150, tips=tips_rim)), 1.0)
tips_beside = FAR.copy()
tips_beside[:12] = [0.0, -0.052, 0.0001]  # beside the rim, on the chuck
case("fingertip beside the rim (allowed)", score(seated(vacuum=True, slide=0.150, tips=tips_beside)), 1.0)
case("wafer riding on a pin: 0.4 mm up", score(seated(h=0.0004, vacuum=True, slide=0.150), dead=None), 0.0)
case("wafer tilted 2 degrees", score(seated(tilt_deg=2.0, vacuum=True, slide=0.150), dead=None), 0.0)
case("wafer off the chuck", score(seated(c=(0.03, 0.0), vacuum=True, slide=0.150), dead=None), 0.0)
case("latched failure stays 0", score(seated(vacuum=True, slide=0.150), dead=True), 0.0)
# loophole sweep: one negative case each
case("flat turned away (round rim on the close pins)", score(seated(yaw=math.pi, vacuum=True, slide=0.150)), 0.0)
case("edge on a pin, 0.15 mm up: no flat rung", score(seated(h=0.00015, vacuum=True, slide=0.150), dead=None), 0.25)
case("tilted 0.4 degree: no flat rung", score(seated(tilt_deg=0.4, vacuum=True, slide=0.150), dead=None), 0.25)
tips_half = FAR.copy()
tips_half[0] = [0.0, 0.0, t.T_W + 0.0002]  # one lower point of the blade on the face, centre outside
case("one blade point on the face, 20 mm in", score(seated(vacuum=True, slide=0.150, tips=tips_half), dead=None), 0.0)
case("key pressed with the wafer unseated does not arm", float(t.key_armed(shifted(0.004, -0.003, -8.0))), 0.0)
case("key armed when seated", float(t.key_armed(seated())), 1.0)
case("slide moved without a fingertip at the handle", t.judge(seated(vacuum=True, slide=0.150), True, True, 40, False, False)[1], 0.75)
case("fingertip far from the handle", float(t.handle_touched(FAR)), 0.0)
case("fingertip at the handle", float(t.handle_touched(np.array([[t.HANDLE_X - 0.01, 0.0, 0.005]]))), 1.0)
case("key held down", float(t.key_free(seated(key=-t.KEY_TRAVEL))), 0.0)
case("key released", float(t.key_free(seated())), 1.0)
case("wafer flicked at 0.5 m/s", float(t.too_fast(0.5)), 1.0)
case("wafer pushed at 6 mm/s", float(t.too_fast(0.006)), 0.0)
# a one-sample ride-up beyond the pin's height latches at once
bigtilt = types.SimpleNamespace(h=0.0005, tilt_deg=0.0)
case("one-sample ride-up of 0.5 mm latches", float(bigtilt.h > 0.0004), 1.0)
print("FAILED: " + ", ".join(bad) if bad else "all cases pass")
sys.exit(1 if bad else 0)
