"""The scorer on hand-built states, with no simulator: python3 check_score.py. Runs task.py's own `Trial`, `interleaved`, `in_stack` and their helpers,
read from its source, against stand-ins for the simulator's state; the joints and the hands-clear reading are stubbed."""

import ast
import contextlib
import io
import json
import math
import os
import pathlib
import types

import numpy as np

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {"np": np, "math": math, "json": json, "os": os, "d": types.SimpleNamespace(TABLE_TOP=0.0)}


class Stub:
    """mechkit stand-in: `state` reads the hand-built record; anything else is not needed by the scorer."""

    @staticmethod
    def state(env, name):
        return env[name]

    def __getattr__(self, name):
        raise AttributeError(name)


ns["m"] = Stub()
for node in ast.parse(src).body:  # constants, functions and the Trial class; scene building that needs the simulator is skipped
    if isinstance(node, (ast.Assign, ast.AnnAssign, ast.FunctionDef, ast.ClassDef)):
        try:
            exec(compile(ast.Module([node], []), "task.py", "exec"), ns)
        except Exception:
            pass
ns["_guide"] = lambda env, on: False
ns["_clamp"] = lambda env, on: False
ns["hands_gap"] = lambda env, *a: env["gap"]
X0, X_TOUCH, SEAT_X, LS, HY, ZH, PITCH = (ns[k] for k in ("X0", "X_TOUCH", "SEAT_X", "LS", "HY", "ZH", "PITCH"))


class Part:
    def __init__(self, pos, R):
        self.pos, self.R, self.lin_vel = np.asarray(pos, float), np.asarray(R, float), np.zeros(3)

    def rotation(self):
        return self.R


def rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def camera(dx=0.0, roll=0.0, flip=False):
    """The camera at its mounted pose, moved dx along the hole line, turned `roll` degrees about the hole line, or turned round (lens away)."""
    R = ns["rx"](math.radians(roll)) @ (rz(180.0) if flip else np.eye(3)) @ ns["RX90"]
    hinge = np.array([ns["HX"], HY, ZH])
    pos = hinge + ns["rx"](math.radians(roll)) @ (ns["CAM_NOM"] - hinge) + [dx, 0.0, 0.0]
    return Part(pos, R)


def screw(tip_x, turn=0.0):
    """The screw on the holes' line, tip at tip_x, knob's face LS behind it, turned `turn` radians about its axis."""
    return Part([tip_x - LS, HY, ZH], ns["rx"](turn))


def run(states, gap=0.10):
    """Step a fresh Trial through (camera, screw) states; return the score the task reports at the end and the trial."""
    tr = ns["Trial"]()
    with contextlib.redirect_stdout(io.StringIO()):  # the trial's own progress lines
        tr.reset()
        for cam, scr in states:
            tr.step({"camera": cam, "screw": scr, "gap": gap})
    return (tr.final if tr.done else tr.live_tier), tr


def turned_in(cam, turns, follow=True, start=X_TOUCH):
    """Push the tip to the nut, then turn in 10-degree steps; with `follow`, the tip goes in as far as the turns allow (the thread draws it)."""
    out = [(cam, screw(start))]
    a, n = 0.0, int(round(turns * 36))
    for _ in range(n):
        a += math.radians(10.0)
        credit = X_TOUCH + a / (2 * math.pi) * PITCH + 0.0005
        out.append((cam, screw(min(credit - 0.0001, SEAT_X) if follow else start, a)))
    return out


def turns_to(test):
    for k in range(1, 400):
        s, tr = run(turned_in(camera(), k / 36))
        if test(s, tr):
            return k / 36
    return float("nan")


rows = []
full = turned_in(camera(), 1.6)
rows.append(("full: turned home, hands 10 cm clear, still 1 s", run(full + [full[-1]] * 25)[0], 1.0))
rows.append(("seated, a hand 3 cm from the camera", run(full + [full[-1]] * 25, gap=0.03)[0], 0.75))
rows.append(("push_seat: tip pushed to the seat, not turned", run([(camera(), screw(X_TOUCH)), (camera(), screw(SEAT_X))] + [(camera(), screw(SEAT_X))] * 25)[0], 0.5))
rows.append(("no_turn: tip at the nut, let go", run([(camera(), screw(X_TOUCH))] * 25)[0], 0.5))
wrong = [(camera(roll=6.0), screw(X_TOUCH)), (camera(roll=6.0), screw(SEAT_X)), (camera(), screw(SEAT_X))] + [(camera(), screw(SEAT_X))] * 25
rows.append(("wrong order: camera rolled 6 deg, screw pushed to the seat, camera squared after", run(wrong)[0], 0.5))
rows.append(("far side: screw on the hole line, knob's face 21 mm past the first finger", run([(camera(), screw(X0 + 0.021 + LS))] * 25)[0], 0.25))
rows.append(("turned 1.6 turns in, then turned back out to the start", run(full + [(camera(), screw(X_TOUCH, a)) for a in np.radians(576 - 10 * np.arange(1, 59))] + [(camera(), screw(X_TOUCH, np.radians(-4)))] * 25)[0], 0.5))
rows.append(("camera only, mounted", run([(camera(), screw(X0 - 0.06))] * 25)[0], 0.25))
rows.append(("camera turned round, lens away from the gripper's fingers", run([(camera(flip=True), screw(X_TOUCH))] * 25)[0], 0.0))
rows.append(("camera 2 mm along the hole line from its mounted pose", run([(camera(dx=0.002), screw(X_TOUCH))] * 25)[0], 0.0))
rows.append(("nothing", run([(Part([0.21, -0.16, 0.017], np.eye(3)), screw(X0 - 0.06))] * 25)[0], 0.0))

bad = 0
for name, got, want in rows:
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:4.2f} (want {want:4.2f})  {name}")
print(f"turns for 0.75: {turns_to(lambda s, tr: s >= 0.75):.2f}; turns to seated: {turns_to(lambda s, tr: tr.snug):.2f}")
raise SystemExit(1 if bad else 0)
