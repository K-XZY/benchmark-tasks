"""The scorer on hand-built states, with no simulator: where the cover is while the egg comes out, the egg's path from the pan to the blade to
the heel, and the cover back on the pan at the end.

    python check_score.py

Exits non-zero if any case gives another value than the scoring text.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys
import types
from dataclasses import dataclass

import numpy as np

sys.modules["yam_arena"] = types.ModuleType("yam_arena")  # the harness is not needed: only its names, so that mechkit's meshes can be built
sys.modules["yam_arena"].__path__ = []
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")


@dataclass(frozen=True)
class Box:  # the harness's shape, its fields only
    name: str
    pos: tuple
    color: tuple = (0.5, 0.5, 0.5)
    mass: float | None = None
    yaw: float = 0.0
    size: tuple = (0.04, 0.04, 0.04)


@dataclass(frozen=True)
class ObjectState:
    pos: object
    quat_xyzw: object
    lin_vel: object
    ang_vel: object
    low: object
    high: object


tf.TABLE_TOP, tf.Box, tf.ObjectState = {"station": 0.0}, Box, ObjectState
dk = sys.modules["demokit"] = types.ModuleType("demokit")  # the scripted demo's kit is not needed to score
dk.__getattr__ = lambda name: (lambda *a, **k: None)
sys.modules["yam_arena"].ik = sys.modules["yam_arena.ik"] = types.ModuleType("yam_arena.ik")  # the arm IK: demo only
sys.path.insert(0, str(pathlib.Path(__file__).parent))
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

bad = 0


def check(label, got, want):
    global bad
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {label}: {got} (want {want})")


def q_y(deg):  # a turn about y
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def part(pos, q=(0.0, 0.0, 0.0, 1.0)):
    z = np.zeros(3)
    return t.m.PartState(pos=np.asarray(pos, dtype=float), quat_xyzw=np.asarray(q, dtype=float), lin_vel=z, ang_vel=z, low=z, high=z)


TRAY = np.array([t.TX, t.TY, t.TOP + 0.0003])
EGG0 = np.array([t.EGG_X[0], t.TY, t.ZF + t.EGG_T / 2])
EGG2 = np.array([t.EGG_X[1], t.TY, t.ZF + t.EGG_T / 2])
MUF = np.array([t.MUF0[0], t.MUF0[1], t.TOP + 0.0012])
ON_HEEL = MUF + np.array([0.0, 0.0, t.MUF_H + t.EGG_T / 2])
TURNER_REST = np.array([t.TURNER0[0], t.TURNER0[1], t.TOP + 0.03])

# the cover's places
SEATED = (TRAY + np.array([0.0, 0.0, t.DEPTH]), (0.0, 0.0, 0.0, 1.0))
HELD_BESIDE = (np.array([0.34, 0.21, t.TOP + 0.125]), (0.0, 0.0, 0.0, 1.0))  # the demo's hold, left of the pan
HELD_HIGH_OVER = (TRAY + np.array([0.0, 0.0, t.DEPTH + 0.15]), (0.0, 0.0, 0.0, 1.0))  # lifted straight up 15 cm over the pan
HELD_UPRIGHT = (np.array([0.40, 0.25, t.TOP + 0.09]), (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)))  # stood on its long edge, held behind the pan
ON_COUNTER = (np.array([0.34, 0.21, t.TOP]), (0.0, 0.0, 0.0, 1.0))  # set down flat on the counter beside the pan
ON_COUNTER_FRONT = (np.array([0.40 - 0.35, t.TY, t.TOP]), (0.0, 0.0, 0.0, 1.0))  # set down flat in front of the pan
SLID_HALF = (TRAY + np.array([0.0, 0.12, t.DEPTH]), (0.0, 0.0, 0.0, 1.0))  # slid half off along the rim, still on it
LEANED = (TRAY + np.array([0.0, t.OUT_W / 2 + 0.05, t.DEPTH / 2]), (math.sin(math.radians(-25)), 0.0, 0.0, math.cos(math.radians(-25))))  # one edge on the rim, the other on the counter
JUST_OVER = (TRAY + np.array([0.0, 0.0, t.DEPTH + 0.005]), (0.0, 0.0, 0.0, 1.0))  # lifted only 5 mm over its seat


def st(egg, lid, turner=None, egg2=EGG2, tray=TRAY):
    turner = TURNER_REST if turner is None else turner
    return {"egg": part(egg), "egg2": part(egg2), "lid": part(*lid), "turner": part(turner), "muffin": part(MUF), "tray": part(tray)}


def on_blade(at):
    """The egg riding the blade, the blade's origin at `at`."""
    return np.asarray(at, dtype=float) + np.array([0.0, 0.0, t.BT + t.EGG_T / 2]), np.asarray(at, dtype=float)


class Env:
    class unwrapped:
        step_dt = 0.05


def run(states):
    tr = t.TRACK
    tr.reset()
    for s in states:
        t.m.state = lambda env, name, s=s: s[name]
        tr.step(Env)
    return t.score([])


def serve(lid_out, lid_end, path=None, steps_each=6):
    """The cover seated, then at `lid_out` while the egg rides the blade out of the pan and over the heel and slides onto it, then `lid_end`."""
    out = [st(EGG0, SEATED)] * steps_each + [st(EGG0, lid_out)] * steps_each
    ways = path or [np.array([EGG0[0], t.TY, t.ZF]), np.array([EGG0[0], t.TY, t.ZRIM + 0.03]), np.array([t.TX - 0.25, t.TY, t.ZRIM + 0.03]),
                    np.array([MUF[0], MUF[1], MUF[2] + t.MUF_H + 0.012])]
    for w in ways:
        e, k = on_blade(w)
        out += [st(e, lid_out, k)] * steps_each
    out += [st(ON_HEEL, lid_out)] * steps_each + [st(ON_HEEL, lid_end)] * steps_each
    return out


check("doing nothing scores 0", run([st(EGG0, SEATED)] * 40), 0.0)
check("cover held level beside the pan, egg served, cover seated again: 1.0", run(serve(HELD_BESIDE, SEATED)), 1.0)
check("cover lifted straight up 15 cm over the pan and held there, egg served, cover seated: 1.0", run(serve(HELD_HIGH_OVER, SEATED)), 1.0)
check("cover stood upright on its long edge behind the pan, held, egg served, cover seated: 1.0", run(serve(HELD_UPRIGHT, SEATED)), 1.0)
check("cover set down flat on the counter beside the pan, egg served, cover seated: 1.0", run(serve(ON_COUNTER, SEATED)), 1.0)
check("cover set down flat on the counter in front of the pan, egg served, cover seated: 1.0", run(serve(ON_COUNTER_FRONT, SEATED)), 1.0)
check("cover held beside the pan, egg served, cover never put back: 0.6", run(serve(HELD_BESIDE, HELD_BESIDE)), 0.6)
check("cover set on the counter, egg served, cover left there: 0.6", run(serve(ON_COUNTER, ON_COUNTER)), 0.6)
check("cover slid half off along the rim, egg brought out under it: 0 (void)", run(serve(SLID_HALF, SEATED)), 0.0)
check("cover leaned with one edge on the rim, egg brought out: 0 (void)", run(serve(LEANED, SEATED)), 0.0)
check("cover lifted only 5 mm over its seat, egg brought out: 0 (void)", run(serve(JUST_OVER, SEATED)), 0.0)
check("cover put back but 3 mm proud of its seat: 0.6", run(serve(HELD_BESIDE, (SEATED[0] + np.array([0.0, 0.0, 0.003]), SEATED[1]))), 0.6)
check("cover put back 10 mm off across: 0.6", run(serve(HELD_BESIDE, (SEATED[0] + np.array([0.010, 0.0, 0.0]), SEATED[1]))), 0.6)
e_up, k_up = on_blade(np.array([t.TX - 0.25, t.TY, t.ZRIM + 0.03]))
check("egg out on the blade and held there, cover beside: 0.3", run(serve(HELD_BESIDE, SEATED)[:24] + [st(e_up, HELD_BESIDE, k_up)] * 12), 0.3)
carried = [st(EGG0, SEATED)] * 6 + [st(EGG0, HELD_BESIDE)] * 6 + [st(EGG0 + np.array([0.0, 0.0, 0.10]), HELD_BESIDE)] * 15 + [st(ON_HEEL, HELD_BESIDE)] * 6 + [st(ON_HEEL, SEATED)] * 6
check("egg carried out by the fingers (off pan, blade and heel for 0.75 s), cover seated: 0", run(carried), 0.0)
dropped = serve(HELD_BESIDE, SEATED, path=[np.array([EGG0[0], t.TY, t.ZF]), np.array([EGG0[0], t.TY, t.ZRIM + 0.03]),
                                            np.array([MUF[0], MUF[1], MUF[2] + t.MUF_H + 0.08])])
check("egg slid off the blade 8 cm above the heel's top: 0", run(dropped), 0.0)
over_heel = np.array([MUF[0], MUF[1], MUF[2] + t.MUF_H + 0.006])
e_oh, k_oh = on_blade(over_heel)
pinched = serve(HELD_BESIDE, SEATED)[:30] + [st(e_oh, HELD_BESIDE, k_oh)] * 6 + [st(e_oh + np.array([0.0, 0.0, 0.02]), HELD_BESIDE, k_oh)] * 2 \
    + [st(ON_HEEL, HELD_BESIDE)] * 6 + [st(ON_HEEL, SEATED)] * 6
check("blade held over the heel, the fingers lift the egg 2 cm off it and set it on the heel within 0.1 s: 0", run(pinched), 0.0)
mid = serve(HELD_BESIDE, SEATED, path=[np.array([EGG0[0], t.TY, t.ZF]), np.array([EGG0[0], t.TY, t.ZRIM + 0.03]),
                                        np.array([MUF[0], MUF[1], MUF[2] + t.MUF_H + 0.025])])
check("egg slid off the blade 2.5 cm above the heel's top (over the 20 mm limit): 0", run(mid), 0.0)
held_off = (SEATED[0] + np.array([0.006, 0.006, 0.0]), SEATED[1])
check("cover put back 8.5 mm off on the diagonal (6 mm each way): 0.6", run(serve(HELD_BESIDE, held_off)), 0.6)
e_no, k_no = on_blade(np.array([t.TX - t.OUT_L / 2 - 0.010, t.TY, t.ZRIM + 0.03]))
check("egg out on the blade but held only 10 mm past the pan's outline: 0", run(serve(HELD_BESIDE, SEATED)[:24] + [st(e_no, HELD_BESIDE, k_no)] * 12), 0.0)
beside = np.array([t.TX - t.OUT_L / 2 - 0.06, t.TY, t.ZRIM + 0.03])
e_pb, k_pb = on_blade(beside)
pinch_blade = [st(EGG0, SEATED)] * 6 + [st(EGG0, HELD_BESIDE, beside)] * 6 + [st(EGG0 + np.array([0.0, 0.0, 0.09]), HELD_BESIDE, beside)] * 3 \
    + [st(e_pb, HELD_BESIDE, k_pb)] * 6
for w in (np.array([t.TX - 0.25, t.TY, t.ZRIM + 0.03]), np.array([MUF[0], MUF[1], MUF[2] + t.MUF_H + 0.012])):
    e, k = on_blade(w)
    pinch_blade += [st(e, HELD_BESIDE, k)] * 6
pinch_blade += [st(ON_HEEL, HELD_BESIDE)] * 6 + [st(ON_HEEL, SEATED)] * 6
check("egg lifted out of the pan by the fingers in 0.15 s, laid on the blade beside the pan, then served: 0", run(pinch_blade), 0.0)
gone2 = serve(HELD_BESIDE, SEATED)
gone2[-1] = dict(gone2[-1], egg2=part(np.array([0.40, -0.10, t.TOP + 0.02])))
edge_down = serve(HELD_BESIDE, SEATED)[:-12]
e_tilt = MUF + np.array([0.0, 0.055, t.MUF_H * 0.5 + 0.01])
for _ in range(12):
    s_ = dict(edge_down[-1]); s_["egg"] = part(e_tilt, q_y(35.0)); edge_down.append(s_)
check("egg slid off to the side of the heel, tipped 35 degrees with its edge on the counter: 0 (void)", run(edge_down), 0.0)
check("the other egg taken out of the pan: 0 (void)", run(gone2), 0.0)
sys.exit(1 if bad else 0)
