"""The scorer on hand-built trials, with no simulator: python3 check_score.py.

Loads task.py with stand-ins for the harness's modules, then feeds its real Trial step by step with made-up states (the honest placing and the lazy
routes) and checks each score against the scoring text. Nothing here runs a simulation.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import os
import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
os.environ["CHAIR_TRACE"] = "0"

ya = types.ModuleType("yam_arena")
tf = types.ModuleType("yam_arena.task_file")


@dataclasses.dataclass(frozen=True)
class Box:
    name: str
    pos: tuple = (0.0, 0.0, 0.0)
    size: tuple = (0.1, 0.1, 0.1)
    mass: float | None = None
    yaw_deg: float = 0.0
    start_quat: tuple | None = None
    solver_iters: int = 32


@dataclasses.dataclass(frozen=True)
class ObjectState:
    pos: np.ndarray
    quat_xyzw: np.ndarray
    lin_vel: np.ndarray
    ang_vel: np.ndarray
    low: np.ndarray
    high: np.ndarray


tf.Box, tf.ObjectState, tf.TABLE_TOP = Box, ObjectState, {"station": 0.7475}
ya.task_file = tf
sys.modules["yam_arena"], sys.modules["yam_arena.task_file"] = ya, tf
dk = types.ModuleType("demokit")
dk.TABLE_TOP, dk.DT, dk.CONTROL_HZ = 0.7475, 0.05, 20
sys.modules["demokit"] = dk
sys.path.insert(0, str(HERE))
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

spec = importlib.util.spec_from_file_location("task", HERE / "task.py")
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)
TOP, SURF = T.TOP, T.SURF

RZ180 = np.diag([-1.0, -1.0, 1.0])          # the chair at the start: standing, facing the arms
FLIPPED = np.diag([1.0, -1.0, -1.0])        # turned over about world y: seat down, legs up, its back toward the arms
X_LAND = T.TX - 0.018 + T.SEAT_X / 2        # the seat centre's x with its rear edge 18 mm past the table's near edge


def Ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def chair(pos, R, v=(0, 0, 0)):
    return gk.Pose(pos, R, lin_vel=v)


def at(P, R, v=(0, 0, 0)):
    """The chair whose seat plate's middle point is P in the world, turned by R."""
    return chair(np.asarray(P, float) - R @ np.array([0.0, 0.0, T.SEAT_Z - T.SEAT_T / 2]), R, v)


def grips(c, opening=T.SEAT_T / 2):
    """Both hands pinching the seat plate at its side edges, fingers pointing in, jaws across the plate."""
    out = []
    for sy in (1.0, -1.0):
        mid = c.to_world([[0.0, sy * (T.SEAT_Y / 2 - T.IN), T.SEAT_Z - T.SEAT_T / 2]])[0]
        z, fing = c.rotation() @ [0, 0, 1.0], c.rotation() @ [0, -sy, 0.0]
        R = np.column_stack([z, np.cross(fing, z), fing])
        out.append(gk.hand(mid, R, opening=opening))
    return gk.tips(out[0], out[1])


PARK = gk.tips()


def st(c, tips):
    return {"chair": c, "tips": tips}


P0 = np.array([T.X0, 0.0, TOP + 0.002 + T.SEAT_Z - T.SEAT_T / 2])


def lift_phase(n=10):
    start = at(P0, RZ180)
    s = [st(start, PARK)] * 5 + [st(start, grips(start))] * 3
    for k in range(n):  # up 25 cm held
        c = at(P0 + [0, 0, 0.25 * (k + 1) / n], RZ180)
        s.append(st(c, grips(c)))
    return s


def turn_phase(n=10):
    s = []
    for k in range(n):  # turned over about world y through the seat plate's middle, held
        c = at(P0 + [0, 0, 0.25], Ry(180 * (k + 1) / n) @ RZ180)
        s.append(st(c, grips(c)))
    return s


def carry_phase(x_end, n=8):
    """Held, seat down: across at height, then down to 12 mm over the top."""
    s = []
    top_z = SURF + T.SEAT_T / 2 + 0.012 + 0.0
    for k in range(n):
        c = at([T.X0 + (x_end_p(x_end) - T.X0) * (k + 1) / n, 0.0, P0[2] + 0.25], FLIPPED)
        s.append(st(c, grips(c)))
    for k in range(n):
        c = at([x_end_p(x_end), 0.0, P0[2] + 0.25 + (top_z - P0[2] - 0.25) * (k + 1) / n], FLIPPED)
        s.append(st(c, grips(c)))
    return s


def x_end_p(x_origin):
    return x_origin


def honest(x_end=X_LAND, hold=50, drop_h=0.012, tilt=0.0, tail="clear", y=0.0, fall_v=0.0):
    s = lift_phase() + turn_phase()
    s += carry_phase(x_end)
    R = Ry(tilt) @ FLIPPED
    held = at([x_end, y, SURF + T.SEAT_T / 2 + drop_h], R)
    s += [st(held, grips(held))] * 3
    down = at([x_end, y, SURF + T.SEAT_T / 2 + math.sin(math.radians(abs(tilt))) * T.SEAT_X / 2], R)  # tilted: resting on one edge
    s += [st(down, grips(down, opening=0.02))] * 2          # fingers opened
    if fall_v:
        s += [st(at([x_end, y, SURF + T.SEAT_T / 2 + drop_h / 2], R, v=(0, 0, -fall_v)), PARK)]
    after = [st(down, PARK)] if tail == "clear" else [st(down, grips(down, opening=0.02))] if tail == "near" else [st(down, grips(down))]
    return s + after * hold


def run(states) -> float:
    T.TRIAL.reset()
    cur = {}
    m.state, m.fingertips = (lambda env, name: cur[name]), (lambda env: cur["tips"])
    for s in states:
        cur.update(s)
        T.TRIAL.step(None)
    return T.score(None)


def fall(x, z0, steps=6):
    """The chair, seat down, let go with its seat plate z0 over the top and falling."""
    out, z, v = [], z0, 0.0
    for _ in range(steps):
        v += 9.81 * 0.05
        z = max(z - v * 0.05, SURF + T.SEAT_T / 2)
        out.append(st(at([x, 0.0, z], FLIPPED, v=(0, 0, -v)), PARK))
    return out


start = at(P0, RZ180)
side = chair([T.X0, 0.0, TOP + T.SEAT_Y / 2], Ry(90) @ RZ180)


def one_hand_lift():
    s = [st(start, PARK)] * 5
    for k in range(10):
        c = at(P0 + [0, 0, 0.25 * (k + 1) / 10], RZ180)
        g = grips(c)
        s.append(st(c, gk.tips(g[:12], gk.hand(gk.PARKED["right"], opening=0.03))))
    return s + [st(c, PARK)] * 20


def flat(x, y=0.0):
    return at([x, y, SURF + T.SEAT_T / 2], FLIPPED)


floor_flat = at([0.0, 0.0, TOP + 0.235], FLIPPED)  # origin: seat face down on the floor... legs up, back toward the arms
dropped_on_top = lift_phase() + turn_phase() + carry_phase(X_LAND)[:4] + fall(X_LAND, SURF + 0.20) + [st(flat(X_LAND), PARK)] * 50
set_down_floor_upright = lift_phase() + [st(at(P0, RZ180), grips(at(P0, RZ180)))] * 4 + [st(at(P0, RZ180), PARK)] * 50
turned_on_floor = lift_phase() + turn_phase() + [st(floor_flat, grips(floor_flat))] * 3 + [st(floor_flat, PARK)] * 50
upright_on_top = lift_phase() + [st(chair([T.TX + 0.15, 0.0, SURF + 0.002], RZ180), PARK)] * 50
dragged = honest()[:-50] + [st(flat(X_LAND + 0.01 * k), PARK) for k in range(1, 10)] * 6
legs_down_on_top = lift_phase() + turn_phase()[:5] + [st(at([T.TX + 0.25, 0.0, SURF + 0.01], Ry(120) @ RZ180), PARK)] * 30
feet_on_top_held = lift_phase() + [st(at([T.TX + 0.15, 0.0, SURF + 0.002 + T.SEAT_Z - T.SEAT_T / 2], RZ180), grips(at([T.TX + 0.15, 0.0, SURF + 0.002 + T.SEAT_Z - T.SEAT_T / 2], RZ180)))] * 5
slides_off = honest(hold=10) + [st(at([X_LAND - 0.10 - 0.05 * k, 0.0, max(TOP + 0.01, SURF + T.SEAT_T / 2 - 0.06 * k)], FLIPPED, v=(0, 0, -1.5)), PARK) for k in range(1, 12)] * 2

CASES = {
    "the honest placing: lifted, turned over, seat down on the top, rear edge 18 mm past the edge, let go, hands clear, still 2 s": (honest(), 1.0),
    "the same, let go 6 cm over the top (falls at 1.1 m/s)": (honest(drop_h=0.06, fall_v=1.1), 1.0),
    "the same, let go 12 cm over the top (falls at 1.5 m/s)": (honest(drop_h=0.12, fall_v=1.5), 0.0),
    "the same, set down from 25 mm instead of 12 mm (let go that close to the top)": (honest(drop_h=0.025), 1.0),
    "the same, rear edge 25 mm inside the edge (the back would stand on the top)": (honest(x_end=T.TX + 0.025 + T.SEAT_X / 2), 0.0),
    "the same, rear edge 25 mm past the edge": (honest(x_end=T.TX - 0.025 + T.SEAT_X / 2), 1.0),
    "the same, rear edge 45 mm past the edge": (honest(x_end=T.TX - 0.045 + T.SEAT_X / 2), 0.75),
    "seat tilted 8 degrees on the top": (honest(tilt=8.0), 0.5),
    "seat tilted 20 degrees on the top": (honest(tilt=20.0), 0.5),
    "seat tilted 3 degrees on the top": (honest(tilt=3.0), 1.0),
    "a hand left 2 cm off the seat edge at the end": (honest(tail="near"), 0.75),
    "a hand still pinching the seat at the end": (honest(tail="pinch"), 0.5),
    "the end still held only 1 s": (honest(hold=26), 0.75),
    "nothing done": ([st(start, PARK)] * 60, 0.0),
    "the chair left standing on the floor, hands clear": ([st(start, PARK)] * 60, 0.0),
    "the chair knocked onto its side on the floor": ([st(start, PARK)] * 5 + [st(side, PARK)] * 50, 0.0),
    "one hand lifts it 25 cm, then it is put down": (one_hand_lift(), 0.0),
    "lifted by both hands, set back down upright on the floor": (set_down_floor_upright, 0.25),
    "lifted, turned over, set down upside down on the floor beside the table": (turned_on_floor, 0.5),
    "lifted and put upright on the top, never turned": (upright_on_top, 0.25),
    "turned over and dropped from 20 cm onto the top": (dropped_on_top, 0.0),
    "placed, then the seat dragged 9 cm along the top": (dragged, 0.0),
    "placed, then pushed off the table's edge to the floor": (slides_off, 0.0),
    "turned, then put down on its side on the top, released and still": (legs_down_on_top, 0.25),
    "carried upright over the top and lowered until its feet touch it, held": (feet_on_top_held, 0.0),
}
if os.environ.get("DBG"):
    st_, = [v for k, v in CASES.items() if k.startswith(os.environ["DBG"])]
    cur = {}
    T.TRIAL.reset()
    m.state, m.fingertips = (lambda env, name: cur[name]), (lambda env: cur["tips"])
    for i, x in enumerate(st_[0]):
        cur.update(x)
        T.TRIAL.step(None)
        t = T.TRIAL
        print(i, t.lifted, t.turned, bool(t.dropped), bool(t.scraped), bool(t.dragged), t.air, t.last)
    raise SystemExit
bad = 0
for name, (states, want) in CASES.items():
    got = run(states)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {name}: {got} (text: {want})")
print("all as the scoring text says" if not bad else f"{bad} case(s) differ from the scoring text")
raise SystemExit(1 if bad else 0)
