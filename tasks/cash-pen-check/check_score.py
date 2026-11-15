"""The scorer on hand-built trials, with no simulator: python3 check_score.py.

Loads task.py with stand-ins for the harness's modules, then feeds its real Trial step by step with made-up states (the honest check and the lazy
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
os.environ["CASH_TRACE"] = "0"

# -- stand-ins for the harness, enough for task.py to load ------------------------------------------------------------------------------------------
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
TOP = T.TOP
m.set_visible = lambda *a, **k: None

# -- states ------------------------------------------------------------------------------------------------------------------------------------------
NOTE0 = np.array([T.NOTE_AT[0], T.NOTE_AT[1], TOP + 0.0002])
CUP = np.array([T.CUP_AT[0], T.CUP_AT[1], TOP + 0.003])
UP = np.eye(3)
FAR_L, FAR_R = gk.hand(gk.PARKED["left"], opening=0.03), gk.hand(gk.PARKED["right"], opening=0.03)


PIN_H = (T.PIN[0] + T.PIN[1]) / 2  # a pressing fingertip's reported points, in the middle of the calibrated window


def pin_hand(note_pos, h=None):
    """The left's two fingertips pressed on the note's +y end, 59 mm apart across it (`h`: the reported points' height over the face)."""
    return gk.hand(note_pos + [0.0, 0.062, T.NOTE_T + (PIN_H if h is None else h)], opening=0.025)


def pen_hand(pen_pos):
    """The right holding the pen 30 mm below its top."""
    return gk.hand(pen_pos + [0.0, 0.0, T.PEN_L - 0.030], opening=T.PEN_R)


def state(note_pos, pen_pos, left, right, note_v=(0.0, 0.0, 0.0), pen_v=(0.0, 0.0, 0.0)):
    return {"note": gk.Pose(note_pos, UP, lin_vel=note_v), "pen": gk.Pose(pen_pos, UP, lin_vel=pen_v), "tips": gk.tips(left, right)}


def trial(slip=0.0, pin=True, strokes=1, length=0.015, mat="accept", pen_back=True, lifted=False, dropped=False, off_note=False, end_steps=30,
          pin_h=None, tip_h=0.0002, scribble=False, dabs=0, mat_shift=0.0, lift_h=0.0, hover_h=None):
    """The honest check as a list of states, with named changes: the note pinned, one stroke of `length` along -y over 20 steps while the note
    slips `slip` along x, the pen back in its cup, the note pushed onto `mat`, hands away, still for `end_steps`."""
    out = []
    note = NOTE0.copy()
    pen = CUP.copy()
    for _ in range(10):  # the pin goes down; the pen is taken
        out.append(state(note, pen, pin_hand(note, pin_h) if pin else FAR_L, FAR_R))
    for k in range(dabs):  # dabs: the felt touched down and lifted, not moving
        dab = note + [0.0, -0.030 + 0.004 * k, T.NOTE_T + tip_h]
        for _ in range(3):
            out.append(state(note, dab, pin_hand(note, pin_h) if pin else FAR_L, pen_hand(dab)))
        for _ in range(6):
            out.append(state(note, dab + [0.0, 0.0, 0.02], pin_hand(note, pin_h) if pin else FAR_L, pen_hand(dab + [0.0, 0.0, 0.02])))
    for s in range(strokes):
        start = note + [0.0, -0.008 - 0.005 * s, T.NOTE_T + tip_h]
        for k in range(21):  # the stroke: the tip on the paper, moving 0.75 mm a step (15 mm/s at 20 Hz)
            n_now = note + [slip * k / 20, 0.0, 0.0]
            along = (0.0025 if (k // 2) % 2 else 0.0) if scribble else length * k / 20  # scribble: back and forth over 2.5 mm
            tip = start + [slip * k / 20, -along, 0.0]
            if off_note:
                tip = tip + [0.05, 0.0, -T.NOTE_T - 0.0004]  # off the note's edge, down on the counter
            out.append(state(n_now, tip, pin_hand(n_now, pin_h) if pin else FAR_L, pen_hand(tip), note_v=(slip, 0, 0), pen_v=(0.0, -0.03, 0.0)))
        note = note + [slip, 0.0, 0.0]
        for _ in range(6):  # the tip lifted off
            out.append(state(note, start + [slip, -length, 0.02], pin_hand(note, pin_h) if pin else FAR_L, pen_hand(start + [slip, -length, 0.02])))
    if dropped:
        for k in range(5):
            out.append(state(note, np.array([0.3, -0.1, TOP + 0.10 - 0.05 * k]), FAR_L, FAR_R, pen_v=(0.0, 0.0, -1.0)))
        pen = np.array([0.3, -0.1, TOP + 0.003])
    elif pen_back:
        pen = CUP.copy()
    else:
        pen = np.array([0.28, -0.12, TOP + 0.10])
    right_after = pen_hand(pen) if not pen_back and not dropped else FAR_R
    for _ in range(10):  # the pen in its cup, let go; the left lets go of the note
        out.append(state(note, pen, FAR_L, right_after))
    target = {"accept": T.ACCEPT_AT, "reject": T.REJECT_AT, "none": (note[0], note[1])}[mat]
    end = np.array([target[0], target[1] + mat_shift, TOP + 0.0002]) + ([0.0, 0.0, 0.03] if lifted else [0.0, 0.0, 0.0])
    for k in range(20):  # pushed along by its end (`lift_h`: raised that much off the counter mid-way)
        pos = note + (end - note) * (k + 1) / 20 + [0.0, 0.0, lift_h * math.sin(math.pi * (k + 1) / 20)]
        out.append(state(pos, pen, FAR_L, gk.hand(pos + [0.0, T.NOTE_L / 2 + 0.01, 0.002], opening=0.0), note_v=(0.0, -0.3, 0.0)))
    for _ in range(end_steps):  # hands away, still (`hover_h`: the right's fingertips held that far over the note's middle)
        out.append(state(end, pen, FAR_L, right_after if hover_h is None else gk.hand(end + [0.0, 0.0, T.NOTE_T + hover_h], opening=0.0)))
    return out


def run(states) -> float:
    T.TRIAL.reset()
    feed = iter(states)
    cur = {}

    def st(env, name):
        return cur[name]

    m.state, m.fingertips = st, (lambda env: cur["tips"])
    for s in states:
        cur.update(s)
        T.TRIAL.step(None)
    return T.score(None)


CASES = {
    "the honest check: pinned, one 15 mm stroke, the pen back, the note on the green mat": (trial(), 1.0),
    "the same, the note left on the red mat (genuine note)": (trial(mat="reject"), 0.0),
    "the same, the note left where it was checked": (trial(mat="none"), 0.75),
    "the same, the pen left out of its cup": (trial(pen_back=False, mat="none"), 0.5),
    "pinned, but the note slid 3 mm under the stroke": (trial(slip=0.003, mat="none"), 0.25),
    "no pin: the note slides 8 mm with the pen (a smear)": (trial(pin=False, slip=0.008), 0.0),
    "a stroke of only 6 mm": (trial(length=0.006, mat="none"), 0.25),
    "four strokes": (trial(strokes=4), 0.0),
    "the tip pressed down on the counter, off the note": (trial(off_note=True), 0.0),
    "the note lifted 3 cm onto its mat": (trial(lifted=True), 0.0),
    "the pen dropped": (trial(dropped=True), 0.0),
    "nothing done": ([state(NOTE0, CUP, FAR_L, FAR_R)] * 40, 0.0),
    "the honest check, held only 0.9 s on the mat": (trial(end_steps=18), 0.75),
    "the pin hovering 2 mm over the note (not pressing)": (trial(pin_h=0.002), 0.0),
    "the felt hovering 0.9 mm over the paper (never touching)": (trial(tip_h=0.0009), 0.0),
    "a 2.5 mm scribble back and forth (12.5 mm of path)": (trial(scribble=True, mat="none"), 0.25),
    "five dabs, then one stroke": (trial(dabs=5), 0.0),
    "the note half off the green mat": (trial(mat_shift=0.10), 0.75),
    "the note lifted 6 mm off the counter on its way to the mat": (trial(lift_h=0.006), 0.0),
    "the note lifted 4 mm off the counter on its way to the mat": (trial(lift_h=0.004), 1.0),
    "a hand held 3.5 cm over the note's middle at the end": (trial(hover_h=0.035), 1.0),
    "a hand held 2.5 cm over the note's middle at the end": (trial(hover_h=0.025), 0.75),
    "the pin 0.5 mm over the note (not pressing)": (trial(pin_h=0.0005), 0.0),
    "the pin 0.9 mm over the note (not pressing)": (trial(pin_h=0.0009), 0.0),
    "the felt riding 0.4 mm over the paper (touching, never firm)": (trial(tip_h=0.0004), 0.25),
    "the felt riding 0.55 mm over the paper (touching, never firm)": (trial(tip_h=0.00055), 0.25),
}
bad = 0
for name, (states, want) in CASES.items():
    got = run(states)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {name}: {got} (text: {want})")
print("all as the scoring text says" if not bad else f"{bad} case(s) differ from the scoring text")
raise SystemExit(1 if bad else 0)
