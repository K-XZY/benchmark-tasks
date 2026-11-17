"""The scorer on hand-built states, with no simulator: each rung, each rule and a case that breaks it.

    python -B check_score.py

Every case is a run of steps fed to task.py's own Trial, all the same state, so the end-state hold (2 s at rest, hands clear) is counted as in a
run. Exits non-zero if any case scores other than the scoring text says.
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
ya = sys.modules["yam_arena"] = types.ModuleType("yam_arena")  # mechkit's geometry and USD text work without the simulator
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")
tf.Box = dataclasses.make_dataclass("Box", [("name", str), ("pos", tuple), ("size", tuple), ("mass", float, 0.0)], frozen=True)
tf.ObjectState = dataclasses.make_dataclass("ObjectState", [("pos", object, None), ("quat_xyzw", object, None)], frozen=True)
tf.TABLE_TOP = {"station": 0.7475, "tabletop": 0.75}
spec = importlib.util.spec_from_file_location("reshelve_task", HERE / "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
TOP = t.TOP


class S:
    def __init__(self, pos, R, vel=0.0):
        self.pos, self._R = np.asarray(pos, float), np.asarray(R, float)
        self.lin_vel = np.array([vel, 0.0, 0.0])
        self.ang_vel = np.zeros(3)

    def rotation(self):
        return self._R


def rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


AWAY = np.array([[0.32, 0.305, TOP + 0.26]] * 12 + [[0.32, -0.305, TOP + 0.26]] * 12)  # both hands at the ready pose


def run_upright(lean=3.0, which=None, tilt_one=None):
    """The run leaning `lean` degrees on the green book, each book on its left tail corner."""
    out = []
    faces = t.upright_faces()
    for i, tt in enumerate(t.RUN_T):
        a = tilt_one if (which == i and tilt_one is not None) else lean
        R = t.rot_x(a)
        c_l = np.array([0.0, tt / 2, -t.RUN_H / 2 - t.rack(tt) / 2])  # the left tail corner
        foot = np.array([t.X_S + t.RUN_D / 2, faces[i][1], TOP + t.Z_S])
        out.append((foot - R @ c_l, R))
    return out


def green(proud=0.0, tilt=3.0, y=None, R=None, z=None):
    R = t.rot_x(tilt) if R is None else R
    y = t.SLOT_Y if y is None else y
    p = np.array([t.X_S - proud + t.NEW_D / 2, y, TOP + t.Z_S])
    tail = R @ np.array([0.0, 0.0, -t.NEW_H / 2])
    p = p - tail
    if z is not None:
        p[2] = z
    return p, R


END0 = np.array([t.X_S + t.END_X / 2, t.Y_PLATE, TOP + t.Z_B])


def state(new=None, run=None, end=None, tips=AWAY, vel=0.0, body=None):
    new = green() if new is None else new
    run = run_upright() if run is None else run
    end = (END0, np.eye(3)) if end is None else end
    return {"newbook": S(*new, vel=vel), "run0": S(*run[0]), "run1": S(*run[1]), "run2": S(*run[2]), "bookend": S(*end), "tips": np.asarray(tips, float), **({} if body is None else {"body": body})}


FAR = np.array([[0.32, 0.305, TOP + 0.5]] * 15 + [[0.32, -0.305, TOP + 0.5]] * 15)  # forearms away from everything


def hold_state():
    """One step of the hold: the right hand's fingertips on the end book's head, the left hand's at the green book."""
    tips = AWAY.copy()
    tips[12:] = [t.X_S + 0.07, t.upright_faces()[2][0] - 0.01, TOP + t.Z_S + 0.22]
    tips[:12] = [t.X_S + 0.05, t.SLOT_Y, TOP + t.Z_S + 0.19]
    return state(new=green(proud=0.06, tilt=0.0), run=run_upright(lean=0.5), tips=tips)


def play(states, steps: int = 70, hold: bool = True, hold_steps: int | None = None, hold_run=None) -> float:
    """Feed the same state (or a list) for `steps` steps; the bookend's mark is where it stood on the first step of a run that starts at
    the true start."""
    tr = t.Trial()
    tr.reset()
    tr.end0 = END0.copy()
    seq = states if isinstance(states, list) else [states] * steps
    if hold:
        h = hold_state()
        if hold_run is not None:
            h = dict(h)
            for i, (p_, R_) in enumerate(hold_run):
                h[f"run{i}"] = S(p_, R_)
        seq = [h] * (t.HOLD_NEED if hold_steps is None else hold_steps) + seq
    for st in seq:
        t.m.state = lambda env, name, st=st: st[name]
        t.m.fingertips = lambda env, st=st: st["tips"]
        t.m.hand_body = lambda env, st=st: st.get("body", FAR)
        tr.step(None)
    return tr.score


def tips_at(point, hand="left"):
    tips = AWAY.copy()
    sl = slice(0, 12) if hand == "left" else slice(12, 24)
    tips[sl] = np.asarray(point, float)
    return tips


start_run = t.run_start()
spine = np.array([t.X_S, t.SLOT_Y, TOP + t.Z_S + 0.08])


def moved_run(dx=0.0, dy=0.0):
    return [(p + [dx, dy, 0.0], R) for p, R in run_upright()]


def body_at(point):
    b = FAR.copy()
    b[:] = np.asarray(point, float)
    return b


def yawed_end(deg):
    return (END0, rz(deg))


def wedge_play():
    """One hand only: it never comes near the run while the other is at the green book; the end state is a full one."""
    return play(state(), hold=False)


def knocked_play():
    """A run book knocked flat for 30 steps, then everything stood up again and held at 1.0."""
    return play([state(run=run_upright(which=0, tilt_one=40.0))] * 30 + [state()] * 70)


def slow_tip_play():
    """Every book at rest by centre speed, but a run book still turning at 0.3 rad/s."""
    st = state()
    st["run2"].ang_vel = np.array([0.3, 0.0, 0.0])
    return play(st)
CASES = [
    ("nothing done: the start state", state(new=(np.array([t.NEW_AT[0], t.NEW_AT[1], TOP + t.NEW_H / 2]), np.eye(3)), run=start_run), 0.0),
    ("1.0: in, flush, upright, at rest, hands clear, held 2 s", state(), 1.0),
    ("held only 39 steps of the 40", [state()] * 20 + [state()] * 39, None),
    ("still moving: the green book at 2 cm/s", state(vel=0.02), 0.0),
    ("left fingertip on the spine (2 cm)", state(tips=tips_at(spine - [0.02, 0, 0], "left")), 0.0),
    ("right fingertip on the end book's head (1 cm)", state(tips=tips_at([t.X_S + 0.07, t.upright_faces()[2][0] - 0.01, TOP + t.Z_S + 0.22], "right")), 0.0),
    ("a fingertip 2.5 cm off the bookend's plate", state(tips=tips_at([t.X_S + 0.07, t.Y_PLATE - 0.025, TOP + t.Z_B + 0.10], "right")), 0.0),
    ("hands 3.5 cm clear of everything", state(tips=tips_at(spine - [0.035, 0, 0], "left")), 1.0),
    ("the bookend shoved 1.2 cm along the row", state(end=(END0 + [0, -0.012, 0], np.eye(3))), 0.0),
    ("the bookend moved 0.4 cm (on its mark within 6 mm)", state(end=(END0 + [0, -0.004, 0], np.eye(3))), 1.0),
    ("the bookend moved 0.8 cm", state(end=(END0 + [0, -0.008, 0], np.eye(3))), 0.0),
    ("the bookend lifted 5 cm", state(end=(END0 + [0, 0, 0.05], np.eye(3))), 0.0),
    ("the bookend turned 90 degrees about the vertical", state(end=yawed_end(90.0)), 0.0),
    ("the bookend turned 180 degrees", state(end=yawed_end(180.0)), 0.0),
    ("the bookend shifted 4 mm and tilted 9.9 degrees", state(end=(END0 + [0, -0.004, 0], t.rot_x(9.9))), 0.0),
    ("the run 8 cm deeper, the green book flush to it", state(run=moved_run(dx=0.08), new=green(proud=-0.08)), 0.0),
    ("the run 30 cm along the row, upright", state(run=moved_run(dy=0.30)), 0.0),
    ("the run books at x +5, 0, -5 cm (mean zero)", state(run=[(p + [d, 0, 0], R) for (p, R), d in zip(run_upright(), (0.05, 0.0, -0.05))]), 0.0),
    ("the run 3 cm into the green book's slot", state(run=moved_run(dy=-0.03)), 0.0),
    ("the run at its start lean (17), green book flush at 19 degrees", state(run=start_run, new=green(tilt=19.0)), 0.5),
    ("the run at 11 degrees, green book flush", state(run=run_upright(lean=11.0)), 0.5),
    ("the green book yawed 10 degrees, otherwise flush", state(new=green(R=rz(10.0) @ t.rot_x(3.0))), 0.5),
    ("a palm 1 cm off the run's head, fingertips 6 cm away", state(body=body_at([t.X_S + 0.07, t.upright_faces()[2][0], TOP + t.Z_S + 0.25])), 0.0),
    ("a run book knocked flat for 30 steps, then stood up again (latched)", "knocked", 0.0),
    ("the hold lasts 19 steps of the 20 needed: capped at 0.5", "hold19", 0.5),
    ("the hand at the run for 20 steps but the run still leans 17 degrees: capped at 0.5", "holdlean", 0.5),
    ("a hand passes the run for one step: capped at 0.5", "hold1", 0.5),
    ("the green book yawed 5 degrees, otherwise flush", state(new=green(R=rz(5.0) @ t.rot_x(3.0))), 1.0),
    ("the green book yawed 7 degrees, otherwise flush", state(new=green(R=rz(7.0) @ t.rot_x(3.0))), 0.5),
    ("one hand only, never at the run: capped at 0.5", "wedge", 0.5),
    ("a run book turning at 0.3 rad/s with the centres at rest", "slow", 0.0),
    ("the bookend tipped 12 degrees", state(end=(END0, t.rot_x(12.0))), 0.0),
    ("a run book fallen over (40 degrees)", state(run=run_upright(which=0, tilt_one=40.0)), 0.0),
    ("a run book off the shelf (2 cm below the board)", state(run=[(p - [0, 0, 0.03] if i == 0 else p, R) for i, (p, R) in enumerate(run_upright())]), 0.0),
    ("0.75: seated, the green book tilted 12 degrees", state(new=green(tilt=12.0)), 0.75),
    ("0.75: seated, the run at 9.5 degrees", state(run=run_upright(lean=9.5)), 0.75),
    ("1.0 edge: the run at 7.9 degrees", state(run=run_upright(lean=7.9)), 1.0),
    ("0.75: the run at 8.1 degrees", state(run=run_upright(lean=8.1)), 0.75),
    ("1.0 edge: spine 9 mm proud", state(new=green(proud=0.009)), 1.0),
    ("0.5: spine 12 mm proud", state(new=green(proud=0.012)), 0.5),
    ("0.5: spine sunk 15 mm", state(new=green(proud=-0.015)), 0.5),
    ("0.5: put in spine first (turned round)", state(new=green(proud=-0.13, R=rz(180.0) @ t.rot_x(3.0))), 0.5),
    ("0.5: flush but the green book tilted 22 degrees", state(new=green(tilt=22.0)), 0.5),
    ("0.5: half in, spine 6 cm proud", state(new=green(proud=0.06)), 0.5),
    ("0.25: entered, spine 9 cm proud (4 cm in)", state(new=green(proud=0.09, tilt=15.0), run=start_run), 0.25),
    ("0: only 2 cm into the gap", state(new=green(proud=0.11, tilt=15.0), run=start_run), 0.0),
    ("0: standing in the gap but tilted 40 degrees", state(new=green(proud=0.0, tilt=40.0)), 0.0),
    ("0: lying across the run's heads", state(new=(np.array([t.X_S + 0.07, 0.05, TOP + t.Z_S + 0.25]), t.rot_x(90.0)), run=start_run), 0.0),
    ("0: on the shelf flush, but in front of the packed books, not in the gap", state(new=green(y=0.15)), 0.0),
    ("0: on the table in front of the gap", state(new=(np.array([0.27, t.SLOT_Y, TOP + t.NEW_H / 2]), np.eye(3)), run=start_run), 0.0),
]


def main() -> None:
    bad = 0
    for name, st, want in CASES:
        if isinstance(st, str):
            got = {"knocked": knocked_play, "wedge": wedge_play, "slow": slow_tip_play,
                   "hold19": lambda: play(state(), hold_steps=19), "hold1": lambda: play(state(), hold_steps=1),
                   "holdlean": lambda: play(state(), hold_run=start_run)}[st]()
        elif want is None:  # the hold: 39 steps at rest after a spell of motion
            got = play([dict(st[0], newbook=S(*green(), vel=0.05))] * 20 + st[20:])
            want = 0.0
        else:
            got = play(st)
        ok = abs(got - want) < 1e-9
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {got:4.2f} (want {want:4.2f})  {name}")
    print(f"{len(CASES) - bad} of {len(CASES)} cases as the scoring text says")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
