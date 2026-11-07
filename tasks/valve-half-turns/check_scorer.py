"""The scorer's lines on hand-built states, with no simulator. Run it with the harness's Python, from this folder: python check_scorer.py"""

from __future__ import annotations

import numpy as np
import task as t

KNOB = np.array([t.VX, t.VY - t.WHEEL_R, t.TOP + 0.112])


def gap(x, y, z):
    """The gap from a point placed relative to the wheel's centre."""
    return t.gap_to_wheel(np.array([[t.VX + x, t.VY + y, t.TOP + t.WHEEL_Z + z]]), KNOB)

CASES = [
    ("3 turns at the start, closed, then open 1.5", lambda: t.task_score(True, 1.5, 3, True), 1.0),
    ("2 turns at the start, closed, then open 1.0", lambda: t.task_score(True, 1.0, 2, True), 1.0),
    ("4 turns at the start, closed, then open 2.0", lambda: t.task_score(True, 2.0, 4, True), 1.0),
    ("0.2 turn past half", lambda: t.task_score(True, 1.7, 3, True), 1.0),
    ("0.21 turn past half", lambda: t.task_score(True, 1.71, 3, True), 0.25),
    ("0.21 turn short of half", lambda: t.task_score(True, 1.29, 3, True), 0.25),
    ("never closed, left at half", lambda: t.task_score(False, 1.5, 3, True), 0.0),
    ("never closed, not touched", lambda: t.task_score(False, 3.0, 3, True), 0.0),
    ("closed and left closed", lambda: t.task_score(True, 0.0, 3, True), 0.0),
    ("closed, reopened fully", lambda: t.task_score(True, 3.0, 3, True), 0.25),
    ("half, still holding the knob", lambda: t.task_score(True, 1.5, 3, False), 0.25),
    ("a finger on the rim's far side, 5 cm from the knob, is not clear", lambda: t.end_ok_now(gap(0.0, t.WHEEL_R, 0.0), 0.0), False),
    ("a finger on a spoke is not clear", lambda: t.end_ok_now(gap(0.015, 0.0, 0.003), 0.0), False),
    ("a finger on the hub is not clear", lambda: t.end_ok_now(gap(0.0, 0.0, 0.006), 0.0), False),
    ("a finger 10 cm over the wheel is clear", lambda: t.end_ok_now(gap(0.0, 0.03, 0.10), 0.0), True),
    ("end state: fingers 3 cm clear, wheel still", lambda: t.end_ok_now(0.03, 0.0), True),
    ("end state: fingers 2.9 cm from the knob", lambda: t.end_ok_now(0.029, 0.0), False),
    ("end state: the wheel still turning", lambda: t.end_ok_now(0.09, 0.2), False),
    ("seed 0 starts 2 turns open", lambda: t.start_turns(0), 2),
    ("seed 1 starts 3 turns open", lambda: t.start_turns(1), 3),
    ("seed 2 starts 4 turns open", lambda: t.start_turns(2), 4),
]


def main() -> None:
    bad = 0
    for label, fn, want in CASES:
        got = fn()
        ok = got == want
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {got!s:6} want {want!s:6} {label}")
    print("a fixed opening of 1.5 turns after closing, at each start:", {n: t.task_score(True, 1.5, n, True) for n in (2, 3, 4)})
    print("the windows round 1, 1.5 and 2 turns do not touch:", 1.0 + t.WINDOW < 1.5 - t.WINDOW and 1.5 + t.WINDOW < 2.0 - t.WINDOW)
    print("FAILED" if bad else "all lines agree")
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
