"""Checks of the scorer's logic (`judge` in task.py) on hand-made state sequences, no simulator: run with the harness's Python from this folder.

    python check_score.py
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as t  # noqa: E402

GOAT, CAT = t.GOAT_JOINT, 0.0
OK = dict(face_up=True, on_table=True, off_mm=2.0, yaw_deg=0.0, gap_m=0.2)


def run(steps) -> float:
    """Each step: (arrow, lever, overrides). Returns the score after the last step."""
    mem, s = {}, 0.0
    for arrow, lever, kw in steps:
        s = t.judge(mem, **{**OK, **kw}, arrow_deg=arrow, lever_deg=lever)
    return s


def hold(arrow, lever, n, **kw):
    return [(arrow, lever, kw)] * n


CASES = {
    "nothing done": (hold(CAT, 0.0, 60), 0.0),
    "arrow on the goat only": (hold(GOAT, 0.0, 60), 0.33),
    "arrow 14 deg off the goat's centre": (hold(GOAT + 14.0, 0.0, 10), 0.33),
    "arrow 16 deg off: the next picture's edge": (hold(GOAT + 16.0, 0.0, 10), 0.0),
    "full act": (hold(GOAT, 0.0, 5, gap_m=0.0) + hold(GOAT, 70.0, 3, gap_m=0.0) + hold(GOAT, 0.0, 41), 1.0),
    "full act, held only 1.5 s": (hold(GOAT, 70.0, 3, gap_m=0.0) + hold(GOAT, 0.0, 30), 0.67),
    "pull short of the trigger (60 of 67.5)": (hold(GOAT, 60.0, 5, gap_m=0.0) + hold(GOAT, 0.0, 60), 0.33),
    "pull with the arrow on the cat, then turn to the goat": (hold(CAT, 70.0, 5, gap_m=0.0) + hold(GOAT, 0.0, 60), 0.33),
    "pull on the goat, then arrow knocked off": (hold(GOAT, 70.0, 5, gap_m=0.0) + hold(GOAT + 40.0, 0.0, 60), 0.0),
    "pull while the toy is dragged 40 mm off its marks": (hold(GOAT, 70.0, 5, gap_m=0.0, off_mm=40.0) + hold(GOAT, 0.0, 60, off_mm=40.0), 0.33),
    "pull while the toy is turned 15 deg": (hold(GOAT, 70.0, 5, gap_m=0.0, yaw_deg=15.0) + hold(GOAT, 0.0, 60), 0.33),
    "played, then dragged off its marks": (hold(GOAT, 70.0, 5, gap_m=0.0) + hold(GOAT, 0.0, 60, off_mm=40.0), 0.67),
    "played, lever held down at the end": (hold(GOAT, 70.0, 60, gap_m=0.0), 0.67),
    "played, hand left 30 mm from the toy": (hold(GOAT, 70.0, 5, gap_m=0.0) + hold(GOAT, 0.0, 60, gap_m=0.03), 0.67),
    "played, then the toy tipped (void)": (hold(GOAT, 70.0, 5, gap_m=0.0) + hold(GOAT, 0.0, 2, face_up=False) + hold(GOAT, 0.0, 60), 0.0),
    "toy pushed off the table (void)": (hold(GOAT, 0.0, 5) + hold(GOAT, 0.0, 2, on_table=False), 0.0),
}


def main() -> int:
    bad = 0
    for name, (steps, want) in CASES.items():
        got = run(steps)
        ok = abs(got - want) < 1e-9
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {name}: {got} (want {want})")
    print(f"{len(CASES) - bad} of {len(CASES)} pass")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
