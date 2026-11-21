"""The scoring table, the carton test and the hands-clear distance, on hand-built values, with no simulator and no GPU.
Run with the harness's Python, from this folder: python check_score.py"""

from __future__ import annotations

import sys

import numpy as np

import task as t


def main() -> int:
    bad = 0

    def check(name, ok, got=""):
        nonlocal bad
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {name}{': ' + str(got) if got != '' else ''}")

    for name, args, want in (  # judge(temp, final, boxed, released, dead[, dropped])
        ("final lock, in the carton, let go", (True, True, True, True, False), 1.0),
        ("final lock, in the carton, still held", (True, True, True, False, False), 0.67),
        ("final lock, carried but not in the carton", (True, True, False, True, False), 0.67),
        ("temporary click only, in the carton, let go", (True, False, True, True, False), 0.33),
        ("cap only swung shut, no click, in the carton", (False, False, True, True, False), 0.0),
        ("final lock, in the carton, let go, but dropped", (True, True, True, True, False, True), 0.67),
        ("final lock, in the carton, but tipped past 30 degrees earlier", (True, True, True, True, True), 0.0),
    ):
        got = t.judge(*args)
        check(name, got == want, got)
    c = np.array([t.BX, t.BY, t.TOP + t.BT])
    check("upright on the carton floor", t.in_carton(c, 2.0))
    check("leaning 12 degrees in the carton", not t.in_carton(c, 12.0))
    check("on the counter beside the carton", not t.in_carton(np.array([t.CX, t.CY, t.TOP]), 0.0))
    check("held 3 cm above the carton floor", not t.in_carton(c + [0, 0, 0.03], 0.0))
    check("resting on the carton's wall edge", not t.in_carton(c + [0, -t.BW / 2 + t.HY - 0.01, 0], 0.0))
    check("a point on the handle bar is 0 from the outline", t.clearance(np.array([[0.0, (t.H_Y0 + t.H_Y1) / 2, 0.145]])) == 0.0)
    check("a point 3 cm above the shut cap is 2 cm clear", t.clearance(np.array([[0.0, 0.0, t.CAP_Z1 + 0.031]])) >= 0.02)
    check("a point 1 cm off the side wall is not 2 cm clear", t.clearance(np.array([[0.0, -t.HY - 0.012, 0.1]])) < 0.02)
    # drop test: a container that falls while no gripper goes down with it is dropped; one that falls with a gripper holding it is not
    class FakeEnv:  # feeds Trial.step prepared heights through patched helpers
        pass

    def run_drop(zs, hand_dz, tip_clr):
        """zs: container heights per step; hand_dz: heights of the left gripper link; tip_clr: left fingertips' distance to the outline (right arm far away)."""
        tr = t.Trial()
        tr.reset()
        saved = (t.m.state, t.m.fingertips, t.grip_z, t.clearance, t.in_carton)

        class S:
            pos = np.zeros(3)

            def deg(self, n): return t.OPEN
            def joint(self, n): return 0.0
            def tilt_deg(self): return 0.0
            def to_local(self, p): return p

        for z, hz, cl in zip(zs, hand_dz, tip_clr):
            st = S()
            st.pos = np.array([t.BX, t.BY, z])
            t.m.state = lambda env, name, st=st: st
            t.m.fingertips = lambda env, cl=cl: np.vstack([np.full((12, 3), cl), np.full((12, 3), 9.0)])
            t.clearance = lambda p: float(p[0, 0]) if p[0, 0] < 5 else 9.0
            t.in_carton = lambda *a: False
            t.grip_z = lambda env, hz=hz: [hz, 9.0]
            tr.step(None)
        t.m.state, t.m.fingertips, t.grip_z, t.clearance, t.in_carton = saved
        return tr.dropped

    n = 8
    base = [t.TOP + 0.1 - 0.012 * i for i in range(n)]
    check("container falls 12 mm a step, hand stays up: dropped", run_drop(base, [1.0] * n, [0.0] * n))
    check("container and hand fall together at 12 mm a step: not dropped", not run_drop(base, [1.0 - 0.012 * i for i in range(n)], [0.0] * n))
    check("container falls, hand falls but fingertips 2 cm away: dropped", run_drop(base, [1.0 - 0.012 * i for i in range(n)], [0.02] * n))
    slow = [t.TOP + 0.1 - 0.003 * i for i in range(n)]
    check("container lowered 3 mm a step by the hand: not dropped", not run_drop(slow, [1.0 - 0.003 * i for i in range(n)], [0.0] * n))
    creep = [t.TOP + 0.1 - 0.0015 * i for i in range(n)]
    check("container creeps 1.5 mm a step with the hand still: not flagged (under 6 mm in 3 steps)", not run_drop(creep, [1.0] * n, [0.0] * n))
    check("container falls 3 mm a step, hand still: dropped (over 6 mm in 3 steps)", run_drop(slow, [1.0] * n, [0.0] * n))
    print("all ok" if not bad else f"{bad} bad")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
