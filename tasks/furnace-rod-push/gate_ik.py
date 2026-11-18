"""IK dry run, no simulator: both hands along the whole push, hand-over, draw-back and stowing, seeded step by step (kit README 19).

    python gate_ik.py        (needs numpy and yam_arena on the path)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

TOP = station.TABLE_TOP
XR = float(os.environ.get('XR', '0.40'))
ZF = TOP + 0.075
ZR = ZF + 0.0075
MARK_U = 0.63
U_R, U_L = 0.83, 0.72  # grips behind the furnace end, right and left
YOFF = 0.30
S0, S_HAND, S_END, S_OUT = 0.22 + YOFF, 0.62 + YOFF, 0.83 + YOFF, 0.17 + YOFF
LIFT = float(os.environ.get('LIFT', '0.03'))


def roll(g, deg):
    return d._as_grip(g.rot @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(deg)))


class Dry:
    def __init__(self, side):
        self.side, self.base = side, np.asarray(station.BASES[side], float)
        self.q = np.array(READY, float)
        self.p, self.R = ik.tcp_pose(self.q, self.base)
        self.rows = []

    def move(self, label, p, R, n=24):
        worst_mm, worst_deg, j5, step = 0.0, 0.0, [], 0.0
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            pp = self.p + (np.asarray(p, float) - self.p) * s
            RR = ik.slerp_matrix(self.R, R, s)
            q, pe, re_ = ik.solve(pp, ik.matrix_to_quat_xyzw(RR), self.q, base_pos=self.base, iters=60)
            step = max(step, float(np.degrees(np.abs(q - self.q)).max()))
            self.q = q
            worst_mm, worst_deg = max(worst_mm, pe * 1000), max(worst_deg, math.degrees(re_))
            j5.append(math.degrees(q[4]))
        self.p, self.R = np.asarray(p, float), R
        self.rows.append((label, worst_mm, worst_deg, min(j5), max(j5), step))

    def report(self, name):
        ok = True
        print(f"{self.side} {name}:")
        for label, mm, deg, lo, hi, step in self.rows:
            bad = mm > 3.0 or deg > 2.0 or step > 10.0
            ok = ok and not bad
            print(f"  {label:<22} miss {mm:5.1f} mm {deg:4.1f} deg  j5 {lo:6.1f}..{hi:6.1f}  step {step:4.1f}{'  <-- MISS' if bad else ''}")
        return ok


def grips():
    out = {}
    tilts = tuple(float(t) for t in os.environ.get("TILTS", "90,85,80,75").split(","))
    for th in ((0.0, 15.0, 30.0, 45.0, 60.0) if os.environ.get('GRIPMODE') == 'h' else tilts):
        for phi in (0.0, 30.0, -30.0):
            for flip in (False, True):
                for rl in (90.0, -90.0):
                    g = roll(d.tilted(th, phi, flip), rl)
                    if os.environ.get("GRIPMODE") == "h":  # jaws close sideways, across the rod: the pad length resists yaw
                        if abs(g.rot[0, 0]) > 0.95:
                            out[(th, phi, flip, rl)] = g
                    elif abs(g.rot[2, 0]) > 0.9:  # jaws close vertically
                        out[(th, phi, flip, rl)] = g
    return out


def run_right(gr):
    """The demo's right-hand path: take the rod from behind along the fingers, push to the hand-over, clear; take it again, lift, draw it out, lay it on the rests, clear."""
    R = Dry("right")
    a = gr.rot[:, 2]
    y = lambda s, u: s - u  # noqa: E731
    p1 = np.array((XR, y(S0, U_R), ZR))
    R.move("approach", p1 - 0.10 * a + (0, 0, 0.05), gr.rot, 30)
    R.move("behind", p1 - 0.10 * a, gr.rot)
    R.move("on rod", p1, gr.rot)
    R.move("push to hand-over", (XR, y(S_HAND, U_R), ZR), gr.rot, 60)
    R.move("clear up", (XR, y(S_HAND, U_R) - 0.05, ZR + 0.12), gr.rot)
    p2 = np.array((XR, y(S_HAND, U_R), ZR))
    R.move("approach 2", p2 - 0.10 * a + (0, 0, 0.05), gr.rot, 30)
    R.move("behind 2", p2 - 0.10 * a, gr.rot)
    R.move("on rod 2", p2, gr.rot)
    R.move("lift", p2 + (0, 0, LIFT), gr.rot)
    R.move("pull out", (XR, y(S_OUT, U_R), ZR + LIFT), gr.rot, 60)
    R.move("lay", (XR, y(S_OUT, U_R), ZR), gr.rot)
    R.move("clear", (XR - 0.03, y(S_OUT, U_R), ZR + 0.10), gr.rot)
    return R


def run_left(gl):
    L = Dry("left")
    a = gl.rot[:, 2]
    y = lambda s, u: s - u  # noqa: E731
    p1 = np.array((XR, y(S_HAND, U_L), ZR))
    L.move("approach", p1 - 0.10 * a + (0, 0, 0.05), gl.rot, 30)
    L.move("behind", p1 - 0.10 * a, gl.rot)
    L.move("on rod", p1, gl.rot)
    L.move("push to end", (XR, y(S_END, U_L), ZR), gl.rot, 40)
    L.move("pull to hand-over", (XR, y(S_HAND, U_L), ZR), gl.rot, 40)
    L.move("left clear up", (XR, y(S_HAND, U_L), ZR + 0.12), gl.rot)
    return L


def main():
    import io, contextlib
    G = grips()
    print(len(G), "candidate grips", flush=True)
    for name, fn in (("right", run_right), ("left", run_left)):
        good = []
        for k, g in G.items():
            with contextlib.redirect_stdout(io.StringIO()):
                D = fn(g)
                ok = D.report(name)
            worst = max(r[1] for r in D.rows)
            print(name, k, "OK" if ok else f"no (worst miss {worst:.1f} mm)", flush=True)
            if ok:
                good.append(k)
        print(name, "passing:", good, flush=True)
        for k in good[:3]:
            fn(G[k]).report(name + " " + str(k))


if __name__ == '__main__':
    main()
