"""IK dry run, no simulator: the right hand carries the wand's coil along the arm, top pass then underside pass at the maker's stand-off (kit README 19).

    python gate_ik.py          (needs numpy and yam_arena on the path)
Env: AX (arm x), YS (shoulder y), ARM_DIR (+1 fingertips toward +y, -1 toward -y), ZA (arm height above the table).
"""

from __future__ import annotations

import io
import math
import os
import pathlib
import sys
import contextlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

TOP = station.TABLE_TOP
LO = np.radians([-150.0, 0.0, 0.0, -97.0, -90.0, -120.0])  # the YAM's joint limits (yam_arm.urdf)
HI = np.radians([180.0, 210.0, 180.0, 90.0, 90.0, 120.0])
AX = float(os.environ.get("AX", "0.32"))
YS = float(os.environ.get("YS", "-0.45"))
DIR = float(os.environ.get("ARM_DIR", "1"))
ZA = TOP + float(os.environ.get("ZA", "0.32"))
STAND = 0.05
ARM_L, WRIST_S = 0.70, 0.52
R_SH, R_WR, R_HAND = 0.05, 0.0275, 0.02
U_GRIP, U_C0, U_C1, U_END = 0.10, 0.263, 0.483, 0.483
L_COIL = 0.5 * (U_C0 + U_C1) - U_GRIP  # grip to coil centre along the wand
HALF = (0.016, 0.0415)  # coil half thickness (along the jaw direction, hand col 0) and half width (along the fingers, col 2)
WATCH_S = WRIST_S - 0.06
S0, S1, S2 = 0.06, 0.62, 0.86  # coil centre: first and last of a pass, and clear of the fingertips for the drop under


def arm_r(s):
    s = np.clip(s, 0, ARM_L)
    return np.where(s < WRIST_S, R_SH + (R_WR - R_SH) * s / WRIST_S, R_WR + (R_HAND - R_WR) * (s - WRIST_S) / (ARM_L - WRIST_S))


def arm_pt(s):
    return np.array([AX, YS + DIR * s, ZA])


def clearance(tcp, R, w):
    """Smallest gap between the wand body (sampled points of its handle and body) and the arm's surface."""
    us = np.linspace(0.0, U_END, 40)
    best = 9.9
    for u in us:
        half = (0.016, 0.016) if u < 0.12 else (0.016, 0.0415)
        for a in (-1, 1):
            for b in (-1, 1):
                p = tcp + w * (u - U_GRIP) + R[:, 0] * a * half[0] + R[:, 2] * b * half[1]
                s = (p[1] - YS) * DIR
                if -0.02 <= s <= ARM_L + 0.02:
                    best = min(best, float(np.hypot(p[0] - AX, p[2] - ZA) - arm_r(s)))
    return best


def roll(g, deg):
    return d._as_grip(g.rot @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(deg)))


class Dry:
    def __init__(self):
        self.base = np.asarray(station.BASES["right"], float)
        self.q = np.array(READY, float)
        self.p, self.R = ik.tcp_pose(self.q, self.base)
        self.rows = []
        self.margin = 9.9
        self.tight = None

    def move(self, label, p, R, n=24, w=None):
        worst_mm, worst_deg, j5, step, clr = 0.0, 0.0, [], 0.0, 9.9
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            pp = self.p + (np.asarray(p, float) - self.p) * s
            RR = ik.slerp_matrix(self.R, R, s)
            q, pe, re_ = ik.solve(pp, ik.matrix_to_quat_xyzw(RR), self.q, base_pos=self.base, iters=60)
            step = max(step, float(np.degrees(np.abs(q - self.q)).max()))
            self.q = q
            mg = np.minimum(q - LO, HI - q)
            if float(mg.min()) < self.margin:
                self.margin, self.tight = float(mg.min()), (label, int(np.argmin(mg)) + 1, round(math.degrees(float(q[int(np.argmin(mg))])), 1))
            worst_mm, worst_deg = max(worst_mm, pe * 1000), max(worst_deg, math.degrees(re_))
            j5.append(math.degrees(q[4]))
            if w is not None:
                clr = min(clr, clearance(pp, RR, w))
        self.p, self.R = np.asarray(p, float), R
        self.rows.append((label, worst_mm, worst_deg, min(j5), max(j5), step, clr))

    def summary(self):
        mm = max(r[1] for r in self.rows)
        deg = max(r[2] for r in self.rows)
        return f"worst miss {mm:.1f} mm {deg:.1f} deg, j5 {min(r[3] for r in self.rows):.0f}..{max(r[4] for r in self.rows):.0f}, joint margin {math.degrees(self.margin):.1f} deg at {self.tight}"

    def ok(self, strict=True):
        return all(r[1] <= 3.0 and r[2] <= 2.0 and r[5] <= 10.0 and r[6] >= 0.02 for r in self.rows)

    def report(self, tag=""):
        print(tag)
        for label, mm, deg, lo, hi, step, clr in self.rows:
            bad = mm > 3.0 or deg > 2.0 or step > 10.0 or clr < 0.02
            print(f"  {label:<24} miss {mm:5.1f} mm {deg:4.1f} deg  j5 {lo:6.1f}..{hi:6.1f}  step {step:4.1f}  wand-arm gap {clr * 1000:6.1f} mm{'  <-- MISS' if bad else ''}")


def hz(R, w):
    return abs(R[2, 0]) * HALF[0] + abs(R[2, 2]) * HALF[1] + abs(w[2]) * 0.0


HOOK = (float(os.environ.get("HX", "0.32")), float(os.environ.get("HY", "-0.62")))  # the grip point's x, y at the holder; the wand lies along +y
HOOK_Z = float(os.environ.get("HOOK_Z", "0.12"))  # the grip point's height above the table at the holder
RHO = float(os.environ.get('RHO', '0.14'))
SIDE = float(os.environ.get('SIDE', '-1'))
MIDZ = float(os.environ.get('MIDZ', '0.05'))  # the roll under the arm: the wand, kept parallel to it, circles it at RHO
MID = np.array([HOOK[0], HOOK[1], ZA + MIDZ])
  # the roll under the arm: the wand, kept parallel to it, circles it at this radius


def hook_grip(sgn):
    """A down grip whose wand axis (sgn * col 1) points along +y, the wand lying flat in the holder."""
    for yaw in range(0, 360, 15):
        g = d.down(float(yaw))
        if (sgn * g.rot[:, 1])[1] > 0.99:
            return g
    return None


BACK = 0.12


def run(g, sgn=1, hook=None, n=4, lift=0.04, hk=None):
    """One grip throughout: the hand takes the wand from its -x side (fingers along +x, jaws closing vertically on the handle), so there is no turn of the hand."""
    R = g.rot
    w = sgn * R[:, 1]
    D = Dry()
    h = hz(R, w)
    coil = lambda s, side, extra=0.0: arm_pt(s) + np.array([0, 0, side * (float(arm_r(s)) + STAND + extra + h)])
    tcp = lambda c: c - w * L_COIL
    hk = np.array([HOOK[0], HOOK[1], TOP + HOOK_Z]) if hk is None else np.asarray(hk, float)
    a = R[:, 2]
    D.move("to the handle", hk - 0.10 * a, R, 30, w)
    D.move("on the handle", hk, R, 8, w)
    D.move("lift", hk + [0, 0, lift], R, 8, w)
    D.move("back", hk + [-BACK, 0, lift], R, 12, w)
    top = tcp(coil(S0, +1, 0.10)) + [-BACK, 0, 0]
    D.move("rise", np.array([hk[0] - BACK, hk[1], top[2]]), R, 30, w)
    D.move("up", top, R, 20, w)
    D.move("approach shoulder", tcp(coil(S0, +1, 0.03)), R, 30, w)
    for s in np.linspace(S0, S1, 10):
        D.move("top pass", tcp(coil(float(s), +1)), R, n, w)
    cen = lambda s_, phi: arm_pt(s_) + np.array([SIDE * RHO * math.sin(phi), 0.0, RHO * math.cos(phi)])
    SR = float(os.environ.get("SR", "0.5"))
    D.move("along to the roll", tcp(coil(SR, +1, 0.0)), R, 12, w)
    D.move("out to the roll", tcp(cen(SR, 0.0)), R, 12, w)
    RH = float(os.environ.get("ROLLHAND", "0"))  # +-1: the hand also rolls about the wand's axis as it circles the arm
    Ry = lambda a: axis_angle_matrix(np.array([0.0, 1.0, 0.0]), a)
    for k in range(1, 13):
        phi = math.pi * k / 12
        Rk = Ry(RH * phi) @ R
        wk = sgn * Rk[:, 1]
        D.move("roll under", cen(SR, phi) - wk * L_COIL, Rk, 6, wk)
    Rend = Ry(RH * math.pi) @ R
    D.move("in to the stand-off", coil(SR, -1) - w * L_COIL, Rend, 12, w)
    D.move("under to the far end", coil(S1, -1) - w * L_COIL, Rend, 12, w)
    for s in np.linspace(S1, S0, 10):
        D.move("underside pass", coil(float(s), -1) - w * L_COIL, Rend, n, w)
    D.move("clear down", coil(S0, -1, 0.12) - w * L_COIL, Rend, 12, w)
    D.move("hand back", coil(S0, -1, 0.12) - w * L_COIL, R, 20, w)
    D.move("back to the holder", hk + [-BACK, 0, lift], R, 40, w)
    D.move("over the holder", hk + [0, 0, lift], R, 12, w)
    D.move("lay", hk, R, 8, w)
    D.move("draw back", hk - 0.10 * a, R, 8, w)
    return D


def main():
    Rs = roll(d.tilted(90, 0, False), 90)
    if os.environ.get("GRID"):  # one line per layout: GRID=1 ZA=.. HOOK_Z=.. python gate_ik.py
        D = run(Rs, 1, n=8)
        print(f"ZA {ZA - TOP:.2f} hook {HOOK[0]:.2f},{HOOK[1]:.2f},{HOOK_Z:.3f}: {D.summary()}")
        return
    D = run(Rs, 1, n=8)
    D.report("level grip, holder to holder")
    print(D.summary())


if __name__ == "__main__":
    main()
