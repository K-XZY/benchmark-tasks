"""The scorer and the removal rule on hand-built states, with no simulator.

    python -B check_score.py

1. The rule alone: a mount held at a fixed tilt and push, swept 3.5 to 8 cm out on the turning paper, ground step by step with task.py's
   own contact_pressure and removal_rate. Prints the time to 0.3 mm and the removal round the ring and at the centre.
2. The ladder: each rung, each zero, and the lazy routes, from hand-built depth maps fed to task.py's own rungs().
Exits non-zero if any case scores other than the scoring text says.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as t  # noqa: E402

DT = 0.05


def grind(force, tilt_deg=0.0, seconds=150.0, stop_mm=None):
    """Hold the mount at `tilt_deg` (toward +x) under `force` N, sweeping; returns (depth mm, seconds run)."""
    depth = np.zeros(len(t.POINTS))
    ang = math.radians(tilt_deg)
    R = np.array([[math.cos(ang), 0, math.sin(ang)], [0, 1, 0], [-math.sin(ang), 0, math.cos(ang)]])
    n = int(seconds / DT)
    for k in range(n):
        tt = k * DT
        s = 0.5 - 0.5 * math.cos(2 * math.pi * tt / t.SWEEP_PERIOD)
        r = t.SWEEP_IN + (t.SWEEP_OUT - t.SWEEP_IN) * s
        world = np.c_[t.POINTS, depth] @ R.T
        h = world[:, 2]
        x = r + world[:, 0]
        y = world[:, 1]
        p = t.contact_pressure(h, t.AREA, force)
        v = t.OMEGA * np.hypot(x, y)
        depth += t.removal_rate(p, v) * DT
        if stop_mm is not None and depth[: t.N_GRID].mean() * 1000 >= stop_mm:
            return depth * 1000, tt
    return depth * 1000, seconds


def main() -> int:
    bad = []
    print("1. the rule (flat hold unless a tilt is given):")
    rows = [(15.0, 0.0), (13.0, 0.0), (11.0, 0.0), (8.0, 0.0), (5.0, 0.0), (13.0, 0.3), (13.0, 1.0), (13.0, 3.0)]
    for force, tilt in rows:
        mm, sec = grind(force, tilt, 300.0, stop_mm=0.3)
        q = t.scored_points(mm)
        sc, reached, failed = t.rungs(mm)
        print(f"   {force:4.1f} N, tilt {tilt:3.1f} deg: {sec:6.1f} s to mean 0.3 mm (or the 300 s limit); ring min/max {q[:-1].min():.3f}/{q[:-1].max():.3f} (at {np.argmax(q[:-1]) * 360 // t.RING_N} deg), centre {q[-1]:.3f}, spread {q.max() - q.min():.3f} -> score {sc}")
        if tilt == 0.0 and force >= 11.0 and sc != 0.75:
            bad.append(f"flat {force} N should reach the line (0.75 before the set-down), got {sc}")
        if tilt >= 1.0 and sc >= 0.75:
            bad.append(f"tilt {tilt} deg should not reach the line rung, got {sc}")
    mm5, _ = grind(5.0, 0.0, 300.0)
    print(f"   5 N for the whole 300 s: mean {mm5[:t.N_GRID].mean():.2f} mm, ring max {t.scored_points(mm5)[:-1].max():.3f} -> score {t.rungs(mm5)[0]}")
    if mm5[:t.N_GRID].mean() >= t.LOW_MM:
        bad.append("a 5 N touch reaches the line within the run")
    mm_light, _ = grind(0.5, 0.0, 300.0)
    print(f"   0.5 N (resting, a light touch): mean {mm_light[:t.N_GRID].mean():.3f} mm")
    if mm_light.max() > 0.0:
        bad.append("a light touch removes material")

    print("2. the ladder:")
    n = len(t.POINTS)
    flat = lambda v: np.full(n, v)  # noqa: E731
    facet = np.clip(t.POINTS[:, 0] * 1000 * 0.01 + 0.08, 0.0, None)  # ground on one side only (a facet)
    wedge = 0.3 + t.POINTS[:, 0] * 1000 * 0.004  # 0.3 mm mean, 0.112 mm across the two edge points on x (0.23 degrees)
    def centre(v_edge, v_mid):  # the ring at v_edge, the centre at v_mid
        d = flat(v_edge)
        d[t.N_GRID + t.RING_N] = v_mid
        return d
    def spot(v, deg, width=0.0):  # the whole face at 0.30 mm except a dip of v mm at `deg` round the ring (width 0: one stop; else a bump of that half-width, degrees)
        d = flat(0.3)
        ang = (np.arange(t.RING_N) * 360.0 / t.RING_N - deg + 180.0) % 360.0 - 180.0
        hit = np.abs(ang) <= max(width, 1e-9)
        d[t.N_GRID: t.N_GRID + t.RING_N][hit] = v
        return d
    quad = [t.N_GRID + k for k in range(0, t.RING_N, t.RING_N // 4)]  # the old four points
    cases = [
        ("nothing", flat(0.0), {}, 0.0),
        ("a facet on one side", facet, {}, 0.25),
        ("ground all over, 0.12 mm (light, lifted early)", flat(0.12), {}, 0.5),
        ("to the line, left on the paper", flat(0.3), {}, 0.75),
        ("to the line, wedge 0.112 mm", wedge, {}, 0.25),
        ("centre ground 0.12 mm past the edge points (dished)", centre(0.25, 0.37), dict(on_rest=True, clear=True, still_done=True), 0.25),
        ("centre over 0.37 mm, edge points at the line", centre(0.30, 0.40), dict(on_rest=True, clear=True, still_done=True), 0.0),
        ("edge points 0.06, centre 0.04 (not all 0.05)", centre(0.06, 0.04), {}, 0.25),
        ("to the line, on the rest, clear, still", flat(0.3), dict(on_rest=True, clear=True, still_done=True), 1.0),
        ("to the line, on the rest, hand still on it", flat(0.3), dict(on_rest=True, clear=False, still_done=False), 0.75),
        ("past the line, 0.5 mm", flat(0.5), dict(on_rest=True, clear=True, still_done=True), 0.0),
        ("to the line but one edge point 0.45 mm", spot(0.45, 0), dict(on_rest=True, clear=True, still_done=True), 0.0),
        ("to the line but a spot 0.45 mm at 45 deg, between the old quadrant points", spot(0.45, 45), dict(on_rest=True, clear=True, still_done=True), 0.0),
        ("to the line but a spot 0.38 mm at 150 deg (one stop 0.01 over)", spot(0.38, 150), dict(on_rest=True, clear=True, still_done=True), 0.0),
        ("to the line but a 20 deg wide spot 0.40 mm at 100 deg", spot(0.40, 100, 10), dict(on_rest=True, clear=True, still_done=True), 0.0),
        ("to the line, a stop at 0.37 mm exactly (the cap itself)", spot(0.37, 45), dict(on_rest=True, clear=True, still_done=True), 1.0),
        ("a dip only 0.20 mm at 45 deg (under the line, one stop)", spot(0.20, 45), dict(on_rest=True, clear=True, still_done=True), 0.5),
        ("to the line, thrown", flat(0.3), dict(thrown=True), 0.0),
        ("to the line, a finger on the paper", flat(0.3), dict(on_rest=True, clear=True, still_done=True, touched=True), 0.0),
        ("0.18 mm, on the rest", flat(0.18), dict(on_rest=True, clear=True, still_done=True), 0.5),
    ]
    for label, depth, flags, want in cases:
        sc, _, _ = t.rungs(depth, **flags)
        ok = abs(sc - want) < 1e-9
        print(f"   {'ok ' if ok else 'BAD'} {label:48s} {sc:.2f} (text: {want:.2f})")
        if not ok:
            bad.append(label)
    # a failure stays failed: the same spot read by the old four points alone would have missed it
    d45 = spot(0.45, 45)
    if d45[quad].max() > t.HIGH_MM:
        bad.append("the spot test is not between the old quadrant points")
    print(f"   the 45 deg spot at 0.45 mm: the old four points read {np.round(d45[quad], 2)} (they would have passed it); the ring reads max {d45[t.N_GRID:].max():.2f}")
    vp = np.array([[1.0, 0.0]] * 3)
    shake = t.rel_speed(vp, -np.array([[1.0, 0.0]] * 3))  # mount shaken against the platen's travel at its own speed
    if shake.max() > 1.0 + 1e-9:
        bad.append("shaking beats the platen's speed")
    print(f"   shaking at the platen's own speed: relative speed capped at {shake.max():.2f} (platen 1.00)")
    for label, got, want in [
        ("force with the mount off the paper (the other arm on the platen)", t.loaded_elsewhere(10.0, False), True),
        ("push over 25 N", t.loaded_elsewhere(30.0, True), True),
        ("a 15 N push through the mount", t.loaded_elsewhere(15.0, True), False),
        ("dropped onto the rest at 0.5 m/s, hand clear", t.dropped(True, 0.5, 0.06, True, 0.06), True),
        ("flicked onto the machine deck, hand clear", t.dropped(True, 0.0, 0.12, False, 0.06), True),
        ("set down gently on the rest, hand clear", t.dropped(True, 0.0, 0.06, True, 0.06), False),
        ("carried fast, hand holding it", t.dropped(False, 0.8, 0.2, False, 0.06), False),
    ]:
        print(f"   {'ok ' if got == want else 'BAD'} {label}")
        if got != want:
            bad.append(label)
    print("CHECK:", "FAIL " + "; ".join(bad) if bad else "PASS")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
