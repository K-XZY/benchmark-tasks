"""The score lines on hand-built states, and plans that do not read the figure across seeds, with no simulator: python3 check_score.py. Runs `schedule`, `belt_y` and `judge` from task.py, read from its source."""

import ast
import math
import pathlib
import sys

import numpy as np

src = pathlib.Path(__file__).with_name("task.py").read_text()
keep = {"GOOD_OUT_S", "FAIR_OUT_S", "RAIL_Y", "START_S", "LONG_S", "AHEAD", "WINDOW_DEG"}
ns = {"np": np, "math": math}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign) and any(n.id in keep for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)):
        exec(ast.get_source_segment(src, node), ns)
    if isinstance(node, ast.FunctionDef) and node.name in ("schedule", "belt_y", "judge"):
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
judge, schedule, belt_y = ns["judge"], ns["schedule"], ns["belt_y"]
START, AHEAD, WIN, FAIR = ns["START_S"], ns["AHEAD"], math.radians(ns["WINDOW_DEG"]), ns["FAIR_OUT_S"]

# (seconds outside the band, good press, lamp passed, fingers clear), wanted (full, score)
CASES = {
    "1 s outside, good press, clear": ((1.0, True, True, True), (True, 0.75)),
    "6 s outside, good press, clear": ((6.0, True, True, True), (True, 0.75)),
    "1 s outside, good press, hand still on the bar": ((1.0, True, True, False), (False, 0.75)),
    "1 s outside, late press": ((1.0, False, True, True), (False, 0.75)),
    "1 s outside, no press": ((1.0, False, True, False), (False, 0.75)),
    "10 s outside, good press, clear": ((10.0, True, True, True), (False, 0.5)),
    "13 s outside, good press, clear": ((13.0, True, True, True), (False, 0.0)),
    "press just after the belt starts: the 30 s to the lamp count as missed": ((30.0, False, True, True), (False, 0.0)),
    "press 10 s before the lamp, tracking until then: 10 s missed": ((10.0, False, True, True), (False, 0.5)),
    "before the lamp": ((0.0, False, False, False), (False, 0.0)),
}
bad = 0
for name, (args, want) in CASES.items():
    full, score = judge(*args)
    good = full == want[0] and abs(score - want[1]) < 1e-6
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: full {full}, score {score:.3f}")

seeds = range(100)
runs = []
for s in seeds:
    pts, lamp = schedule(s)
    ts = np.arange(START, lamp, 0.05)
    runs.append((ts, np.array([math.atan2(belt_y(pts, t), AHEAD) for t in ts])))
print(f"time from the belt's start to the lamp: {min(len(r[0]) for r in runs) * 0.05:.1f} to {max(len(r[0]) for r in runs) * 0.05:.1f} s")
fixed = min(min(float(np.sum(np.abs(tr - a) > WIN)) * 0.05 for _, tr in runs) for a in np.radians(np.arange(-30, 31, 2)))
best_mean, best_seed = 1e9, 1e9
for period in (6, 8, 10, 12, 14, 16, 20, 24, 30, 40, 60):
    for phase in np.linspace(0, 1, 9)[:-1]:
        for amp in (10, 15, 20, 25, 31):
            out = [float(np.sum(np.abs(tr - np.radians(amp) * (2 / np.pi) * np.arcsin(np.sin(2 * np.pi * ((ts - START) / period + phase)))) > WIN)) * 0.05 for ts, tr in runs]
            best_mean, best_seed = min(best_mean, float(np.mean(out))), min(best_seed, float(np.min(out)))
print(f"best fixed aim, luckiest schedule: {fixed:.1f} s outside")
print(f"best steady sweep of 440 plans: {best_mean:.1f} s outside on average; luckiest schedule {best_seed:.1f} s; the limit for any credit is {FAIR:.0f} s")
if best_seed <= FAIR or fixed <= FAIR:
    bad += 1
    print("BAD a plan that does not read the figure earns credit")
print("all good" if not bad else f"{bad} FAILED")
sys.exit(1 if bad else 0)
