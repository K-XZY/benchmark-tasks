"""The score lines on hand-built states, with no simulator: python3 check_score.py. Runs `judge` and `kerf_width` from task.py, read from its source."""

import ast
import math
import pathlib
import sys

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {"math": math}
keep = {"TICKS", "RIBBON_W", "THIN_KERF", "GOOD_OFF", "GOOD_KERF", "BLOCK_D", "BLOCK_W", "AMP", "WAVE", "HALF"}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign) and any(n.id in keep for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)):
        exec(ast.get_source_segment(src, node).split("  #")[0], ns)
    if isinstance(node, ast.FunctionDef) and node.name in ("judge", "kerf_width", "curve_slope"):
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
judge, kerf_width, curve_slope = ns["judge"], ns["kerf_width"], ns["curve_slope"]
TICKS, GOOD_KERF, HALF = ns["TICKS"], ns["GOOD_KERF"], ns["HALF"]

# (ticks good, lifted over the ribbon or a second kerf, start off the line, block clear of the ribbon, fingers clear, still), wanted (full, score)
CASES = {
    "all ticks, clear": ((TICKS, False, False, True, True, True), (True, 0.75)),
    "all ticks, a hand still on": ((TICKS, False, False, True, False, True), (False, 0.75)),
    "all ticks, block still on the ribbon": ((TICKS, False, False, False, True, True), (False, 0.75)),
    "half the ticks": ((TICKS // 2, False, False, True, True, True), (False, 0.375)),
    "lifted over the ribbon": ((TICKS, True, False, True, True, True), (False, 0.0)),
    "cut starts over 1 cm off the line": ((TICKS, False, True, True, True, True), (False, 0.0)),
    "no cut": ((0, False, False, True, True, True), (False, 0.0)),
}
bad = 0
for name, (args, want) in CASES.items():
    full, score = judge(*args)
    good = (full, score) == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: full {full}, score {score}")
steepest = max(abs(math.degrees(math.atan(curve_slope(HALF * k / 100.0)))) for k in range(-100, 101))
print(f"steepest turn of the curve: {steepest:.1f} deg; kerf if the block does not turn there: {kerf_width(math.radians(steepest)) * 1000:.1f} mm, limit {GOOD_KERF * 1000:.0f} mm")
if kerf_width(math.radians(steepest)) <= GOOD_KERF:
    bad += 1
    print("BAD a block that does not turn passes the kerf limit")
print("all good" if not bad else f"{bad} FAILED")
sys.exit(1 if bad else 0)
