"""The score lines on hand-built states, with no simulator: python3 check_score.py. Runs `judge` and `kerf_width` from task.py, read from its source."""

import ast
import math
import pathlib
import sys

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {"math": math}
keep = {"TICKS", "RIBBON_W", "THIN_KERF", "GOOD_OFF", "GOOD_KERF", "BLOCK_D", "BLOCK_W", "AMP", "WAVE", "HALF", "SUCCESS_HOLD_S", "HOLD_STEPS", "BOARD_T", "RAIL_H", "FOAM_Z0", "RIBBON_LOW", "GASH_OFF", "START_OFF"}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign) and any(n.id in keep for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)):
        exec(ast.get_source_segment(src, node).split("  #")[0], ns)
    if (isinstance(node, ast.FunctionDef) and node.name in ("judge", "kerf_width", "curve_slope", "curve_y")) or (isinstance(node, ast.ClassDef) and node.name == "Kerf"):
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
judge, kerf_width, curve_slope, curve_y, Kerf = ns["judge"], ns["kerf_width"], ns["curve_slope"], ns["curve_y"], ns["Kerf"]
assert ns["HOLD_STEPS"] == round(ns["SUCCESS_HOLD_S"] * 20), "HOLD_STEPS must be SUCCESS_HOLD_S at 20 Hz"
GOOD_KERF, HALF, BLOCK_D, TICKS = ns["GOOD_KERF"], ns["HALF"], ns["BLOCK_D"], ns["TICKS"]
assert ns["BOARD_T"] < ns["RIBBON_LOW"] < ns["FOAM_Z0"], "the ribbon must end between the board and the foam"

# (good run's share, through edge to edge, gash, second cut, bad start, board cut, block clear of the ribbon, fingers clear, still[, steps held]), wanted (full, score)
CASES = {
    "through on the line, clear, at rest, for 1 s": ((1.0, True, False, False, False, False, True, True, True, 20), (True, 1.0)),
    "through on the line, clear, at rest, for 0.95 s": ((1.0, True, False, False, False, False, True, True, True, 19), (True, 0.75)),
    "through on the line, a hand still on": ((1.0, True, False, False, False, False, True, False, True), (False, 0.75)),
    "through on the line, block still on the ribbon": ((1.0, True, False, False, False, False, False, True, True), (False, 0.75)),
    "through on the line, block still moving": ((1.0, True, False, False, False, False, True, True, False), (False, 0.75)),
    "on the line to the middle, then stopped": ((0.5, False, False, False, False, False, False, True, True), (False, 0.375)),
    "on the line to the middle, then 1 cm off to the near edge": ((0.5, False, False, False, False, False, True, True, True), (False, 0.375)),
    "on the line almost to the near edge, then 3 cm sideways: a gash": ((0.93, False, True, False, False, False, True, True, True), (False, 0.0)),
    "through on the line, then back in: a second cut": ((1.0, True, False, True, False, False, True, True, True), (False, 0.0)),
    "cut starts over 1 cm from the curve's start": ((0.0, False, False, False, True, False, True, True, True), (False, 0.0)),
    "board lifted onto the ribbon": ((1.0, True, False, False, False, True, True, True, True), (False, 0.0)),
    "no cut": ((0.0, False, False, False, False, False, True, True, True), (False, 0.0)),
}
bad = 0
for name, (args, want) in CASES.items():
    full, score = judge(*args)
    good = (full, score) == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: full {full}, score {score}")
# Whole cuts, as the ribbon moves through the block's frame: (u along the block's x, v across it, the block's turn from the ribbon's flat).
def on_line(u: float, turning: bool = True, side: float = 0.0):
    c = min(max(u, -HALF), HALF)
    return (u, curve_y(c) + side, -math.atan(curve_slope(c)) if turning else 0.0)


def run(points):
    k = Kerf()
    for p in points:
        k.feed(*p)
    return k


def span(a: float, b: float, step: float = 0.0005):
    n = int(round(abs(b - a) / step))
    return [a + (b - a) * i / n for i in range(n + 1)]


LAST_TICK = HALF - (TICKS - 0.5) * BLOCK_D / TICKS
far_in, near_out = HALF + 0.02, -HALF - 0.02
WHOLE = {
    "on the line, turning, edge to edge": ([on_line(u) for u in span(far_in, near_out)], dict(through=True, gash=False, second=False, bad_start=False), (1.0, 1.0)),
    "on the line without turning: the kerf widens": ([on_line(u, False) for u in span(far_in, near_out)], dict(through=False, gash=False), (0.05, 0.15)),
    "past every tick, then sideways out of the side": ([on_line(u) for u in span(far_in, LAST_TICK - 0.001)] + [on_line(LAST_TICK - 0.001, True, s) for s in span(0.0, 0.12)], dict(through=False, gash=True), (0.85, 0.95)),
    "to the middle, then back out of the far edge": ([on_line(u) for u in span(far_in, 0.0)] + [on_line(u) for u in span(0.0, far_in)], dict(through=False, gash=False, second=False), (0.49, 0.51)),
    "1 cm off the line from the middle on": ([on_line(u) for u in span(far_in, 0.0)] + [on_line(u, True, 0.01 * min(1.0, -u / 0.005)) for u in span(0.0, near_out)], dict(through=False, gash=False), (0.50, 0.53)),  # off by 5 mm 2.5 mm past the middle
    "through, then back in at the near edge": ([on_line(u) for u in span(far_in, near_out)] + [on_line(u) for u in span(near_out, -HALF + 0.02)], dict(through=True, second=True), (1.0, 1.0)),
    "starts 2 cm off the curve's start": ([on_line(u, True, 0.02) for u in span(far_in, HALF - 0.01)], dict(bad_start=True), (0.0, 0.0)),
}
for name, (points, want, (lo, hi)) in WHOLE.items():
    k = run(points)
    got = {key: getattr(k, key) for key in want}
    good = got == want and lo <= k.good_share() <= hi
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: {got}, good run {k.good_share():.3f}")

steepest = max(abs(math.degrees(math.atan(curve_slope(HALF * k / 100.0)))) for k in range(-100, 101))
print(f"steepest turn of the curve: {steepest:.1f} deg; kerf if the block does not turn there: {kerf_width(math.radians(steepest)) * 1000:.1f} mm, limit {GOOD_KERF * 1000:.0f} mm")
if kerf_width(math.radians(steepest)) <= GOOD_KERF:
    bad += 1
    print("BAD a block that does not turn passes the kerf limit")
print("all good" if not bad else f"{bad} FAILED")
sys.exit(1 if bad else 0)
