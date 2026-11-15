"""The score lines on hand-built states, with no simulator: python3 check_score.py. Runs `_tier` from task.py, read from its source."""

import ast
import pathlib
from types import SimpleNamespace

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign) and any(isinstance(n, ast.Name) and n.id in ("SUCCESS_HOLD_S", "HOLD_STEPS", "BAND_ON") for t in node.targets for n in ast.walk(t)):
        exec(ast.get_source_segment(src, node).split("  #")[0], ns)
    if isinstance(node, ast.FunctionDef) and node.name == "_tier":
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
tier, HOLD = ns["_tier"], ns["HOLD_STEPS"]
assert HOLD == 40 and ns["SUCCESS_HOLD_S"] == 0.0, "the scorer counts its own 2 s hold (40 steps at 20 Hz); the harness adds none"


def state(clamped=True, off_mm=0.3, rod_mm=0.0, clear=True, stripped=False, cycled=True, home_mm=0.3, off_ok=True, on_ok=True, push_ok=True):
    return SimpleNamespace(last={"clamped": clamped, "off_centre_mm": off_mm, "rod_mm": rod_mm, "clear": clear}, stripped=stripped, cycle_done=cycled,
                           home_dist=home_mm / 1000.0, cycle_ok_off=off_ok, cycle_ok_on=on_ok, push_ok=push_ok)


# The tier now; 1.0 is then held 2 s by the scorer before score() returns it (score() caps the tier at 0.6 until then).
CASES = {
    "clamped 0.3 mm from the band's middle, the cycle switching the LED off and on, the push held, hands clear": (state(), 1.0),
    "the same, 1.2 mm from the middle": (state(home_mm=1.2), 0.6),
    "clamped at the middle, but a hand still within reach": (state(clear=False), 0.6),
    "clamped at the middle, but the clamp gives under the 10 N push": (state(push_ok=False), 0.6),
    "clamped at the middle, but the LED did not go out with the rod out": (state(off_ok=False), 0.6),
    "clamped 4 mm from the middle (outside the band), cycle done": (state(home_mm=4.0), 0.0),
    "clamped in the band, not yet cycled, rod home": (state(cycled=False, off_mm=2.0), 0.6),
    "clamped in the band, not yet cycled, rod 5 mm out": (state(cycled=False, off_mm=2.0, rod_mm=5.0), 0.3),
    "clamped outside the band, not yet cycled": (state(cycled=False, off_mm=12.0), 0.0),
    "centred in the band but never clamped": (state(clamped=False, cycled=False), 0.0),
    "clamped at the middle, but the screw turned past 200 degrees (stripped)": (state(stripped=True), 0.0),
}
bad = 0
for name, (st, want) in CASES.items():
    got = tier(st)
    good = got == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: {got}")
print("all as stated" if not bad else f"{bad} case(s) differ from the scoring text")
raise SystemExit(1 if bad else 0)
