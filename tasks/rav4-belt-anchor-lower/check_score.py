"""The scorer's tiers and the pawl's notch rule on hand-built records, with no simulator: python3 check_score.py.
Reads judge, notch_of and the constants from task.py's source."""

import ast
import os
import pathlib

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {"os": os}
for node in ast.parse(src).body:
    if isinstance(node, ast.FunctionDef) and node.name in ("judge", "notch_of"):
        exec(ast.get_source_segment(src, node), ns)
    if isinstance(node, ast.Assign) and all(isinstance(t, ast.Name) and t.id.isupper() for t in node.targets):
        try:
            exec(ast.get_source_segment(src, node), ns)
        except Exception:
            pass
judge, notch_of, H = ns["judge"], ns["notch_of"], ns["HOLD_STEPS"]
done = {"forced": False, "moved": True, "bottom": True, "locked_low": True, "clear": True, "held": H}
cases = [
    ("done, hands off 3 s", {}, 1.0),
    ("nothing (pulled with the button out, or tapped)", {"moved": False, "bottom": False, "locked_low": False, "clear": False, "held": 0}, 0.0),
    ("let go half way: locked at a middle notch", {"bottom": False, "locked_low": False, "clear": False, "held": 0}, 0.33),
    ("at the bottom, button still held in", {"locked_low": False, "clear": False, "held": 0}, 0.67),
    ("locked low, a hand left on the anchor", {"clear": False, "held": 0}, 0.67),
    ("locked low, hands off under 3 s", {"held": H - 1}, 0.67),
    ("forced past the pawl", {"forced": True}, 0.0),
    ("slid to the bottom, then pushed back up to the top and locked", {"moved": False, "bottom": False, "locked_low": False, "clear": False, "held": 0}, 0.0),
]
bad = 0
for name, change, want in cases:
    got = judge({**done, **change})
    bad += got != want
    print(f"{'ok ' if got == want else 'BAD'} {name}: {got} (want {want})")
for z, want in ((0.0, 0), (-0.007, 0), (-0.008, 1), (-0.0329, 2), (-0.058, 4), (-0.07, 4)):
    got = notch_of(z)
    bad += got != want
    print(f"{'ok ' if got == want else 'BAD'} notch_of({z}) = {got} (want {want})")
raise SystemExit(bad)
