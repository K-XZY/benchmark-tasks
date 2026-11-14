"""The score lines on hand-built states, with no simulator: python3 check_score.py. Runs `judge` from task.py, read from its source."""

import ast
import pathlib

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign) and any(isinstance(n, ast.Name) and n.id in ("SUCCESS_HOLD_S", "HOLD_STEPS") for t in node.targets for n in ast.walk(t)):
        exec(ast.get_source_segment(src, node).split("  #")[0], ns)
    if isinstance(node, ast.FunctionDef) and node.name == "judge":
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
judge, HOLD = ns["judge"], ns["HOLD_STEPS"]
assert HOLD == 40 and ns["SUCCESS_HOLD_S"] == 0.0, "the scorer counts its own 2 s hold (40 steps at 20 Hz); the harness adds none"

# (dropped, back latched at the marked U, front latched at the marked U, front latched at any U, level, hands clear, held steps) -> (full, score)
CASES = {
    "both ends at the marked U, level, hands clear, held 2 s": ((False, True, True, True, True, True, HOLD), (True, 1.0)),
    "the same, held 1.95 s": ((False, True, True, True, True, True, HOLD - 1), (True, 0.25)),
    "both at the marked U, but 0.6 degrees out of level": ((False, True, True, True, False, True, HOLD), (False, 0.25)),
    "both at the marked U, a hand still on the rail": ((False, True, True, True, True, False, HOLD), (False, 0.25)),
    "the back at the marked U, the front a U off (sloping)": ((False, True, False, True, False, True, HOLD), (False, 0.25)),
    "the back at the marked U only": ((False, True, False, False, True, True, HOLD), (False, 0.25)),
    "both ends a U off the marked one": ((False, False, False, True, True, True, HOLD), (False, 0.0)),
    "nothing latched": ((False, False, False, False, True, True, HOLD), (False, 0.0)),
    "both at the marked U, but the rail fell after the back latched": ((True, True, True, True, True, True, HOLD), (False, 0.0)),
}
bad = 0
for name, (args, want) in CASES.items():
    got = judge(*args)
    good = got == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: {got}")
print("all as stated" if not bad else f"{bad} case(s) differ from the scoring text")
raise SystemExit(1 if bad else 0)
