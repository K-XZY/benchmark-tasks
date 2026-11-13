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
assert HOLD == 20 and ns["SUCCESS_HOLD_S"] == 0.0, "the scorer counts its own 1 s hold (20 steps at 20 Hz); the harness adds none"

# (lost, dropped, on_rod, reach_ball, over_ball, seated_neutral, home, clear, held steps) -> (full, score)
CASES = {
    "in neutral, the screwdriver home, hands clear, held 1 s": ((False, False, True, True, True, True, True, True, HOLD), (True, 1.0)),
    "the same, held 0.95 s": ((False, False, True, True, True, True, True, True, HOLD - 1), (True, 0.5)),
    "in neutral, but the screwdriver not back in its holder": ((False, False, True, True, True, True, False, True, HOLD), (False, 0.5)),
    "in neutral, a hand still on the fork": ((False, False, True, True, True, True, True, False, HOLD), (False, 0.5)),
    "over the ball, the ball in the first groove": ((False, False, True, True, True, False, True, True, HOLD), (False, 0.5)),
    "on the rod, stopped at the unpressed ball": ((False, False, True, True, False, False, True, True, HOLD), (False, 0.25)),
    "on the rod, short of the ball": ((False, False, True, False, False, False, True, True, HOLD), (False, 0.0)),
    "in neutral, but the ball sprang out earlier": ((True, False, True, True, True, True, True, True, HOLD), (False, 0.0)),
    "in neutral, but the fork was dropped on the way": ((False, True, True, True, True, True, True, True, HOLD), (False, 0.0)),
    "left in its tray": ((False, False, False, False, False, False, True, True, HOLD), (False, 0.0)),
}
bad = 0
for name, (args, want) in CASES.items():
    got = judge(*args)
    good = got == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: {got}")
print("all as stated" if not bad else f"{bad} case(s) differ from the scoring text")
raise SystemExit(1 if bad else 0)
