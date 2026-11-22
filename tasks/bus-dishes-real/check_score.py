"""The scorer on hand-built poses, with no simulator: python3 check_score.py. Runs plate_place, look and Trial from task.py, read from its
source, on the placements and the routes a reviewer would try."""

import ast
import collections
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gatekit as gk  # noqa: E402

src = (HERE / "task.py").read_text()
ns = {"np": np, "math": math, "collections": collections, "gk": gk, "TOP": 0.0, "TRACE": False}
keep = {"HOLD_STEPS", "P_R", "P_H", "P_EDGE_LOW", "P_FOOT", "TUB_TOP", "TUB_BOTTOM", "TUB_H", "TUB_WALL", "TUB_FLOOR", "PLATES", "DROP", "TOUCH", "CLEAR",
        "OPEN_MAX", "REST_V", "REST_STEPS", "STILL", "_RING", "P_POINTS"}
funcs = {"world", "lowest", "tip_gap", "plate_place", "look", "_box_gap", "Trial"}
for node in ast.parse(src).body:
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in funcs:
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
    if isinstance(node, ast.Assign) and any(isinstance(n, ast.Name) and n.id in keep for t in node.targets for n in ast.walk(t)):
        exec(ast.get_source_segment(src, node), ns)
plate_place, Trial = ns["plate_place"], ns["Trial"]
FLOOR, TUB_H, P_R = ns["TUB_FLOOR"], ns["TUB_H"], ns["P_R"]


def rot(axis, deg):
    k = np.array(axis, float) / np.linalg.norm(axis)
    a = math.radians(deg)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


I3 = np.eye(3)
TUB = (np.zeros(3), I3)  # the tub's frame: its outer bottom on the table at the origin
FAR = np.array([[5.0, 5.0, 5.0]])  # hand points far from everything
fails = 0


def check(name, got, want):
    global fails
    ok = got == want
    fails += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {got} (want {want})")


print("== in the tub: every sampled point of the plate inside the tub's walls, above its floor, below its rim")
check("flat on the floor, centred", plate_place(np.array([0.0, 0.0, FLOOR]), I3, TUB), True)
check("flat on the first plate (1.6 cm up)", plate_place(np.array([0.0, 0.0, FLOOR + 0.016]), I3, TUB), True)
check("face down on the floor", plate_place(np.array([0.0, 0.0, FLOOR + ns["P_H"]]), rot((1, 0, 0), 180), TUB), True)
check("leaning 30 deg, its low edge on the floor", plate_place(np.array([0.0, 0.0, FLOOR + P_R * math.sin(math.radians(30))]), rot((0, 1, 0), 30), TUB), True)
check("flat on the tub's rim", plate_place(np.array([0.0, 0.0, TUB_H]), I3, TUB), False)
check("flat on the table beside the tub", plate_place(np.array([0.0, 0.40, 0.0]), I3, TUB), False)
check("partly in: its rim across the wall", plate_place(np.array([0.0, 0.12, FLOOR]), I3, TUB), False)
check("standing on edge, sticking out over the rim", plate_place(np.array([0.0, 0.0, FLOOR + P_R + 0.08]), rot((1, 0, 0), 90), TUB), False)
check("under the tub", plate_place(np.array([0.0, 0.0, -0.03]), I3, TUB), False)


def run(script, open_after=None):
    """`script`: per step, (plate_r pos, plate_l pos, hand points, plate speeds). Returns the trial's score after the last step."""
    t = Trial()
    t.reset()
    for i, (pr, pl, tips, v) in enumerate(script):
        grip = tips if open_after is None or i < open_after else np.zeros((0, 3))  # open_after: the step the jaws open; the hand points stay
        t.update({"plate_r": (np.array(pr, float), I3), "plate_l": (np.array(pl, float), I3), "tub": TUB}, {"plate_r": v[0], "plate_l": v[1]}, tips, grip)
    return t.score, t.dropped, t.fail


def touching(p):
    return np.array([[p[0], p[1], p[2] + ns["P_H"] + 0.002]])


R0, L0 = [0.0, 0.0, FLOOR], [0.0, 0.0, FLOOR + 0.016]
done = [(R0, L0, FAR, (0, 0))] * 45  # 1 s of stillness history, then the 1 s hold
print("== trials")
check("both set down, hands clear, still 1 s", run(done)[0], 1.0)
lift = [(R0, [0.0, 0.0, FLOOR + 0.016 + 0.05], touching([0.0, 0.0, FLOOR + 0.016 + 0.05]), (0, 0))] * 3
check("second plate let go 5 cm up: a drop, never counts", run(done[:5] + lift + done)[0:2], (0.4, {"plate_l"}))
lift = [(R0, [0.0, 0.0, FLOOR + 0.016 + 0.03], touching([0.0, 0.0, FLOOR + 0.016 + 0.03]), (0, 0))] * 3
check("second plate let go 3 cm up: a drop, never counts", run(done[:5] + lift + done)[0:2], (0.4, {"plate_l"}))
lift = [(R0, [0.0, 0.0, FLOOR + 0.016 + 0.008], touching([0.0, 0.0, FLOOR + 0.016 + 0.008]), (0, 0))] * 3
check("second plate let go 8 mm up: set down", run(done[:5] + lift + done)[0], 1.0)
check("second plate still held", run(done[:5] + [(R0, L0, touching(L0), (0, 0))] * 30)[0], 0.4)
near = np.array([[0.0, 0.0, L0[2] + ns["P_H"] + 0.02]])
check("hand left 2 cm off the plates", run([(R0, L0, touching(L0), (0, 0))] * 3 + [(R0, L0, near, (0, 0))] * 30)[0], 0.8)
check("done, then a plate taken back out", run(done + [(R0, [0.0, 0.40, 0.0], FAR, (0, 0))] * 30)[0], 0.4)
check("a plate off the table voids the run", run(done[:25] + [(R0, [0.0, 0.40, -0.10], FAR, (0, 0))] + done)[0], 0.0)
check("nothing done", run([([0.37, 0.53, 0.0], [0.36, 0.78, 0.0], FAR, (0, 0))] * 30)[0], 0.0)
# swing: the jaws open under the near rim (hand points stay within 1 cm) while the far side is 6 cm up; the centre then falls 4.5 cm
hi = [0.0, 0.0, FLOOR + 0.016 + 0.06]
held = [(R0, hi, touching(hi), (0, 0))] * 3
fall = [(R0, [0.0, 0.0, FLOOR + 0.016 + 0.06 - 0.015 * k], touching([0.0, 0.0, FLOOR + 0.016 + 0.06 - 0.015 * k]), (0.1, 0.1)) for k in range(1, 4)]
rest = [(R0, L0, touching(L0), (0, 0))] * 8 + [(R0, L0, FAR, (0, 0))] * 50
check("jaws open under the rim, the plate then falls 4.5 cm: dropped", run(done[:5] + held + fall + rest, open_after=8)[0:2], (0.4, {"plate_l"}))
# flop: the jaws open, the plate rests still on the open lower jaw 6 cm up, and drops 6 cm once the hand slides out
perch = [(R0, hi, touching(hi), (0, 0))] * 10
check("rests on the open jaw, then drops 6 cm when the hand leaves: dropped", run(done[:5] + held + perch + rest, open_after=8)[0:2], (0.4, {"plate_l"}))
print("all ok" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
