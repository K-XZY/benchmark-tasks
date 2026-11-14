"""The scorer on hand-built states, with no simulator: the turns, the minute, the draw with the squeeze following the head, the bin and the hold.

    python check_score.py

Exits non-zero if any case gives another value than the scoring text.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys
import types
from dataclasses import dataclass

sys.modules["yam_arena"] = types.ModuleType("yam_arena")  # the harness is not needed: only its names, so that mechkit's meshes can be built
sys.modules["yam_arena"].__path__ = []
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")


@dataclass(frozen=True)
class Box:  # the harness's shape, its fields only
    name: str
    pos: tuple
    color: tuple = (0.5, 0.5, 0.5)
    mass: float | None = None
    yaw: float = 0.0
    size: tuple = (0.04, 0.04, 0.04)


@dataclass(frozen=True)
class ObjectState:
    pos: object
    quat_xyzw: object
    lin_vel: object
    ang_vel: object
    low: object
    high: object


tf.TABLE_TOP, tf.Box, tf.ObjectState = {"station": 0.0}, Box, ObjectState
sys.path.insert(0, str(pathlib.Path(__file__).parent))
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

bad = 0


def check(label, got, want):
    global bad
    ok = got == want if isinstance(want, bool) else abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {label}: {got} (want {want})")


LO = t.ZB + 0.003  # the head's tip at the tube's bottom


def st(turn=0.0, pinched=True, lo=LO, in_tube=True, in_bin=False, clear=False, fell=False):
    """One step's state: the swab turned `turn` turns, its head's tip at `lo`."""
    a = (turn % 1.0) * 2 * math.pi
    overlap = min(lo + t.HEAD_L, t.ZB + t.PZ1) - max(lo, t.ZB + t.PZ0)
    return {"spin": math.atan2(math.sin(a), math.cos(a)), "pinched": pinched and in_tube and overlap > 0.005, "in_tube": in_tube,
            "at_bottom": lo - (t.ZB + 0.001) < t.BOTTOM, "overlap": overlap if in_tube else -1.0, "off": 0.0 if in_tube else 0.05, "head_lo": lo,
            "in_bin": in_bin, "clear": clear, "fell": fell}


def run(states):
    tr = t.TRIAL
    tr.reset()
    for s in states:
        t.now = lambda env, s=s: s
        tr.step(None)
    return tr


def turns(n=5.2):
    return [st(turn=0.05 * k) for k in range(int(round(n / 0.05)) + 1)]


T5 = 0.05 * (len(turns()) - 1)


def wait(steps=t.WAIT_STEPS, pinched=True):
    return [st(turn=T5, pinched=pinched)] * steps


def draw(gap_at=None, step=0.001, turn_each=0.004, pause_open=False, gap=0.001):
    """The head drawn from the bottom to the mouth, 1 mm a step, turning; `gap_at`: a rise of 1 mm there with no squeeze on the head; `pause_open`: every
    15 mm the squeeze let go while the head stays still (the hand moving up)."""
    out, lo, k = [], LO, 0
    while lo < t.ZB + t.PZ1:
        k += 1
        lo += step
        out.append(st(turn=T5 + turn_each * k, lo=lo, pinched=not (gap_at is not None and -0.0005 < lo - gap_at < gap - 0.0005)))
        if pause_open and k % 15 == 0:
            out += [st(turn=T5 + turn_each * k, lo=lo, pinched=False)] * 10
    return out


out_of_tube = [st(turn=T5, lo=t.ZB + t.T_LEN + 0.02, in_tube=False)] * 5
binned = [st(turn=T5, lo=0.0, in_tube=False, in_bin=True, clear=True)]
check("doing nothing scores 0", run([st(pinched=False)] * 50).score, 0.0)
check("turned 5 times with the tube not squeezed: 0", run([st(turn=0.05 * k, pinched=False) for k in range(105)]).score, 0.0)
check("turned 2.2 times squeezed at the bottom: 0.24", run(turns(2.2)).score, 0.24)
check("turned 5.2 times squeezed at the bottom: 0.6", run(turns()).score, 0.6)
check("turned 5 times, then 5 back: 0", run(turns() + [st(turn=T5 - 0.05 * k) for k in range(1, 105)]).score, 0.0)
check("5 turns, then the swab in the tube 59.95 s from the step the fifth turn ended: still 0.6", run(turns(5.0) + wait(t.WAIT_STEPS - 2)).score, 0.6)
check("5 turns, then 60 s: 0.7", run(turns(5.0) + wait(t.WAIT_STEPS - 1)).score, 0.7)
check("after the turns, the minute in the tube (squeeze let go): 0.7", run(turns() + wait(pinched=False)).score, 0.7)
check("the minute, then drawn squeezed all the way, turning: 0.8", run(turns() + wait() + draw()).score, 0.8)
check("the same, the squeeze eased while the head stays still, every 15 mm (the hand moving up): 0.8", run(turns() + wait() + draw(pause_open=True)).score, 0.8)
check("the minute, then a 2 mm rise with no squeeze on the head at 40 mm: 0.7", run(turns() + wait() + draw(gap_at=t.ZB + 0.040, gap=0.002)).score, 0.7)
check("a 1 mm rise with no squeeze (within the 1 mm slack for an eased squeeze): 0.8", run(turns() + wait() + draw(gap_at=t.ZB + 0.040)).score, 0.8)
check("drawn squeezed but turning only 0.07 turn: 0.7", run(turns() + wait() + draw(turn_each=0.001)).score, 0.7)
yank = turns() + wait() + [st(turn=T5 + 0.004 * k, lo=LO + 0.0005 * k) for k in range(1, 61)]  # off the bottom to 33 mm, squeezed, turning 0.24
creep = turns() + wait() + [st(turn=T5, lo=LO + 0.0005 * k) for k in range(1, 25)] + [st(turn=T5 + 0.01 * k, lo=LO + 0.012) for k in range(1, 21)]
# off the bottom to 15 mm squeezed, turned 0.2 there; then the squeeze let go and the head crept up
check("off the bottom squeezed and turned 0.2, then the squeeze let go and the head crept to the mouth at 0.25 mm a step: 0.7",
      run(creep + [st(turn=T5 + 0.2, lo=LO + 0.012 + 0.00025 * k, pinched=False) for k in range(1, 240)]).score, 0.7)
slow = [st(turn=T5 + 0.002 * k, lo=LO + 0.00025 * k, pinched=LO + 0.00025 * k < t.ZB + 0.030) for k in range(1, 300)]  # a fixed squeeze low down
check("a squeeze left low (the head squeezed only below 30 mm), the head pulled through it at 0.25 mm a step, turning: 0.7",
      run(turns() + wait() + slow).score, 0.7)
check("the same squeeze, the head pulled through at 1.3 mm a step: 0.7",
      run(turns() + wait() + [st(turn=T5 + 0.01 * k, lo=LO + 0.0013 * k, pinched=LO + 0.0013 * k < t.ZB + 0.030) for k in range(1, 60)]).score, 0.7)
steps = []  # the demo's pattern: 10 mm lifts squeezed, then the squeeze eased while the head stands still and drifts up 0.5 mm, then squeezed again
lo, k = LO, 0
while lo < t.ZB + t.PZ1:
    for _ in range(20):
        k += 1
        lo += 0.0005
        steps.append(st(turn=T5 + 0.003 * k, lo=lo))
    steps += [st(turn=T5 + 0.003 * k, lo=lo + 0.0005 * j / 10, pinched=False) for j in range(1, 11)]
    lo += 0.0005
check("the demo's draw: squeezed 10 mm lifts, the squeeze eased between them with 0.5 mm of drift: 0.8", run(turns() + wait() + steps).score, 0.8)
check("drawn squeezed and turned 0.24 to 33 mm, then yanked to the mouth between two frames: 0.7",
      run(yank + [st(turn=T5 + 0.24, lo=t.ZB + t.PZ1 - 0.005, pinched=False)]).score, 0.7)
check("the same, the yank's last frame squeezed: 0.7", run(yank + [st(turn=T5 + 0.24, lo=t.ZB + t.PZ1 - 0.005)]).score, 0.7)
check("drawn squeezed and turning, but before the minute: 0.6", run(turns() + wait(600) + draw()).score, 0.6)
check("lifted early, put back to the bottom, a full minute, then drawn: 0.8",
      run(turns() + wait(600) + [st(turn=T5, lo=LO + 0.03)] * 5 + [st(turn=T5, lo=LO)] * 3 + wait() + draw()).score, 0.8)
check("drawn, out, in the bin, hands clear for 19 steps: 0.8", run(turns() + wait() + draw() + out_of_tube + binned * 19).score, 0.8)
tr = run(turns() + wait() + draw() + out_of_tube + binned * 20)
check("for 20 steps (1 s): 1.0", tr.score, 1.0)
check("success() then", t.success(None), True)
check("the swab off the table once: 0 for the rest", run(turns() + wait() + draw() + [st(turn=T5, lo=-0.02, in_tube=False, fell=True)] + binned * 30).score, 0.0)
print("all cases as the scoring text" if not bad else f"{bad} cases differ")
sys.exit(1 if bad else 0)
