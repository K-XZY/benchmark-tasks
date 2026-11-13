"""The scorer's table on hand-built records, with no simulator: python check_score.py (needs yam_arena importable)."""

import task as T

base = {"units": 12, "shield_in": True, "shot_ok": True, "wasted": False, "touched": False, "dropped": False, "in_tray": True, "clear": True}
cases = [
    ({}, 1.0),
    ({"shot_ok": False}, 0.2),  # no safety shot: the dose counts for nothing
    ({"units": 13}, 0.6),  # dialled one over and left there
    ({"units": 12, "in_tray": False}, 0.6),  # the dose set, the pen not put back
    ({"clear": False}, 0.6),  # still held
    ({"shield_in": False}, 0.8),  # the shield not in its tray
    ({"wasted": True}, 0.2),  # a push with the needle below horizontal
    ({"touched": True}, 0.5),  # the needle touched something
    ({"dropped": True}, 0.0),
    ({"shield_in": False, "shot_ok": False}, 0.0),
]
bad = 0
for change, want in cases:
    T.PEN.last = {**base, **change}
    T.PEN.full = T._tier(T.PEN.last) == 1.0  # held 2 s wherever every part is met
    got = T.score(None)
    ok = abs(got - want) < 1e-9 and T.success(None) == (got == 1.0)
    bad += not ok
    print(("ok  " if ok else "FAIL"), change, got, "want", want)
T.PEN.last, T.PEN.full = dict(base), False
unheld = T.score(None)  # a run ending inside the 2 s hold
print(("ok  " if unheld == 0.6 else "FAIL"), "every part met but not yet held 2 s at the end", unheld, "want", 0.6)
bad += unheld != 0.6 or T.success(None)
print("all pass" if not bad else f"{bad} failed")
