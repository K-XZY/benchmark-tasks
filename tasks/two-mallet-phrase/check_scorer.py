"""The scorer on hand-built states, with no simulator and no GPU. Run it with the harness's Python, from this folder: python check_scorer.py"""

from __future__ import annotations

import sys

import numpy as np

import task as t


def trial(notes: list[tuple[float, str, str]], order: tuple) -> t.Trial:
    """Feeds notes to the scorer's own rule, the head over the bar that the note names unless the mallet's name says otherwise."""
    tr = t.Trial()
    tr.reset()
    for when, bar, who in notes:
        tr.t = when
        heads = {"mallet_l": np.array([0.0, 9.0, 0.0]), "mallet_r": np.array([0.0, -9.0, 0.0])}
        heads[who] = np.array([t.BAR_X, t.BARS_Y[bar], t.TOP + t.BAR_TOP + t.HEAD_R])
        tr._note(bar, heads, order)
    return tr


def main() -> int:
    bad = 0

    def check(name: str, got, want) -> None:
        nonlocal bad
        ok = got == want
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {name}: {got}")

    own = {"L": "mallet_l", "R": "mallet_r"}
    order = t.CARDS[0]
    right = [(i * 0.5, b, own[b[0]]) for i, b in enumerate(order)]
    print("The table")
    check("six right, laid down", t.task_score(False, False, False, 6, True)[0], 1.0)
    check("six right, not laid down", t.task_score(False, False, False, 6, False)[0], 0.9)
    check("three right", t.task_score(False, False, False, 3, False)[0], 0.45)
    check("nothing", t.task_score(False, False, False, 0, False)[0], 0.0)
    check("a wrong note, whatever else", t.task_score(True, False, False, 6, True)[0], 0.0)
    check("a mallet dropped", t.task_score(False, True, False, 6, True)[0], 0.0)
    check("a bar pressed and held", t.task_score(False, False, True, 6, True)[0], 0.0)
    print("The notes")
    tr = trial(right, order)
    check("the card, on the beat", (tr.good, tr.wrong), (6, False))
    tr = trial([(i * 0.59, b, w) for i, (_, b, w) in enumerate(right)], order)
    check("gaps 0.59 s: within 20 percent", (tr.good, tr.wrong), (6, False))
    tr = trial([(i * 0.65, b, w) for i, (_, b, w) in enumerate(right)], order)
    check("gaps 0.65 s: only the first counts, no zero", (tr.good, tr.wrong), (1, False))
    swapped = list(right)
    swapped[2], swapped[3] = swapped[3], swapped[2]
    tr = trial([(i * 0.5, b, w) for i, (_, b, w) in enumerate(swapped)], order)
    check("two notes swapped: wrong", (tr.good, tr.wrong), (2, True))
    tr = trial([(0.0, order[0], "mallet_r")], order)
    check("the right bar with the other arm's mallet: wrong", tr.wrong, True)
    tr = trial(right + [(3.0, order[0], own[order[0][0]])], order)
    check("a seventh note: wrong", tr.wrong, True)
    near = np.array([t.BAR_X, t.BARS_Y[order[0]], t.TOP + t.BAR_TOP + t.HEAD_R])
    far = {"mallet_l": np.array([0.0, 9.0, 0.0]), "mallet_r": np.array([0.0, -9.0, 0.0])}
    tr = t.Trial()
    tr.reset()
    tr._note(order[0], far, order)
    check("a bar put down with no head near, by a hand or a shaft: wrong", tr.wrong, True)
    tr = t.Trial()
    tr.reset()
    tr._note(order[0], {**far, own[order[0][0]]: near + [0.0, 0.0, 0.01]}, order)
    check("the head over the bar, but 1 cm above it: wrong", tr.wrong, True)
    tr = t.Trial()
    tr.reset()
    tr._note(order[0], {**far, own[order[0][0]]: near + [0.0, t.BAR[1] / 2 + t.HEAD_R + 0.005, 0.0]}, order)
    check("the head low, beside the bar: wrong", tr.wrong, True)
    tr = t.Trial()
    tr.reset()
    tr._note(order[0], {**far, own[order[0][0]]: near + [0.0, 0.0, 0.004]}, order)
    check("the head 4 mm above the bar's top, as the bar rebounds: right", tr.wrong, False)
    print("The cards")
    check("each card takes turns", all(all(a[0] != b[0] for a, b in zip(c, c[1:])) for c in t.CARDS), True)
    check("each card is six different bars", all(len(set(c)) == 6 for c in t.CARDS), True)
    print("All agree" if not bad else f"{bad} DISAGREE")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
