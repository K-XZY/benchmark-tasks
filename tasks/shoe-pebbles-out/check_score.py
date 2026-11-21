"""Scorer checks on hand-built states, no simulator: python check_score.py (harness Python, from this folder)."""

import numpy as np

import task as t

ok = True


def check(name, got, want):
    global ok
    good = got == want
    ok = ok and good
    print(f"{'ok ' if good else 'BAD'} {name}: {got} (want {want})")


# a stone is in while it is anywhere within the shoe's envelope, and on the collar's lip
check("toe box stone in", bool(t.inside_shoe([0.24, 0.0, 0.032])[0]), True)
check("heel stone in", bool(t.inside_shoe([0.05, 0.01, 0.040])[0]), True)
check("stone on the collar lip, 5 mm above the edge, in", bool(t.inside_shoe([0.06, 0.030, t.top_z(0.06) + 0.005])[0]), True)
check("stone 3 mm beside the side wall, out", bool(t.inside_shoe([0.20, t.half_w(0.20) + 0.003, 0.004])[0]), False)
check("stone 4 mm behind the heel, out", bool(t.inside_shoe([-0.004, 0.0, 0.004])[0]), False)
check("stone on the vamp's top, out", bool(t.inside_shoe([0.20, 0.0, t.top_z(0.20) + 0.004])[0]), False)
check("stone on the table under the heel's edge, out", bool(t.inside_shoe([0.007, 0.002, -0.002])[0]), False)
check("stone 4 mm past the toe, out", bool(t.inside_shoe([t.L + 0.004, 0.0, 0.004])[0]), False)
# every start position is inside
for k, (x, y) in enumerate(t.START):
    check(f"pebble {k} starts in", bool(t.inside_shoe([x, y, float(t.insole_z(x)) + 0.004])[0]), True)
# hands clear: a finger on the pull tab or in the collar is not clear
check("finger on the tab not clear", bool(t.gap_to_shoe([[t.BACK + 0.003, 0.0, 0.12]])[0] >= t.CLEAR_M), False)
check("finger in the collar not clear", bool(t.gap_to_shoe([[0.06, 0.0, 0.08]])[0] >= t.CLEAR_M), False)
check("hand 4 cm above the vamp clear", bool(t.gap_to_shoe([[0.20, 0.0, t.top_z(0.20) + 0.04]])[0] >= t.CLEAR_M), True)
# tiers: count of stones out, and all out plus set down for 1.0
check("none out", t.tier(0, True), 0.0)
check("1 out", t.tier(1, True), 0.0)
check("2 out", t.tier(2, False), 0.25)
check("5 out", t.tier(5, False), 0.5)
check("7 out, set down", t.tier(7, True), 0.75)
check("8 out, not set down", t.tier(8, False), 0.75)
check("8 out, set down", t.tier(8, True), 1.0)
# the trial's score: 1.0 only after 40 steps of the full end state, 0 after a void
tr = t.Trial()
tr.reset()
tr.n_out, tr.set_down, tr.ok_steps = 8, True, 39
check("8 out set down 39 steps", tr.score(), 0.75)
tr.ok_steps = 40
check("8 out set down 40 steps", tr.score(), 1.0)
tr.void = True
check("void", tr.score(), 0.0)
# release: the sole's height at the last held step decides, whatever the shoe does after
def release(h):
    t_ = t.Trial()
    t_.reset()
    t_.n = 50
    t_.track_hold(True, h)
    t_.track_hold(False, 0.0)
    return t_.void
check("let go 2 cm up", release(0.02), True)
check("let go 1.6 cm up", release(0.016), True)
check("let go 1.4 cm up", release(0.014), False)
t_ = t.Trial()
t_.reset()
t_.n = 50
t_.track_hold(False, 0.5)
check("never held, no release void", t_.void, False)
# a held shoe is any grip of both fingertips of one gripper: a wide grip counts, and a release from it is judged the same
t_ = t.Trial()
t_.reset()
t_.n = 50
t_.track_hold(True, 0.035)
t_.track_hold(False, 0.0)
check("wide grip released 3.5 cm up, void", t_.void, True)
t_ = t.Trial()
t_.reset()
t_.n = 50
t_.track_hold(True, 0.0)
check("held marks ever_held", t_.ever_held, True)
# nothing holds it (free for 2 steps) with a sole point 2 cm up after a hold: void; one free step, or never held: no void
for steps, ever, high, want, name in [(2, True, 0.02, True, "held, 2 free steps, sole 2 cm up"), (1, True, 0.02, False, "held, 1 free step, sole 2 cm up"),
                                      (2, True, 0.01, False, "held, 2 free steps, sole 1 cm up"), (2, False, 0.02, False, "never held, sole 2 cm up")]:
    t_ = t.Trial()
    t_.reset()
    t_.n = 50
    t_.ever_held = ever
    for _ in range(steps):
        t_.free_air(False, high)
    check(name, t_.void, want)
# held needs the fingertips to move with the shoe: a finger sitting in the collar while the shoe slides off is not holding
prev_tip = np.array([[0.10, 0.0, 0.02]])  # fingertip in the shoe frame, last step
def slip_of(cur):  # the expression task.py uses
    return float(np.linalg.norm(cur - prev_tip, axis=1)[0] / t.DT)
check("fingertip fixed in the shoe frame (moving with it)", slip_of(prev_tip.copy()) < t.MATCH_V, True)
check("fingertip sliding 1 cm per step along the shoe", slip_of(prev_tip + np.array([[0.01, 0.0, 0.0]])) < t.MATCH_V, False)
print("ALL OK" if ok else "FAILURES")
