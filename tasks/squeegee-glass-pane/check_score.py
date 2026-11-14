"""The scoring table on hand-built records, no simulator: python check_score.py"""
import importlib.util
import pathlib
import sys
import types

# task.py imports the simulator's modules and builds its parts at import time; the table (_tier) needs none of them


class _Any:
    """Stands in for any simulator object or call."""

    def __init__(self, *a, **kw):
        pass

    def __call__(self, *a, **kw):
        return _Any()

    def __getattr__(self, name):
        return _Any()


def _module(name):
    mod = types.ModuleType(name)
    mod.__getattr__ = lambda attr: _Any()
    return mod


for name in ("demokit", "mechkit", "yam_arena", "yam_arena.task_file"):
    sys.modules[name] = _module(name)
sys.modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.0}
mk = sys.modules["mechkit"]
mk.col_box = lambda *a, **kw: "float physxCollision:contactOffset = 0.004"  # _torsional edits the collider's text
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)
bad = 0

# the scored grid: points map to the cells drawn on the glass, and the bands along the rails score nothing
TOP, Y0, Z0 = task.TOP, task.PANE_Y, task.TOP + task.GZ0
grid = [
    ("inside the 7th column and 6th row", (task.GX, Y0 + 0.0125, Z0 + task.WET_Z0 + 5.5 * task.CELL), {(6, 5)}),
    ("1 cm above the bottom rail (in the band left out)", (task.GX, Y0, Z0 + 0.010), set()),
    ("3 cm above the bottom rail (the bottom row)", (task.GX, Y0 - 0.14, Z0 + 0.030), {(0, 0)}),
    ("1 cm under the top rail (in the band)", (task.GX, Y0, Z0 + task.GH - 0.010), set()),
    ("3 cm under the top rail (the top row)", (task.GX, Y0 + 0.14, Z0 + task.GH - 0.030), {(task.NY - 1, task.NZ - 1)}),
    ("1 cm from a side rail (in the band)", (task.GX, Y0 + task.GW / 2 - 0.010, Z0 + 0.10), set()),
]
for name, pt, want in grid:
    got = task.cell_index([pt])
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} cells {sorted(got)} (want {sorted(want)})  {name}")
# every drawn cell's middle maps to that cell
drawn_ok = all(task.cell_index([(task.GX, y, z)]) == {(i, j)} for i, y in enumerate(task.CELL_Y) for j, z in enumerate(task.CELL_Z))
bad += not drawn_ok
print(f"{'ok ' if drawn_ok else 'BAD'} every drawn cell's middle scores as that cell ({task.NY} x {task.NZ})")

base = {"clear": 0.0, "towel_on_glass": False, "dropped": False, "sq_down": False, "hands_clear": False}
done = {"clear": 0.97, "sq_down": True, "hands_clear": True}
cases = [
    ("the full solution: 97 % of the cells clear, the squeegee on its stand, let go, hands away from the pane", done, 1.0),
    ("exactly 95 % clear, tidied", {**done, "clear": 0.95}, 1.0),
    ("94.4 % clear (8 of 144 cells wet), tidied", {**done, "clear": 136 / 144}, round(0.9 * (136 / 144) / 0.95, 3)),
    ("97 % clear, but the squeegee still in the hand on its stand (hands not clear)", {**done, "hands_clear": False}, 0.9),
    ("97 % clear, the squeegee let go off its stand", {**done, "sq_down": False}, 0.9),
    ("half the cells clear (a flat or light blade clears none; one good stroke)", {"clear": 0.5}, round(0.9 * 0.5 / 0.95, 3)),
    ("the towel touched to the glass (it dries the glass itself)", {**done, "towel_on_glass": True}, 0.0),
    ("the squeegee dropped off the bench", {**done, "dropped": True}, 0.0),
    ("nothing done", {}, 0.0),
]
for name, over, want in cases:
    got = task._tier({**base, **over})
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.3f} (want {want:.3f})  {name}")
sys.exit(1 if bad else 0)
