"""Print the scene's key sizes against the published ones (no simulator): python -B check_sizes.py"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task as t  # noqa: E402

MM = 1000.0
ROWS = [
    ("mount diameter (mm)", 2 * t.MOUNT_R * MM, 31.75, "1.25 in epoxy mount, Allied High Tech (alliedhightech.com)"),
    ("mount height (mm)", t.MOUNT_H * MM, 35.0, "trimmed to 35 mm by us (the cup is 1.25 in; height is ours)"),
    ("platen diameter (mm)", 2 * t.PLATEN_R * MM, 203.2, "8 in platen, Allied MetPrep guide (mse.ucr.edu/media/1273/download)"),
    ("platen speed (rpm)", t.RPM, 150.0, "Buehler SumMet PCB method 120 to 150 rpm (pdfcoffee.com/bhuler-metal-preperation-pdf-free.html)"),
    ("package strip thickness (mm)", 2.86, 2.86, "FCBGA substrate, 35 x 35 x 2.86 mm (our part choice; estimate from a datasheet class)"),
    ("scribed line above the face (mm)", t.LINE_MM, 0.30, "ours: the last 0.3 mm of the P180 step"),
    ("scored ring radius (mm)", t.RING_MM, 14.0, "2 mm in from the 16 mm rim"),
    ("ring stops (simulation)", t.RING_N, 72, "every 5 degrees; the real trial's 24 stops are every third"),
    ("holder flange diameter (mm)", 2 * t.FLANGE_R * MM, 60.0, "ours, printed PETG"),
    ("knob across the flats (mm)", t.KNOB[1] * MM, 20.0, "ours; the jaw opens 94 mm, kit README"),
    ("rest, top (mm)", 2 * t.REST_HALF * MM, 50.0, "ours, a printed block"),
    ("finger force cap (N)", t.FINGER_N_MAX, 40.0, "Robocurve's figure for the YAM"),
]
bad = 0
print(f"{'quantity':36s}{'scene':>9s}{'published':>11s}  source")
for name, scene, pub, src in ROWS:
    ok = abs(scene - pub) <= 0.02 * pub + 0.3
    bad += not ok
    print(f"{name:36s}{scene:9.2f}{pub:11.2f}  {src}{'' if ok else '   <-- DIFFERS'}")
print("SIZES:", "FAIL" if bad else "PASS")
sys.exit(1 if bad else 0)
