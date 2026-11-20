# Hold a WidowX 250 arm up as its power is cut for a repair, and lay it limp in its sleep pose without dropping it

A Trossen WidowX 250 6DOF (the arm of BridgeData V2 and ALOHA's leader arm) stands on the bench in its working pose, held by its motors. The robot closes one gripper on the arm's upper arm and the other on its gripper body; one second after both have closed, the arm's torque is cut and its six joints go limp. The robot carries the limp arm, folds it back over its base into Interbotix's sleep pose, and lets go.

**Ability tested:** carrying a limp, gravity-loaded six-joint chain at two points through a power cut, and folding it to a joint-space pose without a hard landing. The shoulder and elbow fold under the arm's own weight as soon as the torque goes, so both holds must already carry it, and the two hands must move together along the fold.

**Why it matters:** robot arms are repaired with their power off, and a powered-down arm cannot hold itself up. Trossen warns that terminating control of its WidowX and ViperX arms "while the arms are not in their sleep configurations will cause them to collapse!", and for ALOHA that the arms "WILL still torque off and drop", so they must be held. The WidowX 250 is the arm BridgeData V2 was collected with and ALOHA's leader arm, so arms that produce robot-learning data are among those serviced. Limp, it is a six-joint chain of 1.58 kg that passes 0.66 m/s if held at the gripper alone.

**Hands:** both are needed. Held only at its gripper, the arm's shoulder and elbow fold and a link passes 0.66 m/s, which voids the run (the `onehand` run below). The left hand holds the upper arm's 28 mm tube 6 cm above the shoulder axis (16.9 cm above the bench in the working pose) and carries the shoulder's load; the right holds the gripper body (also 16.9 cm up) and sets the elbow and wrist.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

**The WidowX model:** `base.stl`, `shoulder.stl`, `upper_arm.stl`, `upper_forearm.stl`, `lower_forearm.stl`, `wrist.stl`, `gripper.stl`, `gripper_prop.stl`, `gripper_bar.stl` and `gripper_finger.stl` are Trossen Robotics' WidowX 250 6DOF meshes from `interbotix_ros_manipulators` (`interbotix_xsarm_descriptions/meshes/wx250s_meshes`, humble branch), and the joint axes, origins, limits, link masses, centres of mass and inertias in `task.py` are from its `urdf/wx250s.urdf.xacro`. They are used under Trossen's BSD 3-Clause licence (Copyright 2023 Trossen Robotics), in `TROSSEN_LICENSE.txt`. The sleep pose is `sleep_positions` in `interbotix_xsarm_control/config/wx250s.yaml`.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
  --instruction "This robot arm is held up by its motors. Close both grippers on it: one second later its power is cut and it goes limp. Lay it down in its sleep pose (shoulder -1.80, elbow 1.55, wrist angle 0.8 rad, the other joints 0) without dropping it, then let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                  # full score, with a video
python demokit.py task.py OUT --no-record --mode none,onehand,wrong,gate     # the lazy routes and the unheld fall
```

The demo is a scripted reference. It reads the WidowX's joint angles (its encoders report with torque off), which a policy would see through the cameras. Each hand comes from the YAMs' side, level, fingers along the arm's pitch axes: the left closes across the upper arm's 28 mm tube 6 cm above the shoulder axis, the right across the gripper body. After the cut it folds the arm to the sleep pose in 30 steps of 0.3 s, each hand turning with its link; each step it reads the joints and moves each hand's goal by half of the gap between where its link is and where it should be. The forearm comes to rest on the upper arm and the gripper on its foam. The right hand then opens where it is, draws 13.5 cm straight back along its fingers and returns to its ready pose; then the left, which opens only to 4 cm so its upper finger does not lift the forearm.

## Building it for a real trial

- **Arm:** the trial lab's own WidowX 250 6DOF (Trossen lists it at several thousand US dollars; price unverified), its base bolted to the bench at the stated position, on its own power supply and U2D2.
- **Power cut:** a script calls the Interbotix SDK's torque-enable service to disable torque on all joints when the operator presses a button once both YAM grippers have closed on the arm, and logs `/joint_states` at 20 Hz throughout. The bus stays healthy, so the encoders keep reporting; the "failed servo" of the instruction is simulated.
- **Foam blocks:** `rest_a` is 12 x 7 cm, 4.3 cm high, at (28, 34.6). `rest_b` is seven 1 cm strips across y, each 12 cm wide in x at x = 28, centred at y = -4, -3, -2, -1, 0, 1 and 2 cm (height y: 8.6, 6.7, 6.7, 7.3, 8.2, 8.6, 9.1 cm), each cut to the gripper's underside; all are listed at load in `task.py`.
- **Pad and tether:** a 2 cm foam pad (polyurethane, any firm grade) where an unheld arm lands; and a tether strap from an overhead bar, 50 cm above the bench over the base's centre (28, 10), to the upper arm's tube 6 cm above the shoulder axis (where the left hand holds), 45 cm long. The attachment is 33 cm from the bar in the working pose and at most 41 cm along the fold to the sleep pose, so the strap stays slack through the whole fold (at least 4 cm of slack) and only limits how far a loose arm swings toward the YAMs. The strap is not in the simulator.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real arm |
|---|---|---|
| Static friction of each unpowered joint | 0.15 N m per XM430-W350 (two on the shoulder), 0.06 N m on the XL430-W250 wrist rotate; dynamic 70 % of it | **Estimate.** Torque off, hang a spring scale on a link at a known radius and read the pull at which the joint starts to turn |
| Viscous damping | 0.05 N m s/rad per servo | **Estimate.** Let a link fall a few degrees and fit the speed from `/joint_states` |
| Rotor inertia seen at the joint | 0.006 kg m² per XM430, 0.003 per XL430 | **Estimate:** a rotor of about 0.5 g cm² times the gear ratio squared (353.5:1, 258.5:1, Robotis) |
| Arm mass | 2.12 kg with the base, 1.58 kg moving (Trossen's URDF) | Kitchen scale on the arm off its base |

## Scoring

See `scoring` in `task.json` and `Trial` in `task.py`. In a real trial, from the logged `/joint_states` (link speeds from them through the arm's kinematics) and the video.

- **1/3, caught:** when the torque is cut, both grippers are closed on the arm (each finger within 2.5 cm of its surface, the jaws under 8 cm apart).
- **2/3, laid:** also every joint within 10 degrees of the sleep pose at the last moment both grippers held it, and now, the arm still (no point faster than 1 cm/s) and resting on its foam. A joint that sags home under gravity after a hand opens does not count.
- **1.0, let go:** then both whole grippers (a lattice 2 cm apart over the fingers, pads, palm, housing and 12 cm of the wrist) at least 5 cm from the arm's surface, the arm still laid, for 2 s.
- **A drop voids the run (0):** after the cut, any point of the arm faster than 0.5 m/s, or a link striking the bench, pad or foam faster than 0.25 m/s. Unheld, the arm passes 0.77 m/s within 0.05 s of the cut (`none`); the demo's fold peaks at 0.16 m/s.

The speeds are of the arm's surface points (from Trossen's meshes) computed from the joint readings through the URDF's kinematics, so a few degrees of drift in a light wrist joint is not a fall, and a dropped arm is.

## Results

| Run | What it does | Score |
|---|---|---|
| `none` | The arms wait; at 30 s the torque is cut and the unheld arm passes 0.77 m/s at once | 0.0 |
| `onehand` | Both hands hold through the cut; the left lets go and the right lays the arm alone by its gripper: the shoulder and elbow fold, a link at 0.66 m/s | 0.0 |
| `wrong` | Laid with the wrist angle 30 degrees short of its sleep angle; after the hands open the gripper sags onto its foam to 9 degrees off, but the hands never set it | 0.33 |
| `full` | Both hands carry the arm through the cut, fold it to the sleep pose, let go one at a time and return to their ready poses (8.9 cm clear) | 1.0 |
| Dream-1 (the model's run, 1,800 steps) | Never closes both grippers on the arm; at 30 s the torque is cut unheld and the arm drops (a link at 0.77 m/s), a valid session with no expiry | 0.0 |
