# Verification

One script per [roadmap](../docs/ROADMAP.md) stage. Each one checks the running
simulation against written pass criteria and saves the measured values. Before starting
a new stage, run the previous stage's script again to confirm nothing has broken.

| Script | Stage | Needs running |
|--------|-------|---------------|
| `stage1_basic_gait.py` | 1 — Stable basic gait | `./run_gazebo.sh`, `ros2 launch hexapod_gait gait.launch.py` |

## Stage 1 — stable basic gait

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 verification/stage1_basic_gait.py          # about 2.5 min
python3 verification/stage1_basic_gait.py --bag    # also record the run with ros2 bag
```

The script sends `/cmd_vel` and measures the robot with **Gazebo ground truth** (body
pose and world foot heights), not with what the robot reports about itself. It checks:

- **Stand:** height, level (Gazebo and IMU), joint angles, all six feet on the ground,
  holds still.
- **Walking:**
  - forward, backward, turn left/right, strafe left/right, and an arc
  - direction, straightness, heading hold and body stability
  - tripod gait (only one tripod in the air), and a 50 Hz leg command stream
- **Stopping:** returns to the stand pose, comes to rest, all feet down.
- **`/cmd_vel` handling:** tiny commands are ignored, oversized commands are handled safely.
- **Overall:** no gait warnings or errors, and the same stand after every manoeuvre.
- **Informational:** what happens when the command publisher goes silent.

The robot ends the run standing still, a little away from where it started.

## Output

Each run writes a folder:

```text
verification/runs/stage1_basic_gait_<date>_<time>/
├── results.json    every check (measured vs. criterion, pass/fail) + per-phase metrics
└── bag/            (with --bag) MCAP recording of the run
```

The exit code is `0` when every required check passes. Bags are kept out of git because
they are large. Record important runs from Stage 2 onward.
