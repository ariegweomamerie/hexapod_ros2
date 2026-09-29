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
pose and world foot heights), not with what the robot reports about itself. Every
duration, rate and window is measured in **simulation time**, the same clock the gait
runs on, so results do not depend on how fast Gazebo runs here; the real-time factor is
reported with the results. It checks:

- **Stand:** height, level (Gazebo and IMU), joint angles, all six feet on the ground,
  holds still.
- **Walking:**
  - forward, backward, turn left/right, strafe left/right, and an arc
  - direction, straightness, heading hold and body stability
  - tripod gait (only one tripod in the air), continuous walking (no false stops),
    and a 50 Hz leg command stream
- **Starting:** no planted foot slides more than 5 mm.
- **Stopping:** after 8 stops at different gait phases, the robot is back in its stand
  pose within 1 s, comes to rest with all feet down, lifts only one tripod at a time,
  and never drags a planted foot more than 5 mm.
- **Command-loss watchdog:** when `/cmd_vel` goes silent without a zero command, the
  robot stops and stands within 1.5 s (0.5 s timeout + stop), without dragging.
- **`/cmd_vel` handling:** tiny commands are ignored, oversized commands are handled safely.
- **Overall:** no gait warnings or errors, and the same stand after every manoeuvre.

"Dragging" is measured from Gazebo: for every foot that stays on the ground, the
farthest it moves horizontally from where that contact began.

The gait's transition logic is also covered offline by unit tests
(`colcon test --packages-select hexapod_gait`). Those tests stop at every point of the
gait cycle, for six command types.

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
