# Demo — recorded walks and media tools

Reference data captured while the hexapod walked forward (toward the face) under
the tripod gait in Gazebo, plus the tool that records the README walking GIF.

| File | Description |
|------|-------------|
| `forward_walk.png` | Performance plot: forward distance vs. time (Figure 4 in the main README). |
| `forward_walk_bag/` | `ros2 bag` of the run — `/joint_states`, `/cmd_vel`, `/tf`, `/leg_controller/controller_state`, `/clock`. |
| `walk_data.csv` | Sampled body Y-position vs. time used to make the plot. |
| `make_plot.py` | Regenerates `forward_walk.png` from `walk_data.csv`. |
| `record_walk_gif.py` | Records `docs/media/hexapod_walking.gif` straight from Gazebo (Figure 2 in the main README). |

## Replay the bag

```bash
source install/setup.bash
ros2 bag play demo/forward_walk_bag
```

## Record your own

```bash
# with Gazebo + gait running:
ros2 bag record -o demo/my_walk \
  /clock /joint_states /cmd_vel /tf /tf_static /leg_controller/controller_state
# then, in another terminal, drive the robot forward:
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}'   # Ctrl+C to stop
```

## Record the walking GIF

With Gazebo, the gait and (for the look-around) the head controller running:

```bash
source install/setup.bash
python3 demo/record_walk_gif.py              # writes docs/media/hexapod_walking.gif
python3 demo/record_walk_gif.py --preview    # just save one still to check the framing
```

The script adds an invisible camera to the Gazebo world at a 3/4 front view of
wherever the robot is. It drives the robot through a fixed routine: stand, walk
forward, turn left, stop, then look left, right and up. It builds the GIF from the
camera frames and removes the camera again. Nothing is recorded from your desktop,
so the clip is steady and framed the same every time. Change the routine in
`SCRIPT`, or the framing with `--distance`, `--height` and `--side`.
