# Demo — recorded forward walk

Reference data captured while the hexapod walked forward (`linear.y = -0.06`,
toward the face) under the tripod gait in Gazebo.

| File | Description |
|------|-------------|
| `forward_walk.png` | Performance plot: forward distance vs. time (Figure 2 in the main README). |
| `forward_walk_bag/` | `ros2 bag` of the run — `/joint_states`, `/cmd_vel`, `/tf`, `/leg_controller/controller_state`, `/clock`. |
| `walk_data.csv` | Sampled body Y-position vs. time used to make the plot. |
| `make_plot.py` | Regenerates `forward_walk.png` from `walk_data.csv`. |

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
# then, in another terminal, drive the robot (forward = -Y):
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {y: -0.06}}'
```
