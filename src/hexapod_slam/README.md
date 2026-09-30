# hexapod_slam

RGB-D perception for the hexapod: **Stage 2.3 is visual odometry only**. No map,
no loop closure, no localisation, no Nav2 - those arrive in Stage 2.4 and later.

The robot is camera-based. There is no LiDAR and no depth-to-laserscan shortcut:
the only inputs to the odometry are the head camera's colour image, depth image
and intrinsics.

## What runs here

| Piece | What it is |
| --- | --- |
| `launch/visual_odometry.launch.py` | `rtabmap_odom/rgbd_odometry` on the head camera |
| `config/rgbd_odometry.yaml` | near-default parameters, deliberately untuned for characterisation |
| `run_odometry_experiment` | drives a loop, records a bag, scores the result against ground truth |
| `reset_run` | teleports the robot to START and zeroes the odometry between runs |

```
                /face_camera/image ─┐
             /face_camera/depth_image ├─► rgbd_odometry ─► /odom  (nav_msgs/Odometry)
            /face_camera/camera_info ─┘                  ─► /odom_info (features, inliers)
                                                         ─► TF odom → base_footprint
```

`rgbd_odometry` synchronises the three inputs with an **exact-time** policy, which
is correct here because Stage 2.2 measured identical stamps on colour and depth
(107/107 frames). It owns the `odom → base_footprint` transform - the one link
left unowned when Gazebo's ground-truth TF was unbridged in Stage 2.2.

## Ground truth is never an input

`/odom_ground_truth` comes from Gazebo's `OdometryPublisher` on the frame
`odom_ground_truth`, which is **not in the TF tree**. It is read in exactly one
place: `evaluate_odometry.py`, after a run, to score the trajectory. Nothing
initialises, corrects or constrains the odometry with it.

## Running an experiment

```bash
./run_facility.sh                                   # simulator + robot
ros2 launch hexapod_gait gait.launch.py             # walking
ros2 launch hexapod_slam visual_odometry.launch.py  # visual odometry

ros2 run hexapod_slam reset_run                     # back to START, odometry zeroed
ros2 run hexapod_slam run_odometry_experiment --loop ring --label runA_ring
```

Each run writes `verification/runs/stage2_vo_<label>_<timestamp>/`:

* `results.json` - ATE, final and worst position error, drift per metre, yaw
  error, tracking losses, stream rates, real-time factor, CPU and GPU
* `trajectory.png` - ground truth against visual odometry, and error vs distance
* `bag/` - rosbag2 (mcap) of the camera, odometry, TF and ground truth, so the
  same run can be replayed into a different configuration without the simulator

`--straight <seconds>` walks a straight line instead of a loop, which is the
quick sanity check; `--no-bag` skips the recording.

## Reading the numbers

*Drift per metre* is the final position error divided by the distance the robot
actually walked - the honest headline for open-loop odometry, since error grows
with distance travelled rather than with time. *ATE (RMSE)* is the spread of the
error over the whole run after both trajectories are expressed from the same
starting pose; no scale is fitted, because RGB-D odometry is metric.

Loop closure is what removes accumulated drift, and that is Stage 2.4's job, not
this stage's. A loop that ends metres from where it started is the expected shape
of the result here; the question Stage 2.3 answers is *how many* metres.
