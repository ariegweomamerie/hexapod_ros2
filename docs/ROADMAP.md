# Hexapod ROS 2 — Simulation Roadmap

This is the long-term plan for the project and the record of where it stands.
Work is **simulation only** (ROS 2 Jazzy + Gazebo Sim) until Stage 16 is complete.

## How we work

1. **One stage at a time.** Stages are never combined, and nothing from a later stage is
   built early (no face tracking, dancing or hardware work while on SLAM).
2. **Check the previous stage first.** Before starting a stage, re-run the previous
   stage's verification and confirm it still passes.
3. **Every stage ends with a test and a report:** run the verification, show the
   results, and explain what passed and what failed.
4. **Approval gate.** The next stage starts only after the results are approved.
5. **Keep what works.** Existing, verified behavior must stay stable. The gait is not
   redesigned for speed, and not modified unless a stage truly requires it.
6. **No hardware features** unless the simulation needs them (hardware is Stage 17).
7. **Record important test runs** with `ros2 bag` from Stage 2 (SLAM) onward. Recording
   must never hold up the stage itself.

Verification scripts live in [`verification/`](../verification/README.md), one per stage.

## Status

| Stage | Name | Status |
|------:|------|--------|
| 1 | Stable basic gait | 🔶 **In verification** — 56/57 checks pass; 1 open issue (see Stage 1) |
| 2 | SLAM | ⬜ Not started |
| 3 | Localization | ⬜ Not started |
| 4 | Nav2 | ⬜ Not started |
| 5 | Navigation tuning | ⬜ Not started |
| 6 | Head controller | 🔷 Built early (see Stage 6) — frozen until its turn |
| 7 | Simulated camera perception | ⬜ Not started (camera sensor exists, no processing) |
| 8 | Face detection and tracking | ⬜ Not started |
| 9 | Object detection | ⬜ Not started |
| 10 | Search / look-around behavior | ⬜ Not started |
| 11 | Person following | ⬜ Not started |
| 12 | Behavior system | ⬜ Not started |
| 13 | Dance behavior | ⬜ Not started |
| 14 | Interaction | ⬜ Not started |
| 15 | Behavior manager | ⬜ Not started |
| 16 | Full simulation integration | ⬜ Not started |
| 17 | Hardware preparation | ⬜ Not started — only after Stage 16 |

Legend: ✅ verified and approved · 🔶 in progress / in verification · 🔷 built ahead of plan,
not yet verified at its stage · ⬜ not started

**Immediate priority:** stable gait → SLAM → localization → Nav2.

---

## Already in place before this roadmap

- **Robot model:** 20 joints (6 legs × 3, plus pan/tilt face), foot pads, mirror-symmetric
  stance, face straight by default.
- **Control:** `ros2_control` in Gazebo, with `leg_controller` and `face_controller`
  (JointTrajectoryController).
- **Gait:** `hexapod_gait` (tripod gait with per-leg IK), driven by `/cmd_vel`
  (`linear.x` forward, `linear.y` left, `angular.z` turn).
- **Sensors in simulation:** IMU (`/imu`), face camera (`/face_camera/image`), Gazebo
  odometry (`/odom`, `odom → base_footprint` TF), and gait-phase foot contacts
  (`/foot_contacts`). There is no lidar.
- **Head:** `hexapod_head` (`/head/cmd`), which belongs to Stage 6. See that stage.
- **Media:** walking demo GIF (`demo/record_walk_gif.py`).

---

## Stage 1 — Stable basic gait

**Goal:** the existing hexapod stands correctly, walks forward and backward, turns, and
responds correctly to `/cmd_vel`. Keep the current gait. Do not redesign it for speed.

**Verification:** `python3 verification/stage1_basic_gait.py` (about 2.5 min, needs Gazebo
and the gait running). Robot motion is measured from Gazebo ground truth, not from the
robot's own reports.

**Run 2026-09-17 — 56/57 required checks passed:**

| Area | Result | Measured |
|------|--------|----------|
| Stand (initial and final) | ✅ | 143.0 mm high, 0.00° tilt, joints within 0.02°, all 6 feet down, 0 drift |
| Walk forward (`x = +0.10`) | ✅ | +27.7 cm in 10 s (2.77 cm/s), 0.8 cm sideways, −0.5° heading change |
| Walk backward (`x = −0.10`) | ✅ | −27.4 cm in 10 s, 1.2 cm sideways, −1.1° heading change |
| Turn left / right (`z = ±0.4`) | ✅ | +60.7° / −59.2° in 10 s (about 6°/s), body centre moved ≤ 1.1 cm |
| Strafe left / right (`y = ±0.10`) | ✅ | +23.5 / −20.0 cm in 8 s, yaw change within ±0.6° |
| Arc (`x = 0.08, z = 0.3`) | ✅ | +15.7 cm forward, +37.5° |
| Tripod gait | ✅ | Every sample: only one tripod in the air (Gazebo foot heights) |
| Body stability while walking | ✅ | Tilt ≤ 0.1°, height 132–144 mm |
| Leg command stream | ✅ | 50.0 Hz steady |
| Deadband (`x = 0.005`) | ✅ | No steps, 0 drift |
| Oversized command (`x = 1, z = 1`) | ✅ | No warnings, stable, still moves |
| Gait warnings/errors | ✅ | 0 |
| **Stop → stand transition** | ❌ | **1.20–1.30 s to settle (criterion ≤ 1.0 s), 8/8 stops** |

**Open issue: the stop transition drags the feet.** On a zero command the gait jumps
straight from the walking pose (feet 120 mm from each hip) to the standing pose
(150 mm). The planted feet are dragged 14–40 mm across the ground, and the lifted
tripod is pushed straight down. Friction slows the joints, so they settle in about
1.2 s, and the body shifts about 18 mm and 1.5° on every stop. Starting to walk
presumably does the same in reverse, but that transition was not measured. It
matters for Stage 4, because Nav2 starts and stops often near a goal. Decision
pending:

- **Option A:** accept the transition as it is and relax the criterion.
- **Option B (recommended):** move the feet between the two footprints only while
  they are lifted ("step into stand / step out of stand"), then re-verify.

**Findings to carry forward (not Stage 1 failures):**

- **No `/cmd_vel` timeout.** If the command publisher stops without sending zero, the
  robot keeps walking on the last command. This is a safety risk once Nav2 drives the
  robot. Adding a timeout changes how `ros2 topic pub` (1 Hz) and
  `teleop_twist_keyboard` behave, so it needs a decision.
- **Actual speed is about 28% of the command** (forward 0.10 m/s gives 2.77 cm/s;
  turning 0.4 rad/s gives about 6°/s). Nav2 velocity limits and controller tuning must
  account for this in Stage 4. It is not a Stage 1 goal.
- **The gait's stride limit.** Stride saturates at about 0.2 m/s of translation or about
  0.7 rad/s of rotation. Beyond that, legs are clamped individually, and a combined
  command loses most of its turn: `x = 1, z = 1` turned only 6° in 5 s. Nav2 limits
  must stay inside this range.
- **`/odom` is perfect in simulation.** It comes from Gazebo's OdometryPublisher, which
  is ground-truth based and has no slip error, while the real robot's odometry will
  drift. Keep this in mind when judging SLAM and localization.

## Stage 2 — SLAM

**Goal:** build a map of a simulated environment while the hexapod walks.

- **Sensing is an open decision.** Choose a sensing setup that fits a camera-based
  hexapod. The old LaserScan display came from a wheeled-robot template and must not be
  restored blindly. Decide at the start of this stage and document the reasoning.
- Build a simulated environment with enough structure to map.
- Verify that the robot can move while mapping, and check the resulting map before
  moving on.
- Record the mapping runs with `ros2 bag`.

## Stage 3 — Localization

**Goal:** localize against the map from Stage 2.

- Verify the estimated pose against Gazebo ground truth.
- Verify TF relationships (`map → odom → base_footprint`).
- Localization stays stable while the hexapod walks and turns.

## Stage 4 — Nav2

**Goal:** autonomous point-to-point navigation through the existing gait.

```text
Nav2 → /cmd_vel → hexapod_gait → leg_controller → hexapod legs
```

- Connect Nav2's velocity output to the existing `/cmd_vel` interface.
- Send navigation goals, and verify planning, movement, turning and stopping.
- Do not modify the gait unless it is truly necessary.

## Stage 5 — Navigation tuning

Tune path following, turning and stopping. Test obstacle avoidance and recovery
behaviors, and validate in several simulated environments.

## Stage 6 — Head controller

A dedicated `hexapod_head` package provides `/head/cmd` (pan + tilt). It clamps commands
to safe servo limits, smooths the motion, and sends commands to `face_controller`. Head
control is never placed inside the gait package.

> **Built ahead of plan (2026-09-17).** `hexapod_head` already exists and meets this
> description: limits come from the URDF, speed-limited smooth moves, 11 unit tests, and
> it was checked live in Gazebo. It is frozen (no new features) until this stage, when
> it will be re-verified.

## Stage 7 — Simulated camera perception

Use the Gazebo face camera, process its stream, and establish the vision pipeline. Keep
camera processing separate from gait and navigation.

## Stage 8 — Face detection and tracking

```text
Simulated camera → AI face detection → face position → /head/cmd → simulated pan/tilt → camera follows face
```

Detect a face, find its position in the image, and track it with pan/tilt so it stays
near the image centre.

## Stage 9 — Object detection

Detect objects in the simulated environment, identify the target, estimate its image
position, and turn the head toward it.

## Stage 10 — Search / look-around behavior

Look around when no target is detected: scan left and right, stop scanning when a target
is found, then track it.

## Stage 11 — Person following

```text
Camera → vision AI → person position ─┬→ /head/cmd
                                      └→ navigation decision → /cmd_vel → hexapod_gait
```

Combine vision with navigation: detect a person, determine their relative direction,
track them with the head, then follow them with `/cmd_vel`.

## Stage 12 — Behavior system

A separate behavior layer (idle, look around, wave, bow, salute, wake, sleep, react to
objects/persons), kept separate from the low-level gait controller.

## Stage 13 — Dance behavior

A coordinated routine of legs, body and head: stand → body sway → left → right → turn →
head movement → return to stand. It is implemented as a behavior, not built into the gait.

## Stage 14 — Interaction

Commands that trigger behaviors, for example `/dance`, `/wave`, `/bow`, `/look_around`,
`/sleep` and `/wake`.

## Stage 15 — Behavior manager

A higher-level manager that coordinates navigation, vision, head movement and behaviors.

## Stage 16 — Full simulation integration

The simulated robot walks, maps, localizes, navigates autonomously, sees, tracks faces,
detects objects, moves its head, searches for targets, follows a person, performs
behaviors and dances.

## Stage 17 — Hardware preparation

**Only after the simulation is validated:**

- Prepare the Raspberry Pi integration.
- Prepare the servos and the PWM controller.
- Prepare the BNO055 IMU and the physical camera.
- Map the simulated interfaces to hardware interfaces, and test the hardware step by step.
