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
| 1 | Stable basic gait | ✅ **Passed** 2026-09-29 — 75/75 checks, twice (see Stage 1) |
| 2 | SLAM | 🔶 **In progress** — sensing decided (RGB-D + RTAB-Map); test facility built and validated |
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

**Goal:** the existing hexapod stands correctly, walks forward and backward, turns,
strafes and follows curves, and responds correctly to `/cmd_vel`. It must stop cleanly,
without dragging its feet, and stop by itself if commands are lost. Keep the current
gait. Do not redesign it for speed.

**Closed 2026-09-29: 75/75 required checks, in two consecutive runs.**

**Verification:**
- `python3 verification/stage1_basic_gait.py` — 75 checks in Gazebo against ground truth.
- `colcon test --packages-select hexapod_gait hexapod_head` — 23 offline unit tests.

### Final results (two runs, same setup)

| Measurement | Limit | Run A | Run B |
|---|---|---|---|
| Required checks | all | **75/75** | **75/75** |
| Worst planted-foot slide when stopping | ≤ 5.0 mm | 2.8 mm | 2.7 mm |
| Worst planted-foot slide when starting | ≤ 5.0 mm | 0.6 mm | 0.6 mm |
| Slowest return to the stand pose | ≤ 1.0 s | 0.83 s | 0.82 s |
| Standing again after `/cmd_vel` goes silent | ≤ 1.5 s | 1.32 s | 1.32 s |
| Gazebo real-time factor during the run | reported | 0.93 | 0.93 |

Walking (run B): forward 84.0 cm and backward 85.9 cm in ~10 s (about 8.4 cm/s),
turning ±19°/s, strafing ±72 cm in ~8 s, body tilt ≤ 0.1°, one tripod lifted at a
time in 100% of samples, 50 Hz leg command stream, and no gait warnings or errors.

### What changed in this stage

1. **Lifted-foot start/stop transitions.** The gait is a small state machine
   (`STANDING → STARTING → WALKING → STOPPING`). Feet move between the walking and
   standing footprints **only while lifted**, one tripod at a time, with a short
   settle pause before the other tripod lifts. Steady walking is untouched, and a
   unit test proves the stride matches the original formula exactly.
2. **`/cmd_vel` watchdog (0.5 s).** If commands stop arriving, the gait stops the
   robot through that same safe transition. README examples publish continuously.
3. **Simulation time.** The gait's control timer, phase, transitions and watchdog run
   on Gazebo's `/clock`, as do the verification's measurements. Before this, the gait
   ran on wall-clock time and stepped too fast for the physics whenever Gazebo fell
   behind real time, which made results depend on machine load.
4. **Leg controller: `interpolate_from_desired_state`.** Each streamed trajectory used
   to restart interpolation from the lagging *measured* state, which delayed commands
   150–180 ms and executed only 27–39% of the planned joint motion.
5. **Faster simulated servos with soft limits.** `position_proportional_gain` 0.1 → 0.3
   (~0.1 s → ~33 ms response). At 0.3 a gravity-loaded joint driven exactly onto a hard
   stop locks up in the physics engine, so the head node keeps commands 0.02 rad inside
   the URDF limits (verified: 5 full-range cycles with no sticking, versus jamming on
   the first attempt without the margin).
6. **Better verification.** Checks for planted-foot dragging (start and stop),
   continuous walking, and the watchdog; stops now sample different gait phases; the
   real-time factor is recorded; phase windows are half-open.

Items 4 and 5 fixed the root cause: the legs now execute ~95% of the planned motion
instead of about a third, so they are no longer still catching up when a foot takes
load. As a side effect, walking is about 3× faster with the gait itself unchanged
(2.8 → 8.4 cm/s at the same command).

### How it got there

| Attempt | Result |
|---|---|
| Original gait | 56/57 — stop took 1.2–1.3 s, feet dragged 14–40 mm |
| New drag/watchdog checks on the original gait | 64/75 — baseline proving the checks catch it |
| Lifted-foot transitions + watchdog | 73/75 — starting clean, stops still slid 6–9 mm |
| \+ controller interpolation fix | 75/75, but only 0.2 mm of margin |
| \+ simulation time, pause tuned to 0.075 s | 74/75 and 75/75 — repeatable, but on the 5 mm limit |
| Gentler step profile ("Option 1") | Rejected: worst case 5.6–8.2 mm. Measurement showed the slide comes from lagging servos catching up on a planted foot, which a smoother path cannot remove |
| Faster servos + soft limits ("Option 2") | **75/75 twice, worst slide 2.7–2.8 mm** |

### Findings to carry forward

- **Stride limit.** The stride saturates at about 0.2 m/s of translation or 0.7 rad/s of
  rotation, each leg clamped separately. Nav2 limits must stay inside this range.
- **`/odom` is perfect in simulation.** It comes from Gazebo's OdometryPublisher and has
  no slip error, while a real robot's odometry drifts. Keep this in mind when judging
  SLAM and localization.
- **Leg joints still use hard limits.** A 0.02 rad margin in the leg IK would clip the
  walking swing of `leg_l3`, so only the head has soft limits. The legs showed no
  sticking in four full runs at the faster gain; revisit if a leg ever freezes.
- **Test environment.** Results are reported with Gazebo's real-time factor. A GNOME
  screen recording (~1100% CPU) once dropped it to 0.6 and produced two misleading
  failures. Use `demo/record_walk_gif.py` to record the robot instead.

## Stage 2 — SLAM

**Goal:** build a map of a simulated environment while the hexapod walks, with
loop closure, and verify the map.

### Sensing decision (2026-09-29)

**RGB-D camera → RTAB-Map** (`rtabmap_odom` for visual odometry, `rtabmap_slam` for
the graph, loop closure and map). No lidar and no depth-to-laserscan: the simulation
mirrors the sensing the real robot will use.

Why RTAB-Map over the alternatives on this stack (Ubuntu 24.04 / ROS 2 Jazzy):

| Option | Jazzy packages | Loop closure | Map for Nav2 | Verdict |
|--------|----------------|--------------|--------------|---------|
| **RTAB-Map** 0.23.7 | apt binaries | yes, appearance-based | occupancy grid + `map → odom` | **chosen** |
| ORB-SLAM3 | none (unmaintained wrappers) | yes | sparse points only | rejected |
| Isaac ROS Visual SLAM | Humble/Jetson focus | yes | no grid | rejected |
| OpenVINS / VINS-Fusion | partial ports | VINS only | no | rejected: odometry only |
| slam_toolbox | apt binaries | yes | yes | excluded: needs a LaserScan |

Ground-truth odometry is **not** used to help SLAM. Gazebo's `/odom` is kept for
scoring only; visual odometry will own `odom → base_footprint`.

### Test facility (built and validated 2026-09-29)

`hexapod_worlds` holds a bespoke **industrial robotics facility**, 15 × 11 m, generated
from primitives and procedural textures (`generate.py`) so it loads with no downloads
and stays reproducible. Eight perimeter rooms — robotics lab, loading, testing, storage,
workshop, maintenance, parts store, equipment room — around a ring corridor with an
equipment core inside the loop.

Designed for this robot and this sensor:

- **Three nested loops** for loop-closure tests: ring 25.6 m, core 19.5 m, rooms 12.0 m.
  At ~9 cm/s a short loop is walkable in minutes, so it gets tested often.
- **START sits on the ring, inside a loop**, not at the end of a corridor.
- **Alternative routes**: every room opens onto the ring, several open into each other,
  giving T-junctions and four-way choices for Nav2 later.
- **Detail placed low.** The camera is 0.14 m off the floor, so the world puts its
  features there: floor lane markings, kick plates, door numbers, pallets, pipe runs.
- **Zone identity**: each area has its own wall and floor materials; the two corridor
  legs are deliberately similar, to test perceptual aliasing honestly.

### Validation results

| Check | Result |
|---|---|
| World loads, no missing resources | ✅ 3 controllers active, no asset errors |
| Props vs walls / each other / START | ✅ 0 overlaps (`validate_slam_world`) |
| Every zone reachable from START | ✅ 10/10 zones, 53.8 m² walkable |
| Loops walkable with a lane | ✅ ring 1.20 m, core 0.90 m, rooms 0.90 m narrowest |
| Doorways | ✅ widened to 1.1 m (1.0 m clear of frames) for a legged robot |
| Visual features from robot height | ✅ 18/18 viewpoints, 343–1476 ORB features (min 150) |
| Places distinguishable | ✅ no pair above 15% descriptor match |
| Robot walks in the facility | ✅ 9.9 cm/s straight down the corridor |
| Robot walks a full loop | ✅ ring loop 24.6 m in 4.8 min, back to START, body tilt ≤ 0.4° |
| Robot passes through doorways | ✅ core loop: in the south door, out the north, tilt ≤ 2.7° |
| Real-time factor, world alone | ✅ 0.99 (empty world: 1.00) — the world itself is nearly free |
| Real-time factor, with robot + camera | ⚠️ 0.54–0.62; the robot's 30 Hz camera rendering the scene is the cost |

Offline unit tests (`colcon test --packages-select hexapod_worlds`, 8 tests) hold the
layout to its invariants: zones tile the floor, doorways sit inside their walls and stay
at least 1.0 m wide, geometry stays inside the building, loops close, START is on the
ring but outside the core, and every zone has a camera viewpoint.

Two bugs the validation caught, both fixed: a ground plane under the floor slabs made
the feet chatter between coincident surfaces (the robot stepped but barely moved,
1.5 cm in 25 s), and the world was first built with ~350 one-box links, which halved
the real-time factor before the geometry was merged into a few links.

**Next in this stage:** swap the RGB camera for an RGB-D sensor (15 Hz, ~87° FOV,
optical frames), bridge depth and points, bring up `rgbd_odometry` and measure it
against ground truth, then add `rtabmap`, map a loop, confirm loop closure, and check
the map against the world's true geometry. `ros2 bag` records each run.

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
