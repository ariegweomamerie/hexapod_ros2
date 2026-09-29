#!/usr/bin/env python3
"""Geometric validation of the facility layout - no simulator needed.

Rasterises walls and props into an occupancy grid, inflates it by the hexapod's
radius, and checks the things that actually stop a walking robot:

  * props that overlap walls, other props, or the robot's start pose
  * doorways and corridors wide enough for the robot to pass and turn
  * every room reachable from START (flood fill, so a blocked door is caught)
  * each designed loop actually walkable, with the clearance along it

    ros2 run hexapod_worlds validate_slam_world

Writes docs/facility_clearance.png: reachable space, obstacles and the loops.
"""
import math
import os
import sys

import numpy as np
from PIL import Image

from hexapod_worlds import layout as L

RES = 0.05                      # grid resolution (m)
ROBOT_RADIUS = 0.28             # hexapod half-width incl. legs (0.53 m wide + margin)
MIN_LANE = 0.90                 # m, a lane the robot can walk and turn in

# Footprints of the prop models (x, y) in metres, from generate.prop_models().
FOOTPRINT = {
    "pillar": (0.42, 0.42), "crate": (0.50, 0.50), "pallet": (1.20, 0.80),
    "shelf_unit": (1.00, 0.45), "barrel": (0.50, 0.50), "workbench": (1.40, 0.70),
    "machine_cnc": (1.30, 1.00), "machine_press": (1.00, 1.00), "control_panel": (0.80, 0.40),
    "pipe_run": (2.20, 0.20), "cable_reel": (1.04, 0.62), "toolbox": (0.62, 0.34),
    "locker": (0.40, 0.50), "robot_cell": (1.60, 1.60),
}


def grid_shape():
    return int(round(L.OUTER[1] / RES)), int(round(L.OUTER[0] / RES))


def _rect_mask(cx, cy, sx, sy, yaw):
    """Mask of a rotated rectangle on the world grid."""
    rows, cols = grid_shape()
    ys, xs = np.mgrid[0:rows, 0:cols]
    wx = (xs + 0.5) * RES - cx
    wy = (ys + 0.5) * RES - cy
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = wx * c - wy * s, wx * s + wy * c
    return (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)


def build_occupancy():
    rows, cols = grid_shape()
    walls = np.zeros((rows, cols), bool)
    for cx, cy, length, vertical in L.wall_boxes():
        sx, sy = (L.WALL_T, length) if vertical else (length, L.WALL_T)
        walls |= _rect_mask(cx, cy, sx, sy, 0.0)
    for fixed, edge, vertical in L.door_frames():
        cx, cy = (fixed, edge) if vertical else (edge, fixed)
        sx, sy = (0.10, L.WALL_T + 0.04) if vertical else (L.WALL_T + 0.04, 0.10)
        walls |= _rect_mask(cx, cy, sx, sy, 0.0)
    props = {}
    for i, (model, x, y, yaw) in enumerate(L.PROPS):
        sx, sy = FOOTPRINT[model]
        props[(i, model, x, y)] = _rect_mask(x, y, sx, sy, yaw)
    return walls, props


def clearance_map(blocked):
    """Distance (m) from every free cell to the nearest blocked cell."""
    import cv2
    free = (~blocked).astype(np.uint8)
    return cv2.distanceTransform(free, cv2.DIST_L2, 5) * RES


def flood(free, start_xy):
    import cv2
    rows, cols = free.shape
    mask = np.zeros((rows + 2, cols + 2), np.uint8)
    img = (free.astype(np.uint8) * 255)
    seed = (int(start_xy[0] / RES), int(start_xy[1] / RES))       # (x, y) = (col, row)
    if not free[seed[1], seed[0]]:
        return None
    cv2.floodFill(img, mask, seed, 128)
    return img == 128


def main():
    walls, props = build_occupancy()
    problems, notes = [], []

    # 1. props must not overlap walls or each other
    for key, mask in props.items():
        i, model, x, y = key
        if (mask & walls).any():
            problems.append(f"{model}#{i} at ({x:.2f}, {y:.2f}) overlaps a wall")
    items = list(props.items())
    for a in range(len(items)):
        for b in range(a + 1, len(items)):
            if (items[a][1] & items[b][1]).any():
                (ia, ma, xa, ya), (ib, mb, xb, yb) = items[a][0], items[b][0]
                problems.append(f"{ma}#{ia} at ({xa:.2f}, {ya:.2f}) overlaps "
                                f"{mb}#{ib} at ({xb:.2f}, {yb:.2f})")

    blocked = walls.copy()
    for mask in props.values():
        blocked |= mask
    clear = clearance_map(blocked)

    # 2. the robot must fit where it starts
    sx, sy = L.START["x"], L.START["y"]
    start_clear = clear[int(sy / RES), int(sx / RES)]
    if start_clear < ROBOT_RADIUS + 0.05:
        problems.append(f"START at ({sx}, {sy}) has only {start_clear * 100:.0f} cm clearance "
                        f"(needs > {(ROBOT_RADIUS + 0.05) * 100:.0f} cm)")

    # 3. reachability: flood fill the space the robot centre can occupy
    free = clear >= ROBOT_RADIUS
    reach = flood(free, (sx, sy))
    if reach is None:
        problems.append("START is inside an obstacle; cannot flood fill")
        reach = np.zeros_like(free)
    for name, (x0, y0, x1, y1), floor, wall, label in L.zone_rects():
        sub = reach[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)]
        area = sub.sum() * RES * RES
        if area < 0.5:
            problems.append(f"zone '{label}' is unreachable from START "
                            f"(only {area:.2f} m2 walkable)")
        else:
            notes.append(f"{label:16s} walkable {area:5.1f} m2")

    # 4. every designed loop must be walkable end to end, with lane clearance
    for label, pts in (("ring loop", L.loop_ring()), ("core loop", L.loop_core()),
                       ("rooms loop", L.loop_rooms())):
        worst, total, blocked_at = 9.9, 0.0, None
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            total += seg
            for t in np.arange(0, seg, RES):
                x, y = x0 + (x1 - x0) * t / seg, y0 + (y1 - y0) * t / seg
                c = clear[int(y / RES), int(x / RES)]
                if c < worst:
                    worst, blocked_at = c, (x, y)
        ok = worst >= MIN_LANE / 2
        (notes if ok else problems).append(
            f"{label:11s} {total:5.1f} m, narrowest {worst * 2:.2f} m"
            + ("" if ok else f" at ({blocked_at[0]:.2f}, {blocked_at[1]:.2f}) - needs "
                             f"{MIN_LANE:.2f} m"))

    # 5. picture: obstacles, reachable space and the loops
    rows, cols = grid_shape()
    img = np.full((rows, cols, 3), 245, np.uint8)
    img[reach] = (198, 228, 205)
    img[free & ~reach] = (235, 225, 205)
    img[blocked] = (70, 70, 78)
    img[walls] = (40, 40, 46)
    pic = Image.fromarray(np.flipud(img)).resize((cols * 3, rows * 3), Image.NEAREST)
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "facility_clearance.png")
    pic.save(path)

    print("layout checks")
    for n in notes:
        print(f"  ok   {n}")
    for p in problems:
        print(f"  FAIL {p}")
    print(f"\nwalkable area from START: {reach.sum() * RES * RES:.1f} m2 of "
          f"{L.OUTER[0] * L.OUTER[1]:.0f} m2 building")
    print(f"clearance picture: {os.path.relpath(path, os.getcwd())}")
    print(f"RESULT: {len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
