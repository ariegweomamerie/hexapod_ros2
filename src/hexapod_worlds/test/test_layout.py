"""Layout invariants for the facility (no simulator, no ROS).

These are the mistakes that are easy to make when moving a room or a prop, and
expensive to find in Gazebo: geometry outside the building, zones that no longer
tile the floor, doorways that fall off the end of their wall, loops that do not
close. The heavier checks - overlaps, reachability, walking lanes - live in
hexapod_worlds/validate.py, which rasterises the whole layout.
"""
import pytest

from hexapod_worlds import layout as L

W, H = L.OUTER


def test_zones_tile_the_interior_without_gaps_or_overlaps():
    """Every floor slab is inside the building and together they cover it once."""
    area = 0.0
    rects = []
    for name, (x0, y0, x1, y1), floor, wall, label in L.zone_rects():
        assert 0 <= x0 < x1 <= W and 0 <= y0 < y1 <= H, f"{name} leaves the building"
        area += (x1 - x0) * (y1 - y0)
        for other in rects:
            ox0, oy0, ox1, oy1 = other
            overlap = (min(x1, ox1) - max(x0, ox0)) * (min(y1, oy1) - max(y0, oy0))
            assert not (x0 < ox1 and ox0 < x1 and y0 < oy1 and oy0 < y1), \
                f"{name} overlaps another zone by {overlap:.2f} m2"
        rects.append((x0, y0, x1, y1))
    interior = (W - 2 * L.WALL_T) * (H - 2 * L.WALL_T)
    assert area == pytest.approx(interior, rel=0.01), \
        f"zones cover {area:.1f} m2 of a {interior:.1f} m2 interior"


def test_doorways_sit_inside_their_wall():
    """A gap (plus the widening) must fall within the wall it is cut from."""
    for x0, y0, x1, y1, gaps in L.WALLS:
        vertical = abs(x1 - x0) < 1e-9
        start, end = (y0, y1) if vertical else (x0, x1)
        last = start
        for g0, g1 in sorted(gaps):
            g0, g1 = g0 - L.DOOR_EXTRA, g1 + L.DOOR_EXTRA
            assert start < g0 < g1 < end, f"doorway {g0:.2f}-{g1:.2f} falls off its wall"
            assert g0 > last, "doorways overlap"
            assert g1 - g0 >= 1.0, f"doorway only {g1 - g0:.2f} m wide"
            last = g1


def test_walls_and_props_are_inside_the_building():
    for cx, cy, length, vertical in L.wall_boxes():
        assert 0 <= cx <= W and 0 <= cy <= H, f"wall at ({cx}, {cy}) is outside"
        assert length > 0
    for model, x, y, yaw in L.PROPS:
        assert L.WALL_T < x < W - L.WALL_T and L.WALL_T < y < H - L.WALL_T, \
            f"{model} at ({x}, {y}) is outside the building"
    for tex, x, y, yaw, z in L.SIGNS:
        assert 0 <= x <= W and 0 <= y <= H and 0.2 < z < L.WALL_H


@pytest.mark.parametrize("name,pts", [("ring", L.loop_ring()), ("core", L.loop_core()),
                                      ("rooms", L.loop_rooms())])
def test_loops_are_closed_and_inside(name, pts):
    assert pts[0] == pts[-1], f"{name} loop does not return to its start"
    assert len(pts) >= 4, f"{name} loop is not a loop"
    for x, y in pts:
        assert L.WALL_T < x < W - L.WALL_T and L.WALL_T < y < H - L.WALL_T


def test_start_is_on_the_ring_inside_the_loop():
    """The robot starts on the corridor, which is what makes the first loop closure
    reachable without leaving a dead end first."""
    x, y = L.START["x"], L.START["y"]
    rx0, ry0, rx1, ry1 = L.R_OUT
    ix0, iy0, ix1, iy1 = L.R_IN
    assert rx0 < x < rx1 and ry0 < y < ry1, "START is not in the ring corridor band"
    assert not (ix0 < x < ix1 and iy0 < y < iy1), "START is inside the equipment core"


def test_every_zone_has_a_viewpoint():
    """inspect_facility_views should look at every part of the world."""
    zones = {name for name, *_ in L.zone_rects()}
    seen = set()
    for label, x, y, yaw in L.VIEWPOINTS:
        for name, (x0, y0, x1, y1), *_ in L.zone_rects():
            if x0 <= x <= x1 and y0 <= y <= y1:
                seen.add(name)
    assert zones == seen, f"no camera viewpoint in: {sorted(zones - seen)}"
