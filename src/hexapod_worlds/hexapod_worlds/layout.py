"""Layout of the hexapod's industrial robotics facility (plain data: no SDF, no ROS).

A medium-sized research/industrial building, 15 x 11 m, designed as a closed-loop
RGB-D SLAM benchmark rather than as decoration.

    +--------------------+-------------------+--------------------+
    |      STORAGE       |     WORKSHOP      |    MAINTENANCE     |
    +---door---+---------+------door---------+------door----------+
    |  P |  +=========== ring corridor ============+  |  E        |
    |  A |  |                                      |  |  Q        |
    |  R |  |   +---- EQUIPMENT CORE (machines) ---+  |  U        |
    |  T |  |   |      (doors north and south)     |  |  I  door  |
    |  S |door  +----------------------------------+ door P       |
    |    |  |                                      |  |  M        |
    +----+--+=======================================+--+-----------+
    |   ROBOTICS LAB     |   LOADING AREA    |     TESTING         |
    +--------------------+-------------------+--------------------+
                    START is on the ring, inside the loop

Why it is shaped this way:
  * The ring corridor is a true closed loop (~26 m) and the equipment core has a
    door at each end, so the core splits the ring into two shorter loops (~17 m).
    At the hexapod's ~8 cm/s, short loops matter: a loop closure you can reach
    in three minutes gets tested far more often than one that takes six.
  * START sits on the ring itself, inside a loop, not at the end of a corridor.
  * Every perimeter room opens onto the ring, and several open into each other
    (lab<->loading, workshop<->maintenance, parts<->lab, equipment<->testing),
    giving alternative routes, T-junctions and four-way choices for Nav2 later.
  * Each zone has its own wall and floor materials, so places look different -
    except the two long corridor legs, which are deliberately similar to test
    perceptual aliasing honestly.
  * The camera sits 0.14 m above the floor, so detail lives low: floor lane
    markings, kick plates, pallets, pipe runs, cable reels and door numbers.

Dimensions in metres, x right, y up, origin at the south-west outer corner.
"""

WALL_T = 0.15
WALL_H = 2.8                    # industrial height; no ceiling (light + GUI view)
OUTER = (15.0, 11.0)
DOOR = 0.9                      # nominal opening in the wall lists below
DOOR_EXTRA = 0.10               # each side: a 0.9 m gap becomes 1.1 m, so the frames
#                                 still leave a full 1.0 m for the robot to walk through

# ring corridor: the band between these two rectangles (1.4 m wide)
R_OUT = (2.6, 2.6, 12.4, 8.4)
R_IN = (4.0, 4.0, 11.0, 7.0)    # equipment core, inside the loop

# strips of perimeter rooms
S_TOP, N_BOT = 2.6, 8.4         # south rooms end / north rooms start
IN0 = WALL_T
IN1X, IN1Y = OUTER[0] - WALL_T, OUTER[1] - WALL_T

# Robot start: south-west corner of the ring, facing east (+x) - inside the loop.
START = dict(x=3.3, y=3.3, yaw=1.5708)

WALLS = [
    # outer shell
    (0.0, 0.0, OUTER[0], 0.0, []),
    (0.0, OUTER[1], OUTER[0], OUTER[1], []),
    (0.0, 0.0, 0.0, OUTER[1], []),
    (OUTER[0], 0.0, OUTER[0], OUTER[1], []),
    # ring outer boundary (corridor <-> perimeter rooms), one door per room
    (R_OUT[0], R_OUT[1], R_OUT[2], R_OUT[1], [(3.4, 4.3), (7.0, 7.9), (10.9, 11.8)]),
    (R_OUT[0], R_OUT[3], R_OUT[2], R_OUT[3], [(3.6, 4.5), (7.2, 8.1), (11.0, 11.9)]),
    (R_OUT[0], R_OUT[1], R_OUT[0], R_OUT[3], [(5.0, 5.9)]),
    (R_OUT[2], R_OUT[1], R_OUT[2], R_OUT[3], [(5.2, 6.1)]),
    # equipment core: doors at both ends make it a through route
    (R_IN[0], R_IN[1], R_IN[2], R_IN[1], [(6.0, 6.9)]),
    (R_IN[0], R_IN[3], R_IN[2], R_IN[3], [(8.2, 9.1)]),
    (R_IN[0], R_IN[1], R_IN[0], R_IN[3], []),
    (R_IN[2], R_IN[1], R_IN[2], R_IN[3], []),
    # south strip dividers (lab | loading | testing)
    (5.0, IN0, 5.0, S_TOP, [(1.0, 1.9)]),          # lab <-> loading shortcut
    (10.0, IN0, 10.0, S_TOP, []),
    # north strip dividers (storage | workshop | maintenance)
    (5.0, N_BOT, 5.0, IN1Y, []),
    (10.0, N_BOT, 10.0, IN1Y, [(9.4, 10.3)]),      # workshop <-> maintenance shortcut
    # west strip (parts store) and east strip (equipment room) against the strips
    (IN0, S_TOP, R_OUT[0], S_TOP, [(1.0, 1.9)]),   # parts store <-> lab
    (IN0, N_BOT, R_OUT[0], N_BOT, []),
    (R_OUT[2], S_TOP, IN1X, S_TOP, [(13.2, 14.1)]),   # equipment room <-> testing
    (R_OUT[2], N_BOT, IN1X, N_BOT, [(13.4, 14.3)]),   # equipment room <-> maintenance
]

# Zones tile the interior; each is a list of rectangles.
ZONES = {
    "ring": ([(R_OUT[0], R_OUT[1], R_OUT[2], R_IN[1]),          # south leg
              (R_OUT[0], R_IN[3], R_OUT[2], R_OUT[3]),          # north leg
              (R_OUT[0], R_IN[1], R_IN[0], R_IN[3]),            # west leg
              (R_IN[2], R_IN[1], R_OUT[2], R_IN[3])],           # east leg
             "floor_lane", "wall_corridor", "ring corridor"),
    "core": ([R_IN], "floor_checkerplate", "wall_machine", "equipment core"),
    "lab": ([(IN0, IN0, 5.0, S_TOP)], "floor_esd", "wall_lab", "robotics lab"),
    "loading": ([(5.0, IN0, 10.0, S_TOP)], "floor_hazard", "wall_loading", "loading area"),
    "testing": ([(10.0, IN0, IN1X, S_TOP)], "floor_epoxy_green", "wall_testing", "testing area"),
    "storage": ([(IN0, N_BOT, 5.0, IN1Y)], "floor_concrete", "wall_storage", "storage room"),
    "workshop": ([(5.0, N_BOT, 10.0, IN1Y)], "floor_concrete_oil", "wall_workshop", "workshop"),
    "maintenance": ([(10.0, N_BOT, IN1X, IN1Y)], "floor_steel_plate", "wall_maintenance",
                    "maintenance"),
    "parts": ([(IN0, S_TOP, R_OUT[0], N_BOT)], "floor_tile_grey", "wall_parts", "parts store"),
    "equipment": ([(R_OUT[2], S_TOP, IN1X, N_BOT)], "floor_concrete", "wall_equipment",
                  "equipment room"),
}

# Props: (model, x, y, yaw). Placed to keep >= 0.9 m lanes for a 0.53 m robot and
# to give every zone a recognisable arrangement.
PROPS = [
    # --- equipment core: machines the ring wraps around; the through route runs
    #     south door -> between the machines -> north door
    ("machine_cnc", 5.15, 4.75, 0.0), ("machine_press", 7.65, 4.75, 0.0),
    ("control_panel", 10.40, 4.45, 0.0), ("machine_cnc", 10.10, 6.35, 3.1416),
    ("pipe_run", 5.35, 6.80, 0.0), ("cable_reel", 7.20, 6.55, 0.0),
    ("toolbox", 5.00, 5.90, 0.5),
    # --- robotics lab: test cell west, bench south, clear lane to both doors
    ("robot_cell", 1.35, 1.15, 0.0), ("workbench", 3.60, 0.60, 0.0),
    ("control_panel", 4.70, 0.62, -1.5708), ("toolbox", 2.75, 1.95, 0.2),
    # --- loading area: bays either side of the dock lane (door x 7.0-7.9)
    ("pallet", 6.60, 0.55, 0.0), ("crate", 5.45, 0.60, 0.1), ("pallet", 8.90, 0.60, 0.0),
    ("crate", 9.60, 1.70, 0.4), ("barrel", 8.25, 2.15, 0.0), ("barrel", 8.80, 2.15, 0.0),
   
    # --- testing area: cell east, bench and pillar west, door lane (10.9-11.8) clear
    ("robot_cell", 13.00, 1.30, 0.0), ("workbench", 10.85, 0.70, 0.0),
    ("pillar", 10.30, 2.10, 0.0), ("control_panel", 14.55, 1.60, -1.5708),
    # --- storage room: two shelf rows with a 1.0 m aisle in front of the door
    ("shelf_unit", 0.90, 9.15, 0.0), ("shelf_unit", 2.00, 9.15, 0.0), ("shelf_unit", 3.10, 9.15, 0.0),
    ("shelf_unit", 0.90, 10.45, 3.1416), ("shelf_unit", 2.00, 10.45, 3.1416),
    ("shelf_unit", 3.10, 10.45, 3.1416), ("pillar", 4.60, 10.40, 0.0),
    # --- workshop: benches along the north wall, press east, lockers west
    ("workbench", 5.90, 10.40, 3.1416), ("workbench", 7.35, 10.40, 3.1416),
    ("machine_press", 9.20, 10.20, 3.1416), ("locker", 5.35, 8.80, 0.0),
    ("locker", 5.80, 8.80, 0.0), ("toolbox", 6.60, 8.85, -0.3), ("cable_reel", 8.90, 9.00, 0.0),
    # --- maintenance: low pipe run, panels, spares
    ("pipe_run", 11.60, 10.55, 0.0), ("control_panel", 14.50, 9.60, -1.5708),
    ("barrel", 10.55, 8.80, 0.0), ("shelf_unit", 13.60, 10.45, 3.1416),
    ("toolbox", 12.40, 8.80, 0.4),
    # --- parts store: shelving both sides, one clear aisle, door (y 5.0-5.9) free
    ("shelf_unit", 0.70, 3.60, 1.5708), ("shelf_unit", 0.70, 4.70, 1.5708),
    ("shelf_unit", 0.70, 7.00, 1.5708), ("shelf_unit", 2.05, 3.60, -1.5708),
    ("shelf_unit", 2.05, 7.00, -1.5708), ("crate", 1.40, 6.30, 0.3),
    # --- equipment room: door (y 5.2-6.1) kept clear
    ("machine_cnc", 13.40, 3.60, 1.5708), ("barrel", 14.30, 5.05, 0.0),
    ("barrel", 14.30, 5.60, 0.0), ("cable_reel", 14.20, 7.10, 0.0),
    ("pipe_run", 13.60, 7.95, 0.0),
    # The ring corridor is deliberately clear of props: it is the loop under
    # test, and a 1.25 m corridor leaves no room for obstacles plus a walking
    # lane. Corridor obstacles belong in a Nav2 variant of this world later.
]

# Wall signage: (texture, x, y, yaw, height). Door numbers sit at 0.75 m, low
# enough for a camera 0.14 m off the floor to read them from a few metres.
SIGNS = [
    ("door_d1", 3.85, R_OUT[1] + 0.1, 0.0, 0.75),
    ("door_d2", 7.45, R_OUT[1] + 0.1, 0.0, 0.75),
    ("door_d3", 11.35, R_OUT[1] + 0.1, 0.0, 0.75),
    ("door_d4", 4.05, R_OUT[3] - 0.1, 3.1416, 0.75),
    ("door_d5", 7.65, R_OUT[3] - 0.1, 3.1416, 0.75),
    ("door_d6", 11.45, R_OUT[3] - 0.1, 3.1416, 0.75),
    ("door_d7", R_OUT[0] + 0.1, 5.45, 1.5708, 0.75),
    ("door_d8", R_OUT[2] - 0.1, 5.65, -1.5708, 0.75),
    ("sign_core", 6.45, R_IN[1] - 0.1, 0.0, 0.9),
    ("sign_hazard", 5.6, R_IN[1] - 0.1, 0.0, 1.2),
    ("sign_exit", 0.12, 5.4, 1.5708, 1.1),
    ("sign_loading", 8.0, 0.12, 0.0, 1.0),
    ("sign_lab", 2.4, 0.12, 0.0, 1.0),
    ("sign_storage", 2.6, OUTER[1] - 0.12, 3.1416, 1.0),
    ("sign_workshop", 7.4, OUTER[1] - 0.12, 3.1416, 1.0),
    ("panel_grid", 14.85, 4.4, -1.5708, 0.9),
    ("panel_grid", 0.12, 7.4, 1.5708, 0.9),
]

# Wall decals: (texture, x, y, yaw, z, length, height) - VISUAL ONLY, no
# collision, a 10 mm plate laid on an existing wall face. They exist for one
# reason: the face camera sits 0.060 m off the floor, so where a corridor ends
# in a bare wall the whole frame fills with that wall's kick plate, and a flat
# painted band gives a corner detector nothing at all.
#
# The ring's NW corner (3.30, 7.70) is the only place in the facility where that
# happens. Measured through the robot's own camera, ORB keypoints there:
#
#     SW corner  385-531      NE corner  503-811
#     SE corner   15-339      NW corner    0-6      <- visual odometry dies here
#
# The west wall at that corner wears "wall_parts", the one plant_wall in the
# facility with neither an accent band nor a label, so below 0.4 m it is a
# single flat grey. This strip runs along the stretch the camera stares into on
# the final approach and through the first 30 deg of the turn.
DECALS = [
    ("kick_service_strip", R_OUT[0] + WALL_T / 2 + 0.005, 7.65, 0.0, 0.20, 1.30, 0.36),
]

LIGHTS = [
    # One lamp per major area: enough to vary brightness between places without
    # paying for a dozen dynamic lights while the camera renders every frame.
    ("lamp_ring_w", 3.30, 5.50, 2.5, 0.96, 0.96, 0.98),
    ("lamp_ring_e", 11.70, 5.50, 2.5, 0.96, 0.96, 0.98),
    ("lamp_core", 7.50, 5.50, 2.5, 1.00, 0.98, 0.92),
    ("lamp_south", 7.50, 1.40, 2.5, 1.00, 0.94, 0.84),
    ("lamp_north", 7.50, 9.60, 2.5, 0.94, 0.98, 1.00),
    ("lamp_west", 1.40, 5.50, 2.4, 0.95, 0.95, 0.95),
    ("lamp_east", 13.60, 5.50, 2.4, 0.98, 0.96, 0.90),
    ("lamp_southeast", 12.40, 1.40, 2.5, 0.96, 1.00, 0.94),
    ("lamp_southwest", 2.40, 1.40, 2.5, 0.92, 0.96, 1.00),
]


# Camera viewpoints for the visual check: (label, x, y, yaw), at robot camera
# height. Chosen to cover every zone plus both corridor legs, from where the
# robot would actually stand.
VIEWPOINTS = [
    ("ring S-west", 3.30, 3.30, 0.0), ("ring S-mid", 7.50, 3.30, 0.0),
    ("ring S-east", 11.70, 3.30, 1.5708), ("ring east", 11.70, 5.50, 1.5708),
    ("ring N-east", 11.70, 7.70, 3.1416), ("ring N-mid", 7.50, 7.70, 3.1416),
    ("ring N-west", 3.30, 7.70, -1.5708), ("ring west", 3.30, 5.50, -1.5708),
    ("core south", 6.45, 4.60, 1.5708), ("core middle", 7.50, 5.75, 0.0),
    ("robotics lab", 3.00, 1.50, 3.1416), ("loading area", 7.40, 1.60, 0.0),
    ("testing area", 11.30, 2.05, 0.0), ("storage room", 4.00, 9.60, 3.1416),
    ("workshop", 7.50, 9.30, 3.1416), ("maintenance", 12.50, 9.50, 0.0),
    ("parts store", 1.40, 5.00, 1.5708), ("equipment room", 13.60, 5.50, 1.5708),
]


def wall_boxes():
    """Split each wall into solid boxes around its doorways -> (cx, cy, length, vertical)."""
    for x0, y0, x1, y1, gaps in WALLS:
        vertical = abs(x1 - x0) < 1e-9
        start, end = (y0, y1) if vertical else (x0, x1)
        fixed = x0 if vertical else y0
        pos = start
        for g0, g1 in sorted(gaps):
            g0, g1 = g0 - DOOR_EXTRA, g1 + DOOR_EXTRA
            if g0 > pos:
                yield _box(fixed, pos, g0, vertical)
            pos = g1
        if end > pos:
            yield _box(fixed, pos, end, vertical)


def door_frames():
    """A slim post each side of every opening: cheap geometry that gives the
    camera strong vertical edges exactly where the robot drives."""
    for x0, y0, x1, y1, gaps in WALLS:
        vertical = abs(x1 - x0) < 1e-9
        fixed = x0 if vertical else y0
        for g0, g1 in gaps:
            for edge in (g0 - DOOR_EXTRA, g1 + DOOR_EXTRA):
                yield (fixed, edge, vertical)


def _box(fixed, a, b, vertical):
    length = b - a
    mid = 0.5 * (a + b)
    return (fixed, mid, length, vertical) if vertical else (mid, fixed, length, vertical)


def zone_rects():
    for name, (rects, floor, wall, label) in ZONES.items():
        for rect in rects:
            yield name, rect, floor, wall, label


def loop_ring():
    """The full ring corridor (~26 m)."""
    m = 0.7                                   # corridor centre line
    return [(R_OUT[0] + m, R_OUT[1] + m), (R_OUT[2] - m, R_OUT[1] + m),
            (R_OUT[2] - m, R_OUT[3] - m), (R_OUT[0] + m, R_OUT[3] - m),
            (R_OUT[0] + m, R_OUT[1] + m)]


def loop_core():
    """Ring west leg plus the through route across the equipment core (~17 m).
    Inside the core the path runs between the machines and the north-wall props."""
    m = 0.7
    return [(R_OUT[0] + m, R_OUT[1] + m), (6.45, R_OUT[1] + m), (6.45, 5.75), (8.65, 5.75),
            (8.65, R_OUT[3] - m), (R_OUT[0] + m, R_OUT[3] - m), (R_OUT[0] + m, R_OUT[1] + m)]


def loop_rooms():
    """Ring plus the shortcut through the lab and loading area (~13 m)."""
    m = 0.7
    return [(R_OUT[0] + m, R_OUT[1] + m), (3.85, R_OUT[1] + m), (3.85, 1.45), (7.45, 1.45),
            (7.45, R_OUT[1] + m), (R_OUT[0] + m, R_OUT[1] + m)]
