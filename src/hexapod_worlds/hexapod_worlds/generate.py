#!/usr/bin/env python3
"""Generate the hexapod's industrial robotics facility: textures, models, world, map.

Everything is built from primitives (boxes and cylinders) plus procedural
textures, so the world loads without downloading anything, renders RGB-D at a
useful real-time factor, and is reproducible from source.

    ros2 run hexapod_worlds generate_slam_world

Outputs (committed, so the world works without re-running this):
    worlds/hexapod_facility.sdf
    models/slam_assets/materials/textures/*.png   shared textures
    models/<prop>/                                reusable prop models
    docs/facility_map.png                         top-down plan with the test loops
"""
import os

from PIL import Image, ImageDraw, ImageFont

from hexapod_worlds import layout as L
from hexapod_worlds import textures as T

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX = "model://slam_assets/materials/textures"
SAVE_PX = 320      # texture size written to disk


# --------------------------------------------------------------------- helpers
def material(texture):
    return f"""<material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.15 0.15 0.15 1</specular>
          <pbr><metal>
            <albedo_map>{TEX}/{texture}.png</albedo_map>
            <roughness>0.85</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>"""


# Parts are emitted as <collision>/<visual> pairs and grouped into ONE link per
# model. Gazebo treats every link as a rigid body, so a facility built from a few
# hundred one-box links runs at about half real time; the same geometry inside a
# handful of links runs near 1.0 and looks identical.
def box(name, size, pose, texture, collide=True, friction=None):
    fr = f"""
          <surface><friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction></surface>""" \
        if friction else ""
    col = f"""
        <collision name="{name}_col"><pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>{fr}</collision>""" if collide else ""
    return f"""{col}
        <visual name="{name}_vis"><pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
          {material(texture)}
        </visual>"""


def cyl(name, radius, length, pose, texture, collide=True):
    col = f"""
        <collision name="{name}_col"><pose>{pose}</pose>
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder>
          </geometry></collision>""" if collide else ""
    return f"""{col}
        <visual name="{name}_vis"><pose>{pose}</pose>
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          {material(texture)}
        </visual>"""


def one_link(name, parts):
    return f"""
      <link name="{name}">{parts}
      </link>"""


# ---------------------------------------------------------------------- props
def prop_models():
    """Reusable facility props, all static scenery and reference points."""
    pillar = (box("plate", "0.42 0.42 0.05", "0 0 0.025 0 0 0", "metal_panel")
              + box("shaft", "0.26 0.26 2.6", "0 0 1.32 0 0 0", "pillar_paint"))
    crate = box("body", "0.50 0.50 0.45", "0 0 0.225 0 0 0", "crate_wood")
    pallet = (box("deck", "1.20 0.80 0.06", "0 0 0.12 0 0 0", "pallet_wood")
              + box("footA", "1.20 0.10 0.09", "0 -0.33 0.045 0 0 0", "pallet_wood")
              + box("footB", "1.20 0.10 0.09", "0 0.33 0.045 0 0 0", "pallet_wood"))
    shelf = "".join([
        box("postA", "0.06 0.45 1.90", "-0.47 0 0.95 0 0 0", "metal_panel"),
        box("postB", "0.06 0.45 1.90", "0.47 0 0.95 0 0 0", "metal_panel"),
        box("deck0", "1.00 0.45 0.05", "0 0 0.35 0 0 0", "metal_panel"),
        box("deck1", "1.00 0.45 0.05", "0 0 0.90 0 0 0", "metal_panel"),
        box("deck2", "1.00 0.45 0.05", "0 0 1.45 0 0 0", "metal_panel"),
        box("bins0", "0.92 0.38 0.26", "0 0 0.50 0 0 0", "shelf_bins", collide=False),
        box("bins1", "0.92 0.38 0.26", "0 0 1.05 0 0 0", "shelf_bins", collide=False),
    ])
    barrel = (cyl("drum", 0.24, 0.88, "0 0 0.44 0 0 0", "barrel_skin")
              + cyl("lid", 0.25, 0.04, "0 0 0.90 0 0 0", "metal_brushed"))
    workbench = "".join([
        box("top", "1.40 0.70 0.07", "0 0 0.80 0 0 0", "bench_top"),
        box("legA", "0.07 0.07 0.78", "-0.65 -0.30 0.39 0 0 0", "metal_panel"),
        box("legB", "0.07 0.07 0.78", "0.65 -0.30 0.39 0 0 0", "metal_panel"),
        box("legC", "0.07 0.07 0.78", "-0.65 0.30 0.39 0 0 0", "metal_panel"),
        box("legD", "0.07 0.07 0.78", "0.65 0.30 0.39 0 0 0", "metal_panel"),
        box("undershelf", "1.30 0.60 0.05", "0 0 0.28 0 0 0", "metal_panel"),
        box("vice", "0.22 0.18 0.20", "0.52 0.0 0.93 0 0 0", "metal_brushed"),
    ])
    machine_cnc = "".join([
        box("base", "1.30 1.00 0.25", "0 0 0.125 0 0 0", "metal_panel"),
        box("body", "1.20 0.90 1.30", "0 0 0.90 0 0 0", "machine_skin"),
        box("hood", "1.24 0.94 0.12", "0 0 1.60 0 0 0", "metal_brushed"),
        box("screen", "0.42 0.05 0.34", "0.30 -0.47 1.15 0 0 0", "control_face", collide=False),
    ])
    machine_press = "".join([
        box("base", "1.00 1.00 0.30", "0 0 0.15 0 0 0", "metal_panel"),
        box("column", "0.55 0.55 1.80", "-0.18 0 1.20 0 0 0", "machine_skin_b"),
        box("ram", "0.70 0.70 0.35", "0.22 0 1.55 0 0 0", "metal_brushed"),
        box("table", "0.80 0.80 0.10", "0.22 0 0.65 0 0 0", "metal_panel"),
    ])
    control_panel = "".join([
        box("cabinet", "0.75 0.35 1.65", "0 0 0.825 0 0 0", "metal_panel"),
        box("face", "0.66 0.04 0.95", "0 -0.18 1.00 0 0 0", "control_face", collide=False),
        box("plinth", "0.80 0.40 0.08", "0 0 0.04 0 0 0", "metal_brushed"),
    ])
    pipe_run = "".join([                       # low pipes: features right at camera height
        cyl("pipeA", 0.07, 2.20, "0 0 0.28 0 1.5708 0", "pipe_metal"),
        cyl("pipeB", 0.05, 2.20, "0 0 0.46 0 1.5708 0", "pipe_metal"),
        box("bracketA", "0.10 0.16 0.55", "-0.95 0 0.27 0 0 0", "metal_panel"),
        box("bracketB", "0.10 0.16 0.55", "0.95 0 0.27 0 0 0", "metal_panel"),
    ])
    cable_reel = "".join([
        cyl("drum", 0.45, 0.50, "0 0 0.45 1.5708 0 0", "cable_black"),
        cyl("flangeA", 0.52, 0.04, "0 -0.25 0.45 1.5708 0 0", "metal_panel"),
        cyl("flangeB", 0.52, 0.04, "0 0.25 0.45 1.5708 0 0", "metal_panel"),
        box("frame", "0.30 0.62 0.20", "0 0 0.10 0 0 0", "metal_panel"),
    ])
    toolbox = (box("chest", "0.62 0.34 0.38", "0 0 0.19 0 0 0", "metal_panel")
               + box("tray", "0.50 0.28 0.10", "0 0 0.43 0 0 0", "metal_brushed"))
    locker = (box("body", "0.40 0.50 1.80", "0 0 0.90 0 0 0", "metal_brushed")
              + box("vent", "0.30 0.03 0.30", "0 -0.26 1.45 0 0 0", "metal_panel", collide=False))
    robot_cell = "".join([                     # fenced test cell: open on one side
        box("mat", "1.60 1.60 0.02", "0 0 0.01 0 0 0", "rubber_mat"),
        box("fenceN", "1.60 0.05 1.20", "0 0.8 0.60 0 0 0", "fence_mesh"),
        box("fenceW", "0.05 1.60 1.20", "-0.8 0 0.60 0 0 0", "fence_mesh"),
        box("fenceE", "0.05 1.60 1.20", "0.8 0 0.60 0 0 0", "fence_mesh"),
        box("postA", "0.09 0.09 1.30", "-0.8 0.8 0.65 0 0 0", "metal_panel"),
        box("postB", "0.09 0.09 1.30", "0.8 0.8 0.65 0 0 0", "metal_panel"),
        box("rig", "0.55 0.55 0.75", "0 0.35 0.375 0 0 0", "machine_skin"),
    ])
    return {
        "pillar": ("Structural pillar", pillar),
        "crate": ("Parts crate", crate),
        "pallet": ("Wooden pallet", pallet),
        "shelf_unit": ("Storage shelving with bins", shelf),
        "barrel": ("Coolant barrel", barrel),
        "workbench": ("Workbench with vice", workbench),
        "machine_cnc": ("CNC machine", machine_cnc),
        "machine_press": ("Press", machine_press),
        "control_panel": ("Control panel", control_panel),
        "pipe_run": ("Low pipe run", pipe_run),
        "cable_reel": ("Cable reel", cable_reel),
        "toolbox": ("Tool chest", toolbox),
        "locker": ("Staff locker", locker),
        "robot_cell": ("Fenced robot test cell", robot_cell),
    }


def model_config(name, description):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>{description}</description>
</model>
"""


def write_models(root):
    for name, (desc, links) in prop_models().items():
        d = os.path.join(root, "models", name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "model.config"), "w") as f:
            f.write(model_config(name, desc))
        with open(os.path.join(d, "model.sdf"), "w") as f:
            f.write(f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <static>true</static>{one_link("body", links)}
  </model>
</sdf>
""")
    assets = os.path.join(root, "models", "slam_assets")
    tex_dir = os.path.join(assets, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)
    with open(os.path.join(assets, "model.config"), "w") as f:
        f.write(model_config("slam_assets", "Shared procedural textures for the facility"))
    with open(os.path.join(assets, "model.sdf"), "w") as f:
        f.write("""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="slam_assets"><static>true</static><link name="unused"/></model>
</sdf>
""")
    textures = T.all_textures()
    for name, img in textures.items():
        # Saved smaller and palette-quantised: the patterns survive at camera range
        # and the repo does not carry 20 MB of per-pixel noise. A texture drawn
        # non-square is one meant for a non-square face (the wall decals), so its
        # aspect is kept - squashing it into SAVE_PX^2 would smear the pattern
        # back out again when the face stretches it.
        w, h = img.size
        scale = SAVE_PX / max(w, h)
        size = (max(1, round(w * scale)), max(1, round(h * scale)))
        small = img.resize(size, Image.LANCZOS)
        small.convert("P", palette=Image.ADAPTIVE, colors=96).save(
            os.path.join(tex_dir, f"{name}.png"), optimize=True)
    return len(textures)


# ----------------------------------------------------------------------- world
def zone_at(x, y):
    for name, rect, floor, wall, label in L.zone_rects():
        x0, y0, x1, y1 = rect
        if x0 - 1e-9 <= x <= x1 + 1e-9 and y0 - 1e-9 <= y <= y1 + 1e-9:
            return name, floor, wall, label
    return None


def wall_texture(cx, cy, vertical):
    """A wall wears the material of the space it faces; rooms win over the
    corridor, so every room keeps its own identity."""
    off = L.WALL_T / 2 + 0.25
    sides = [(cx - off, cy), (cx + off, cy)] if vertical else [(cx, cy - off), (cx, cy + off)]
    found = [zone_at(x, y) for x, y in sides]
    rooms = [z for z in found if z and z[0] != "ring"]
    pick = rooms[0] if rooms else next((z for z in found if z), None)
    return pick[2] if pick else "wall_corridor"


def build_world():
    parts = []

    # Ground beyond the building: VISUAL ONLY. A collision plane here would sit
    # exactly under the floor slabs, and the feet would chatter between two
    # coincident surfaces instead of walking.
    parts.append("""
    <model name="ground">
      <static>true</static>
      <link name="plane">
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><diffuse>0.55 0.56 0.58 1</diffuse></material>
        </visual>
      </link>
    </model>""")

    # floors: one textured slab per zone rectangle
    floors = []
    for i, (name, (x0, y0, x1, y1), floor, wall, label) in enumerate(L.zone_rects()):
        # top face flush with z=0, so the robot stands at the same height as in the
        # empty world the gait was verified in
        floors.append(box(f"floor_{name}_{i}", f"{x1 - x0:.3f} {y1 - y0:.3f} 0.04",
                          f"{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} -0.02 0 0 0", floor,
                          friction=1.0))
    parts.append(f"""
    <model name="floors">
      <static>true</static>{one_link("slabs", ''.join(floors))}
    </model>""")

    # walls, door frames and signs
    links = []
    for i, (cx, cy, length, vertical) in enumerate(L.wall_boxes()):
        size = (f"{L.WALL_T} {length:.3f} {L.WALL_H}" if vertical
                else f"{length:.3f} {L.WALL_T} {L.WALL_H}")
        links.append(box(f"wall_{i}", size, f"{cx:.3f} {cy:.3f} {L.WALL_H / 2:.3f} 0 0 0",
                         wall_texture(cx, cy, vertical)))
    for i, (fixed, edge, vertical) in enumerate(L.door_frames()):
        size = f"{L.WALL_T + 0.04} 0.10 2.10" if not vertical else f"0.10 {L.WALL_T + 0.04} 2.10"
        pose = (f"{fixed:.3f} {edge:.3f} 1.05 0 0 0" if vertical
                else f"{edge:.3f} {fixed:.3f} 1.05 0 0 0")
        links.append(box(f"doorframe_{i}", size, pose, "pillar_paint"))
    for i, (tex, x, y, yaw, z) in enumerate(L.SIGNS):
        links.append(box(f"sign_{i}", "0.55 0.03 0.40", f"{x} {y} {z} 0 0 {yaw}", tex))
    # Wall decals are laid ON an existing wall face and are visual only: no
    # collision box, so the robot's world is geometrically identical with or
    # without them. They add the surface detail a camera 0.06 m off the floor
    # needs where a corridor dead-ends in a bare kick plate.
    for i, (tex, x, y, yaw, z, length, height) in enumerate(L.DECALS):
        links.append(box(f"decal_{i}", f"0.010 {length:.3f} {height:.3f}",
                         f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw}", tex, collide=False))
    parts.append(f"""
    <model name="building">
      <static>true</static>{one_link("structure", ''.join(links))}
    </model>""")

    for i, (model, x, y, yaw) in enumerate(L.PROPS):
        parts.append(f"""
    <include>
      <uri>model://{model}</uri>
      <name>{model}_{i}</name>
      <pose>{x} {y} 0 0 0 {yaw}</pose>
    </include>""")

    lights = "".join(f"""
    <light type="point" name="{n}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <diffuse>{r} {g} {b} 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <attenuation><range>9</range><constant>0.35</constant><linear>0.06</linear>
        <quadratic>0.006</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""" for n, x, y, z, r, g, b in L.LIGHTS)

    return f"""<?xml version="1.0"?>
<!-- Hexapod industrial robotics facility: an RGB-D SLAM benchmark world.
     Generated by hexapod_worlds/generate.py - edit layout.py / textures.py. -->
<sdf version="1.10">
  <world name="hexapod_facility">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <scene>
      <ambient>0.5 0.5 0.53 1</ambient>
      <background>0.72 0.75 0.80 1</background>
      <grid>false</grid>
      <!-- shadows off: they cost more than they add for a
           camera 0.14 m off the floor, and RGB-D rendering is the budget -->
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <pose>0 0 12 0 0 0</pose>
      <direction>-0.4 0.35 -0.9</direction>
      <diffuse>0.85 0.85 0.83 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <cast_shadows>false</cast_shadows>
    </light>{lights}
{''.join(parts)}
  </world>
</sdf>
"""


# --------------------------------------------------------------------- plan view
def write_map(root, px_per_m=90):
    w, h = L.OUTER
    img = Image.new("RGB", (int(w * px_per_m) + 40, int(h * px_per_m) + 70), (252, 252, 252))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(T.FONT_PATHS[0], 15)
        small = ImageFont.truetype(T.FONT_PATHS[0], 13)
    except OSError:
        font = small = ImageFont.load_default()

    def px(x, y):
        return 20 + x * px_per_m, 20 + (h - y) * px_per_m

    fills = {"ring": (226, 232, 240), "core": (208, 214, 224), "lab": (214, 232, 246),
             "loading": (250, 234, 194), "testing": (214, 240, 220), "storage": (250, 226, 200),
             "workshop": (206, 236, 238), "maintenance": (226, 224, 220),
             "parts": (236, 236, 240), "equipment": (224, 228, 234)}
    drawn = set()
    for name, (x0, y0, x1, y1), floor, wall, label in L.zone_rects():
        d.rectangle([*px(x0, y1), *px(x1, y0)], fill=fills.get(name, (240, 240, 240)))
        if name not in drawn:
            cx, cy = px((x0 + x1) / 2, (y0 + y1) / 2)
            d.text((cx - 42, cy - 8), label, fill=(80, 80, 88), font=font)
            drawn.add(name)
    for cx, cy, length, vertical in L.wall_boxes():
        sx, sy = (L.WALL_T, length) if vertical else (length, L.WALL_T)
        d.rectangle([*px(cx - sx / 2, cy + sy / 2), *px(cx + sx / 2, cy - sy / 2)], fill=(55, 55, 60))
    for model, x, y, yaw in L.PROPS:
        cx, cy = px(x, y)
        r = 10 if model.startswith("machine") or model == "robot_cell" else 7
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(196, 74, 60), outline=(120, 30, 20))
    for tex, x, y, yaw, z in L.SIGNS:
        cx, cy = px(x, y)
        d.rectangle([cx - 5, cy - 5, cx + 5, cy + 5], fill=(40, 90, 200))
    for pts, color, width in ((L.loop_ring(), (26, 140, 70), 5),
                              (L.loop_core(), (40, 90, 210), 4),
                              (L.loop_rooms(), (190, 120, 30), 3)):
        d.line([px(x, y) for x, y in pts], fill=color, width=width)
    sx, sy = px(L.START["x"], L.START["y"])
    d.ellipse([sx - 12, sy - 12, sx + 12, sy + 12], fill=(20, 170, 70), outline=(0, 80, 20))
    d.text((sx + 15, sy - 9), "START", fill=(20, 110, 40), font=font)
    legend = [("ring loop ~26 m", (26, 140, 70)), ("core loop ~17 m", (40, 90, 210)),
              ("rooms loop ~24 m", (190, 120, 30)), ("props", (196, 74, 60)),
              ("signs", (40, 90, 200))]
    x = 24
    for text, color in legend:
        y = int(h * px_per_m) + 36
        d.rectangle([x, y, x + 16, y + 12], fill=color)
        d.text((x + 22, y - 2), text, fill=(60, 60, 66), font=small)
        x += 26 + 8 * len(text)
    out = os.path.join(root, "docs")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "facility_map.png")
    img.save(path)
    return path


def source_dir():
    """Where to write. Running the installed entry point would otherwise drop the
    generated world into install/, where the next build overwrites it."""
    for guess in (os.path.join(os.getcwd(), "src", "hexapod_worlds"),
                  os.getcwd() if os.path.basename(os.getcwd()) == "hexapod_worlds" else None):
        if guess and os.path.isdir(os.path.join(guess, "hexapod_worlds")):
            return guess
    return PKG


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=None, help="package directory to write into")
    args = ap.parse_args()
    global PKG
    PKG = args.out or source_dir()
    print(f"writing into {PKG}")
    n_tex = write_models(PKG)
    os.makedirs(os.path.join(PKG, "worlds"), exist_ok=True)
    world_path = os.path.join(PKG, "worlds", "hexapod_facility.sdf")
    with open(world_path, "w") as f:
        f.write(build_world())
    map_path = write_map(PKG)
    print(f"textures : {n_tex}")
    print(f"models   : {len(prop_models())} props + slam_assets")
    print(f"world    : {os.path.relpath(world_path, PKG)} "
          f"({len(list(L.wall_boxes()))} wall boxes, {len(list(L.door_frames()))} door frames, "
          f"{len(L.PROPS)} props, {len(L.SIGNS)} signs, {len(L.DECALS)} decals, "
          f"{len(list(L.zone_rects()))} floor slabs)")
    print(f"map      : {os.path.relpath(map_path, PKG)}")


if __name__ == "__main__":
    main()
