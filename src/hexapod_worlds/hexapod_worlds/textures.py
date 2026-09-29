"""Procedural textures for the hexapod's industrial robotics facility.

Generated rather than downloaded: the world loads with no network, files stay
small, and every pattern is chosen for what an RGB-D SLAM front end needs -
corners, edges and locally distinct detail instead of smooth surfaces that give
a feature detector nothing to hold on to.

The camera sits 0.14 m above the floor, so the detail that matters most lives
low: floor lane markings, kick plates, hazard borders and door numbers.
"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SIZE = 512      # drawing canvas; generate.py saves a smaller, quantised copy
FONT_PATHS = ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"]


def _font(size):
    for path in FONT_PATHS:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _grain(img, amount=10, seed=0):
    rng = np.random.default_rng(seed)
    arr = np.asarray(img).astype(np.int16)
    arr += rng.integers(-amount, amount + 1, arr.shape, dtype=np.int16)
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def _base(color):
    return Image.new("RGB", (SIZE, SIZE), color)


def _shift(c, d):
    return tuple(max(0, min(255, v + d)) for v in c)


# --------------------------------------------------------------------- walls
def plant_wall(color, kick, seed, accent=None, label=None):
    """Painted block wall: dark kick plate along the bottom (what the low camera
    sees), a colour band at chest height, and faint block joints."""
    img = _base(color)
    d = ImageDraw.Draw(img)
    for y in range(0, SIZE, 64):                                  # block courses
        d.line([0, y, SIZE, y], fill=_shift(color, -18), width=3)
        off = 0 if (y // 64) % 2 else 64
        for x in range(-64, SIZE + 64, 128):
            d.line([x + off, y, x + off, y + 64], fill=_shift(color, -14), width=3)
    if accent:
        d.rectangle([0, int(SIZE * 0.52), SIZE, int(SIZE * 0.60)], fill=accent)
    d.rectangle([0, int(SIZE * 0.86), SIZE, SIZE], fill=kick)     # kick plate
    d.line([0, int(SIZE * 0.86), SIZE, int(SIZE * 0.86)], fill=_shift(kick, -50), width=5)
    if label:
        d.text((18, int(SIZE * 0.60)), label, fill=_shift(color, -90), font=_font(46))
    return _grain(img, 7, seed)


def panel_wall(color, seed, bolts=True):
    """Bolted steel panelling: strong repeating edges, good for tracking."""
    img = _base(color)
    d = ImageDraw.Draw(img)
    for x in range(0, SIZE + 1, 128):
        d.rectangle([x - 5, 0, x + 5, SIZE], fill=_shift(color, -35))
    for y in range(0, SIZE + 1, 170):
        d.rectangle([0, y - 5, SIZE, y + 5], fill=_shift(color, -35))
    if bolts:
        for x in range(24, SIZE, 64):
            for y in range(24, SIZE, 85):
                d.ellipse([x - 5, y - 5, x + 5, y + 5], fill=_shift(color, -70))
    d.rectangle([0, int(SIZE * 0.88), SIZE, SIZE], fill=_shift(color, -60))
    return _grain(img, 9, seed)


def mesh_wall(color, seed):
    img = _base(color)
    d = ImageDraw.Draw(img)
    for i in range(-SIZE, SIZE, 26):
        d.line([i, 0, i + SIZE, SIZE], fill=_shift(color, -40), width=3)
        d.line([i, SIZE, i + SIZE, 0], fill=_shift(color, -40), width=3)
    return _grain(img, 6, seed)


# -------------------------------------------------------------------- floors
def concrete(seed, tone=152, stains=8):
    img = _grain(_base((tone, tone, tone)), 20, seed)
    d = ImageDraw.Draw(img)
    rng = np.random.default_rng(seed)
    for _ in range(stains):
        x, y = rng.integers(0, SIZE, 2)
        r = int(rng.integers(25, 80))
        d.ellipse([x - r, y - r, x + r, y + r], fill=(tone - 14,) * 3)
    d.line([0, SIZE // 2, SIZE, SIZE // 2], fill=(tone - 40,) * 3, width=4)   # saw cuts
    d.line([SIZE // 2, 0, SIZE // 2, SIZE], fill=(tone - 40,) * 3, width=4)
    return _grain(img, 7, seed + 5)


def oily_concrete(seed):
    img = concrete(seed, 128, stains=4)
    d = ImageDraw.Draw(img)
    rng = np.random.default_rng(seed)
    for _ in range(7):                                             # oil patches
        x, y = rng.integers(60, SIZE - 60, 2)
        r = int(rng.integers(30, 70))
        d.ellipse([x - r, y - r * 3 // 4, x + r, y + r * 3 // 4], fill=(76, 74, 72))
    return img


def lane_floor(seed):
    """Corridor epoxy with painted lane edges and a dashed centre line."""
    img = _grain(_base((122, 126, 132)), 12, seed)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, SIZE, 30], fill=(232, 186, 42))
    d.rectangle([0, SIZE - 30, SIZE, SIZE], fill=(232, 186, 42))
    for x in range(0, SIZE, 72):
        d.rectangle([x, SIZE // 2 - 6, x + 36, SIZE // 2 + 6], fill=(228, 228, 228))
    return img


def hazard_floor(seed):
    img = _base((222, 190, 56))
    d = ImageDraw.Draw(img)
    for i in range(-SIZE, SIZE * 2, 118):
        d.polygon([(i, 0), (i + 59, 0), (i + 59 - SIZE, SIZE), (i - SIZE, SIZE)], fill=(48, 48, 48))
    return _grain(img, 8, seed)


def checker_plate(seed, tone=(148, 150, 156)):
    img = _base(tone)
    d = ImageDraw.Draw(img)
    for gy in range(0, SIZE, 48):
        for gx in range(0, SIZE, 48):
            off = 24 if (gy // 48) % 2 else 0
            d.line([gx + off + 6, gy + 12, gx + off + 30, gy + 26], fill=_shift(tone, 34), width=7)
            d.line([gx + off + 6, gy + 34, gx + off + 30, gy + 20], fill=_shift(tone, -34), width=7)
    return _grain(img, 9, seed)


def esd_floor(seed):
    """Anti-static lab floor: blue with a fine conductive grid."""
    img = _grain(_base((58, 92, 126)), 13, seed)
    d = ImageDraw.Draw(img)
    for i in range(0, SIZE + 1, 64):
        d.line([i, 0, i, SIZE], fill=(86, 122, 156), width=2)
        d.line([0, i, SIZE, i], fill=(86, 122, 156), width=2)
    return img


def epoxy(seed, tone):
    img = _grain(_base(tone), 10, seed)
    d = ImageDraw.Draw(img)
    for i in range(0, SIZE + 1, 256):
        d.line([0, i, SIZE, i], fill=_shift(tone, -22), width=4)
    return img


def tiles(seed, a, b, n=8, grout=(110, 110, 110)):
    img = _base(a)
    d = ImageDraw.Draw(img)
    step = SIZE // n
    for i in range(n):
        for j in range(n):
            if (i + j) % 2:
                d.rectangle([i * step, j * step, (i + 1) * step, (j + 1) * step], fill=b)
    for i in range(n + 1):
        d.line([i * step, 0, i * step, SIZE], fill=grout, width=3)
        d.line([0, i * step, SIZE, i * step], fill=grout, width=3)
    return _grain(img, 7, seed)


def steel_floor(seed):
    img = checker_plate(seed, (132, 134, 140))
    d = ImageDraw.Draw(img)
    for x in range(0, SIZE, 170):                                   # plate seams
        d.line([x, 0, x, SIZE], fill=(96, 98, 104), width=6)
    return img


# ----------------------------------------------------------------- materials
def brushed_metal(seed, tone=(178, 181, 186)):
    rng = np.random.default_rng(seed)
    arr = np.full((SIZE, SIZE, 3), tone, dtype=np.int16)
    arr += rng.integers(-11, 12, (SIZE, 1, 3), dtype=np.int16)
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def painted_metal(seed, tone):
    img = _grain(_base(tone), 8, seed)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, SIZE - 1, SIZE - 1], outline=_shift(tone, -50), width=10)
    for x in range(64, SIZE, 128):                                  # bolt rows
        for y in range(64, SIZE, 128):
            d.ellipse([x - 6, y - 6, x + 6, y + 6], fill=_shift(tone, -60))
    return img


def machine_skin(seed, tone=(66, 96, 132)):
    """Machine housing: panel seams, a vent grille and a warning stripe."""
    img = _grain(_base(tone), 7, seed)
    d = ImageDraw.Draw(img)
    d.rectangle([0, int(SIZE * 0.06), SIZE, int(SIZE * 0.12)], fill=(228, 186, 44))
    d.rectangle([40, 150, SIZE - 40, 330], fill=_shift(tone, -28), outline=_shift(tone, -60),
                width=5)
    for y in range(170, 320, 22):                                   # vents
        d.line([60, y, SIZE - 60, y], fill=_shift(tone, -55), width=7)
    d.rectangle([0, int(SIZE * 0.9), SIZE, SIZE], fill=_shift(tone, -70))
    return img


def control_face(seed):
    """Control panel: screen, buttons and labels - a highly distinctive landmark."""
    img = _base((54, 58, 66))
    d = ImageDraw.Draw(img)
    d.rectangle([28, 24, SIZE - 28, 210], fill=(24, 46, 40), outline=(120, 130, 140), width=5)
    for y in range(44, 200, 26):                                    # screen lines
        d.line([44, y, SIZE - 60, y], fill=(70, 190, 130), width=4)
    palette = [(200, 60, 50), (230, 190, 60), (80, 190, 90), (70, 120, 210)]
    for i, c in enumerate(palette):
        x = 60 + i * 100
        d.ellipse([x, 260, x + 62, 322], fill=c, outline=(20, 20, 24), width=4)
    d.rectangle([60, 360, SIZE - 60, 420], fill=(180, 184, 190), outline=(90, 94, 100), width=4)
    d.text((72, 372), "CTRL-04", fill=(30, 30, 34), font=_font(40))
    return _grain(img, 6, seed)


def crate_wood(seed):
    img = _base((162, 120, 70))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, SIZE - 1, SIZE - 1], outline=(112, 80, 44), width=24)
    d.line([0, 0, SIZE, SIZE], fill=(112, 80, 44), width=18)
    d.line([SIZE, 0, 0, SIZE], fill=(112, 80, 44), width=18)
    d.rectangle([150, 40, 380, 120], fill=(232, 226, 210))
    d.text((166, 54), "PARTS", fill=(60, 44, 24), font=_font(48))
    return _grain(img, 11, seed)


def pallet_wood(seed):
    img = _base((178, 142, 92))
    d = ImageDraw.Draw(img)
    for x in range(0, SIZE, 96):
        d.rectangle([x, 0, x + 64, SIZE], fill=(158, 124, 78), outline=(120, 92, 56), width=4)
    return _grain(img, 12, seed)


def barrel_skin(seed):
    img = _base((36, 86, 152))
    d = ImageDraw.Draw(img)
    for y in (int(SIZE * 0.26), int(SIZE * 0.70)):
        d.rectangle([0, y, SIZE, y + 28], fill=(222, 224, 228))
    d.text((52, int(SIZE * 0.40)), "COOLANT", fill=(240, 240, 240), font=_font(54))
    return _grain(img, 9, seed)


def books_or_bins(seed):
    """Shelf contents: coloured bins - clutter that makes each shelf distinct."""
    img = _base((84, 86, 92))
    d = ImageDraw.Draw(img)
    rng = np.random.default_rng(seed)
    palette = [(190, 80, 60), (70, 120, 180), (90, 160, 90), (220, 180, 70), (150, 90, 170)]
    x = 0
    while x < SIZE:
        w = int(rng.integers(40, 90))
        c = palette[int(rng.integers(0, len(palette)))]
        top = int(rng.integers(30, 110))
        d.rectangle([x, top, x + w, SIZE], fill=c, outline=_shift(c, -50), width=4)
        d.rectangle([x + 12, top + 18, x + w - 12, top + 44], fill=(238, 238, 238))
        x += w + 6
    return _grain(img, 7, seed)


def rubber_mat(seed):
    img = _grain(_base((54, 56, 60)), 9, seed)
    d = ImageDraw.Draw(img)
    for x in range(16, SIZE, 40):
        for y in range(16, SIZE, 40):
            d.ellipse([x, y, x + 18, y + 18], fill=(72, 74, 80))
    return img


def pipe_metal(seed):
    img = brushed_metal(seed, (156, 158, 166))
    d = ImageDraw.Draw(img)
    for x in range(0, SIZE, 128):                                   # flanges
        d.rectangle([x, 0, x + 26, SIZE], fill=(120, 122, 130))
    d.rectangle([0, int(SIZE * 0.42), SIZE, int(SIZE * 0.5)], fill=(186, 140, 40))
    return img


# --------------------------------------------------------------------- signs
def sign(text, sub, bg, fg, seed, marks=0):
    img = _base(bg)
    d = ImageDraw.Draw(img)
    d.rectangle([10, 10, SIZE - 10, SIZE - 10], outline=fg, width=10)
    d.text((30, 40), text, fill=fg, font=_font(150 if len(text) <= 3 else 92))
    if sub:
        d.text((30, 250), sub, fill=fg, font=_font(56))
    rng = np.random.default_rng(seed)
    for i in range(marks):                                          # unique marker pattern
        x, y = int(rng.integers(40, SIZE - 110)), int(rng.integers(330, SIZE - 70))
        s = int(rng.integers(30, 60))
        if i % 2:
            d.ellipse([x, y, x + s, y + s], fill=fg)
        else:
            d.rectangle([x, y, x + s, y + s], fill=fg)
    return _grain(img, 5, seed)


def hazard_sign(seed):
    img = _base((240, 200, 40))
    d = ImageDraw.Draw(img)
    d.polygon([(SIZE // 2, 60), (SIZE - 60, SIZE - 80), (60, SIZE - 80)], outline=(30, 30, 30),
              width=18)
    d.text((SIZE // 2 - 26, 210), "!", fill=(30, 30, 30), font=_font(190))
    return _grain(img, 4, seed)


def grid_panel(seed):
    img = _base((228, 230, 236))
    d = ImageDraw.Draw(img)
    for i in range(0, SIZE + 1, 32):
        d.line([i, 0, i, SIZE], fill=(44, 48, 66), width=3)
        d.line([0, i, SIZE, i], fill=(44, 48, 66), width=3)
    for i in range(0, SIZE, 128):
        d.ellipse([i + 42, i + 42, i + 86, i + 86], fill=(196, 52, 44))
    return _grain(img, 5, seed)


def all_textures():
    """name -> PIL image, shared by the world and every prop model."""
    tex = {
        # walls: one identity per zone (the two long corridor legs share, on purpose)
        "wall_corridor": plant_wall((206, 208, 212), (64, 70, 82), 1, accent=(60, 96, 160)),
        "wall_machine": panel_wall((132, 138, 148), 2),
        "wall_lab": plant_wall((226, 232, 238), (52, 84, 120), 3, accent=(48, 130, 190),
                               label="LAB"),
        "wall_loading": plant_wall((224, 206, 150), (150, 110, 40), 4, accent=(232, 176, 40),
                                   label="LOADING"),
        "wall_testing": plant_wall((206, 226, 208), (48, 96, 64), 5, accent=(60, 160, 90),
                                   label="TEST"),
        "wall_storage": plant_wall((232, 200, 168), (150, 92, 40), 6, accent=(206, 112, 40),
                                   label="STORE"),
        "wall_workshop": plant_wall((192, 214, 216), (48, 92, 96), 7, accent=(40, 150, 160),
                                    label="WORKSHOP"),
        "wall_maintenance": panel_wall((156, 148, 140), 8),
        "wall_parts": plant_wall((214, 214, 218), (86, 86, 92), 9),
        "wall_equipment": mesh_wall((146, 150, 158), 10),
        # floors
        "floor_lane": lane_floor(11),
        "floor_checkerplate": checker_plate(12),
        "floor_esd": esd_floor(13),
        "floor_hazard": hazard_floor(14),
        "floor_epoxy_green": epoxy(15, (96, 132, 106)),
        "floor_concrete": concrete(16),
        "floor_concrete_oil": oily_concrete(17),
        "floor_steel_plate": steel_floor(18),
        "floor_tile_grey": tiles(19, (188, 188, 184), (158, 158, 154), n=6),
        # prop materials
        "metal_panel": painted_metal(31, (168, 172, 178)),
        "metal_brushed": brushed_metal(32),
        "machine_skin": machine_skin(33),
        "machine_skin_b": machine_skin(34, (96, 106, 92)),
        "control_face": control_face(35),
        "crate_wood": crate_wood(36),
        "pallet_wood": pallet_wood(37),
        "barrel_skin": barrel_skin(38),
        "shelf_bins": books_or_bins(39),
        "rubber_mat": rubber_mat(40),
        "pipe_metal": pipe_metal(41),
        "fence_mesh": mesh_wall((120, 126, 134), 42),
        "bench_top": painted_metal(43, (146, 120, 92)),
        "pillar_paint": plant_wall((228, 228, 232), (206, 160, 40), 44),
        "cable_black": _grain(_base((44, 44, 48)), 10, 45),
        # signage: numbered doors and zone signs
        "sign_core": sign("CORE", "EQUIPMENT", (232, 236, 244), (30, 52, 110), 51, 3),
        "sign_hazard": hazard_sign(52),
        "sign_exit": sign("EXIT", "-->", (232, 246, 234), (24, 118, 56), 53, 2),
        "sign_loading": sign("LOAD", "DOCK 2", (252, 240, 214), (176, 96, 20), 54, 3),
        "sign_lab": sign("LAB", "ROBOTICS", (234, 242, 250), (36, 86, 160), 55, 3),
        "sign_storage": sign("STORE", "AISLE 1-6", (250, 234, 214), (170, 88, 28), 56, 4),
        "sign_workshop": sign("SHOP", "MECHANICAL", (226, 244, 246), (28, 110, 118), 57, 4),
        "panel_grid": grid_panel(58),
    }
    for i in range(1, 9):                                           # numbered doors D1..D8
        tex[f"door_d{i}"] = sign(f"D{i}", ["LAB", "LOADING", "TESTING", "STORAGE", "WORKSHOP",
                                           "MAINT", "PARTS", "EQUIP"][i - 1],
                                 (248, 248, 250), (40, 44, 52), 60 + i, 2)
    return tex
