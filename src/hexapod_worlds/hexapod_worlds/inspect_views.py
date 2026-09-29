#!/usr/bin/env python3
"""Check the facility through the robot's eyes, before any SLAM is wired up.

Puts a probe camera at the hexapod's camera height (0.14 m) at a viewpoint in
every zone, grabs a frame, and reports what a visual SLAM front end would get:

  * ORB keypoints per view - too few means visual odometry will lose tracking
  * how distinct the views are from each other (descriptor matching), so we know
    places are recognisable, and we can see the deliberate corridor look-alikes
  * a contact sheet of every view, to eyeball the world from robot height

Needs the facility running (./run_facility.sh, GUI or headless).

    ros2 run hexapod_worlds inspect_facility_views
"""
import os
import sys
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode

from hexapod_worlds import layout as L
from hexapod_worlds.textures import FONT_PATHS

WORLD = "hexapod_facility"
PROBE = "view_probe"
TOPIC = "/view_probe/image"
CAM_HEIGHT = 0.14          # the hexapod's camera height above the floor
MIN_FEATURES = 150         # below this a frame is poor for visual odometry


def probe_sdf(x, y, yaw):
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{PROBE}">
    <static>true</static>
    <pose>{x} {y} {CAM_HEIGHT} 0 0 {yaw}</pose>
    <link name="link">
      <sensor name="probe" type="camera">
        <always_on>1</always_on><update_rate>10</update_rate>
        <topic>{TOPIC.lstrip('/')}</topic>
        <camera>
          <horizontal_fov>1.0472</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>25.0</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""


def call(gz, service, req, req_type, timeout=8000):
    ok, rep = gz.request(service, req, req_type, Boolean, timeout)
    return ok and rep.data


def main():
    gz = GzNode()
    frames = {"latest": None}
    gz.subscribe(GzImage, TOPIC, lambda m: frames.__setitem__("latest", m))

    call(gz, f"/world/{WORLD}/remove/blocking", Entity(name=PROBE, type=Entity.MODEL), Entity)
    x0, y0, yaw0 = L.VIEWPOINTS[0][1:]
    if not call(gz, f"/world/{WORLD}/create/blocking",
                EntityFactory(sdf=probe_sdf(x0, y0, yaw0)), EntityFactory):
        print("could not add the probe camera - is the facility running?")
        return 1

    orb = cv2.ORB_create(1500)
    results, images = [], []
    try:
        for label, x, y, yaw in L.VIEWPOINTS:
            pose = Pose(name=PROBE)
            pose.position.x, pose.position.y, pose.position.z = x, y, CAM_HEIGHT
            pose.orientation.z, pose.orientation.w = np.sin(yaw / 2), np.cos(yaw / 2)
            call(gz, f"/world/{WORLD}/set_pose", pose, Pose)
            frames["latest"] = None
            t0 = time.time()
            while frames["latest"] is None and time.time() - t0 < 5:
                time.sleep(0.05)
            time.sleep(0.4)                       # let a frame after the move arrive
            m = frames["latest"]
            if m is None:
                print(f"  {label}: no frame")
                continue
            rgb = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
            grey = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            kp, desc = orb.detectAndCompute(grey, None)
            results.append(dict(label=label, n=len(kp), desc=desc,
                                brightness=float(grey.mean()),
                                contrast=float(grey.std())))
            images.append(rgb.copy())
    finally:
        call(gz, f"/world/{WORLD}/remove/blocking", Entity(name=PROBE, type=Entity.MODEL), Entity)

    if not results:
        print("no views captured")
        return 1

    # how similar is each pair of views? (matched ORB descriptors, as a fraction)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    n = len(results)
    sim = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            di, dj = results[i]["desc"], results[j]["desc"]
            if di is None or dj is None:
                continue
            good = [m for m in matcher.match(di, dj) if m.distance < 40]
            sim[i, j] = sim[j, i] = len(good) / max(1, min(len(di), len(dj)))

    print(f"{'view':16s} {'features':>9s} {'bright':>7s} {'contrast':>9s}   closest other view")
    poor = []
    for i, r in enumerate(results):
        j = int(np.argmax(sim[i]))
        flag = "" if r["n"] >= MIN_FEATURES else "  <-- POOR"
        if r["n"] < MIN_FEATURES:
            poor.append(r["label"])
        print(f"{r['label']:16s} {r['n']:9d} {r['brightness']:7.0f} {r['contrast']:9.1f}   "
              f"{results[j]['label']} ({sim[i, j] * 100:.0f}%){flag}")

    # contact sheet
    cols, rows = 3, (len(images) + 2) // 3
    tw, th = 320, 240
    sheet = Image.new("RGB", (cols * tw, rows * th), "white")
    try:
        font = ImageFont.truetype(FONT_PATHS[0], 16)
    except OSError:
        font = ImageFont.load_default()
    for k, (img, r) in enumerate(zip(images, results)):
        tile = Image.fromarray(img).resize((tw, th))
        d = ImageDraw.Draw(tile)
        d.rectangle([0, 0, tw, 22], fill=(0, 0, 0))
        d.text((6, 3), f"{r['label']} - {r['n']} features", fill=(255, 255, 255), font=font)
        sheet.paste(tile, ((k % cols) * tw, (k // cols) * th))
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "facility_views.png")
    sheet.save(path)

    worst = min(r["n"] for r in results)
    aliased = [(results[i]["label"], results[j]["label"], sim[i, j])
               for i in range(n) for j in range(i + 1, n) if sim[i, j] > 0.25]
    print(f"\nviews: {n}, fewest features: {worst} (want >= {MIN_FEATURES})")
    print("look-alike pairs (>25% matched): "
          + (", ".join(f"{a}~{b} {s * 100:.0f}%" for a, b, s in aliased) or "none"))
    print(f"contact sheet: {path}")
    print(f"RESULT: {len(poor)} view(s) below the feature threshold"
          + (f": {', '.join(poor)}" if poor else ""))
    return 1 if poor else 0


if __name__ == "__main__":
    sys.exit(main())
