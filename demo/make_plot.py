#!/usr/bin/env python3
"""Plot the recorded forward-walk: forward distance vs time, from demo/walk_data.csv."""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

t, y = [], []
with open("demo/walk_data.csv") as f:
    for row in csv.DictReader(f):
        try:
            t.append(float(row["t_s"])); y.append(float(row["y_m"]))
        except (ValueError, KeyError):
            pass
t = np.array(t); y = np.array(y)
# forward = -Y (toward the face); forward distance travelled = y0 - y, in cm
fwd = (y[0] - y) * 100.0

# steady-state speed via linear fit after the ~2 s ramp
mask = t > 2.0
slope = np.polyfit(t[mask], fwd[mask], 1)[0]  # cm/s

plt.figure(figsize=(8, 4.5))
plt.plot(t, fwd, "o-", color="#2b8cbe", lw=2, ms=5, label="measured (Gazebo)")
tl = np.array([t[mask][0], t[-1]])
plt.plot(tl, slope * (tl - t[mask][0]) + fwd[mask][0], "--", color="#e34a33",
         lw=1.5, label=f"steady speed ≈ {slope:.1f} cm/s")
plt.title("Hexapod — Forward Walk (tripod gait, Gazebo)", fontweight="bold")
plt.xlabel("time (s)")
plt.ylabel("forward distance (cm)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("demo/forward_walk.png", dpi=130)
print(f"saved demo/forward_walk.png  (steady speed {slope:.2f} cm/s, total {fwd[-1]:.1f} cm)")
