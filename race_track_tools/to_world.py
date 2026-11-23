#!/usr/bin/env python3
"""Convert a trajectory CSV from the start frame to the world frame (for flying on GPS, without --vio).

    python3 to_world.py race_track_tools/traj_prof_7_10_laps.csv race_track_tools/traj_prof_7_10_laps_world.csv

The start frame has its origin at the spawn point and its x axis along the spawn heading (make_track.py --spawn).
Default spawn = make_track.py default: x 0, y 0, heading 90 deg. Position is rotated and shifted, velocity,
acceleration and jerk are only rotated, z is unchanged, and the heading (yaw, radians) is rotated by the spawn heading.
"""
import argparse
import math

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("src")
ap.add_argument("dst")
ap.add_argument("--spawn", type=float, nargs=3, default=[0.0, 0.0, 90.0], metavar=("X", "Y", "YAW_DEG"),
                help="spawn pose used in make_track.py (default 0 0 90)")
a = ap.parse_args()

with open(a.src) as f:
    header = f.readline().strip()
d = np.loadtxt(a.src, delimiter=",", skiprows=1)
if d.shape[1] != 14 or header.split(",")[:4] != ["t", "x", "y", "z"]:
    raise SystemExit("expected t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw")
sx, sy, syaw = a.spawn
c, s = math.cos(math.radians(syaw)), math.sin(math.radians(syaw))
R = np.array([[c, -s], [s, c]])
o = d.copy()
o[:, 1:3] = d[:, 1:3] @ R.T + [sx, sy]
for k in (4, 7, 10):                       # velocity, acceleration, jerk: rotation only
    o[:, k:k + 2] = d[:, k:k + 2] @ R.T
yaw = d[:, 13] + math.radians(syaw)
o[:, 13] = (yaw + math.pi) % (2 * math.pi) - math.pi
np.savetxt(a.dst, o, delimiter=",", fmt="%.5f", header=header, comments="")
print(f"{a.src} -> {a.dst}: spawn {sx} {sy} {syaw} deg; first point {np.round(o[0, 1:4], 3).tolist()}, "
      f"last point {np.round(o[-1, 1:4], 3).tolist()}")