#!/usr/bin/env python3
"""Make a constant-height copy of a trajectory CSV (diagnostic: does the lateral error depend on the vertical motion?).

    python3 flatten_z.py traj.csv traj_flat.csv [z]

z defaults to the first point's height. Columns: t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw. The horizontal path, speed
and timing stay the same; z, vz, az, jz are replaced. Gates are then not on the path, so the gate score is meaningless."""
import sys
import numpy as np

src, dst = sys.argv[1], sys.argv[2]
with open(src) as f:
    header = f.readline().strip()
d = np.loadtxt(src, delimiter=",", skiprows=1)
z = float(sys.argv[3]) if len(sys.argv) > 3 else float(d[0, 3])
d[:, 3] = z
d[:, 6] = d[:, 9] = d[:, 12] = 0.0
np.savetxt(dst, d, delimiter=",", header=header, comments="", fmt="%.6f")
print(f"{dst}: z = {z:.2f} m, {len(d)} rows, {d[-1, 0]:.1f} s")