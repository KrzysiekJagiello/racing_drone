#!/usr/bin/env python3
"""Generate a circular racing track of square gates at different heights.

Writes into --out: race_track/model.config and model.sdf (Gazebo model, visual only),
waypoints_world.csv (gate centres, Gazebo ENU) and waypoints_start_frame.csv (same gates in the
drone's start frame, the EKF frame after the switch to VIO). --spawn must match the iris spawn pose.
Gates face the travel direction (clockwise by default); heights are gate centres and cycle.

Example: python3 make_track.py --radius 6 --center 5 5 --heights 1.5 2.5 3.5 2.5
"""
import argparse
import csv
import math
import os


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gates", type=int, default=8)
    ap.add_argument("--radius", type=float, default=6.0, help="circle radius [m]")
    ap.add_argument("--center", type=float, nargs=2, default=[5.0, 5.0], metavar=("X", "Y"), help="world [m]")
    ap.add_argument("--heights", type=float, nargs="+", default=[1.5, 2.5, 3.5, 2.5, 1.5, 2.5, 3.5, 2.5],
                    help="gate centre heights [m]")
    ap.add_argument("--opening", type=float, default=1.8, help="inner width and height of a gate [m]")
    ap.add_argument("--first-angle", type=float, default=202.5, help="world angle of gate 0 around the centre [deg]")
    ap.add_argument("--ccw", action="store_true", help="counter-clockwise (default clockwise)")
    ap.add_argument("--spawn", type=float, nargs=3, default=[0.0, 0.0, 90.0], metavar=("X", "Y", "YAW_DEG"),
                    help="where the drone spawns in the world: x, y, heading [deg]")
    ap.add_argument("--out", default="track_out")
    a = ap.parse_args()

    n, W, t, d = a.gates, a.opening, 0.12, 0.12      # t = bar thickness, d = depth along travel
    sgn = 1.0 if a.ccw else -1.0
    cx, cy = a.center
    sx, sy, syaw = a.spawn

    gates = []
    for k in range(n):
        th = math.radians(a.first_angle) + sgn * k * 2 * math.pi / n
        x, y = cx + a.radius * math.cos(th), cy + a.radius * math.sin(th)
        z = a.heights[k % len(a.heights)]
        yaw = th + sgn * math.pi / 2                  # travel direction = gate normal
        zb = z - W / 2 - t                            # underside of the bottom bar
        if zb <= 0.05:
            raise SystemExit(f"gate {k}: height {z} m puts the frame on the ground, use >= {W / 2 + t + 0.1:.2f} m")
        gates.append((x, y, z, yaw, zb))
        if not (-4 <= x <= 14 and -4 <= y <= 14):
            print(f"warning: gate {k} at ({x:.1f}, {y:.1f}) is outside x,y in [-4, 14], where the vio_scene props are kept clear")

    def mat(c):
        r, g, b = c
        return (f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
                f"<specular>0.1 0.1 0.1 1</specular></material>")

    def visual(name, pose, size, color):
        return (f'      <visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
                f'{mat(color)}</visual>')

    links = []
    for i, (x, y, z, yaw, zb) in enumerate(gates):
        col = (0.10, 0.80, 0.20) if i == 0 else (1.0, 0.45, 0.0)   # gate 0 (start) is green
        pole = (0.35, 0.35, 0.35)
        v = [
            visual("top",    f"0 0 {W / 2 + t / 2:.4f} 0 0 0", f"{d} {W + 2 * t:.4f} {t}", col),
            visual("bottom", f"0 0 {-(W / 2 + t / 2):.4f} 0 0 0", f"{d} {W + 2 * t:.4f} {t}", col),
            visual("left",   f"0 {W / 2 + t / 2:.4f} 0 0 0 0", f"{d} {t} {W}", col),
            visual("right",  f"0 {-(W / 2 + t / 2):.4f} 0 0 0 0", f"{d} {t} {W}", col),
            visual("pole",   f"0 0 {zb / 2 - z:.4f} 0 0 0", f"0.06 0.06 {zb:.4f}", pole),
        ]
        links.append(f'    <link name="gate_{i}">\n      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>\n'
                     + "\n".join(v) + "\n    </link>")

    sdf = ('<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="race_track">\n    <static>true</static>\n'
           + "\n".join(links) + "\n  </model>\n</sdf>\n")
    cfg = ('<?xml version="1.0"?>\n<model>\n  <name>race_track</name>\n  <version>1.0</version>\n'
           '  <sdf version="1.9">model.sdf</sdf>\n'
           f'  <description>Circle of {n} gates at different heights (visual only).</description>\n</model>\n')

    mdir = os.path.join(a.out, "race_track")
    os.makedirs(mdir, exist_ok=True)
    with open(os.path.join(mdir, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(mdir, "model.config"), "w") as f:
        f.write(cfg)

    c, s = math.cos(math.radians(-syaw)), math.sin(math.radians(-syaw))
    with open(os.path.join(a.out, "waypoints_world.csv"), "w", newline="") as fw, \
            open(os.path.join(a.out, "waypoints_start_frame.csv"), "w", newline="") as fs:
        ww, ws = csv.writer(fw), csv.writer(fs)
        ww.writerow(["gate", "x", "y", "z", "yaw_deg", "opening", "bar"])
        ws.writerow(["gate", "x", "y", "z", "yaw_deg", "opening", "bar"])
        for i, (x, y, z, yaw, _) in enumerate(gates):
            ww.writerow([i, f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{math.degrees(yaw) % 360:.1f}", f"{W:.3f}", f"{t:.3f}"])
            dx, dy = x - sx, y - sy
            ws.writerow([i, f"{c * dx - s * dy:.3f}", f"{s * dx + c * dy:.3f}", f"{z:.3f}",
                         f"{(math.degrees(yaw) - syaw) % 360:.1f}", f"{W:.3f}", f"{t:.3f}"])

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={"width_ratios": [1.3, 1]})
        th = [i * 2 * math.pi / 200 for i in range(201)]
        ax[0].plot([cx + a.radius * math.cos(u) for u in th], [cy + a.radius * math.sin(u) for u in th],
                   color="0.8", lw=1)
        for i, (x, y, z, yaw, _) in enumerate(gates):
            nx, ny = -math.sin(yaw), math.cos(yaw)       # along the gate plane
            ax[0].plot([x - nx * W / 2, x + nx * W / 2], [y - ny * W / 2, y + ny * W / 2],
                       color="tab:green" if i == 0 else "tab:orange", lw=4, solid_capstyle="butt")
            ax[0].annotate("", xy=(x + math.cos(yaw) * 1.1, y + math.sin(yaw) * 1.1), xytext=(x, y),
                           arrowprops=dict(arrowstyle="->", color="0.3"))
            ax[0].annotate(f"{i}  z={z:.1f}", (x, y), textcoords="offset points", xytext=(6, 6), fontsize=8)
        ax[0].plot(sx, sy, "ks")
        ax[0].annotate("", xy=(sx + math.cos(math.radians(syaw)) * 1.5, sy + math.sin(math.radians(syaw)) * 1.5),
                       xytext=(sx, sy), arrowprops=dict(arrowstyle="-|>", color="k"))
        ax[0].annotate("drone start", (sx, sy), textcoords="offset points", xytext=(6, -12), fontsize=8)
        ax[0].set_aspect("equal")
        ax[0].set_title("top-down, world frame (arrows = travel direction)")
        ax[0].set_xlabel("x [m]")
        ax[0].set_ylabel("y [m]")
        ax[0].grid(alpha=0.3)
        ax[1].bar(range(n), [g[2] for g in gates],
                  color=["tab:green"] + ["tab:orange"] * (n - 1))
        ax[1].set_title("gate centre height")
        ax[1].set_xlabel("gate")
        ax[1].set_ylabel("z [m]")
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, "preview.png"), dpi=110)
    except ImportError:
        print("matplotlib missing, skipping preview.png")

    lap = sum(math.dist(gates[i][:3], gates[(i + 1) % n][:3]) for i in range(n))
    print(f"{n} gates, lap length about {lap:.1f} m, written to {a.out}/")


if __name__ == "__main__":
    main()