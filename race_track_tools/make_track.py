#!/usr/bin/env python3
"""Generate a circular racing track of square gates at different heights, with ArUco tags.

Writes into --out: race_track/model.config and model.sdf (Gazebo model, visual only),
waypoints_world.csv (gate centres, Gazebo ENU) and waypoints_start_frame.csv (same gates in the
drone's start frame, the EKF frame after the switch to VIO), tags_world.csv, tags_start_frame.csv
and tags_png/ (printable tags). --spawn must match the iris spawn pose.
Gates face the travel direction (clockwise by default); heights are gate centres and cycle.

Tags (DICT_4X4_100) are geometry (white panel + black cells) on each frame corner, approach face.
The frame bar is widened to fit them (--bar); the opening stays --opening. Corner 0 = top-left, 1 = top-right, 2 = bottom-right, 3 = bottom-left, as seen by
the approaching drone. Tag id = 4 * gate + corner, so --tag-corners and --tag-gates never renumber.

Example:
  python3 make_track.py --radius 6 --center 5 5 --heights 1.5 2.5 3.5 2.5
  python3 make_track.py --tag-corners 0 1       # top corners only
  python3 make_track.py --model-dir             # write the model straight into the sim package
"""
import argparse
import csv
import math
import os

SIM_MODELS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "sitl_launcher", "models"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gates", type=int, default=8)
    ap.add_argument("--radius", type=float, default=6.0, help="circle radius [m]")
    ap.add_argument("--center", type=float, nargs=2, default=[5.0, 5.0], metavar=("X", "Y"), help="world [m]")
    ap.add_argument("--heights", type=float, nargs="+", default=[1.5, 2.5, 3.5, 2.5, 1.5, 2.5, 3.5, 2.5],
                    help="gate centre heights [m]")
    ap.add_argument("--opening", type=float, default=1.8, help="inner width and height of a gate [m]")
    ap.add_argument("--bar", type=float, default=None,
                    help="frame bar width [m] (default 0.12, or wide enough to carry the tags if there are any)")
    ap.add_argument("--first-angle", type=float, default=202.5, help="world angle of gate 0 around the centre [deg]")
    ap.add_argument("--ccw", action="store_true", help="counter-clockwise (default clockwise)")
    ap.add_argument("--spawn", type=float, nargs=3, default=[0.0, 0.0, 90.0], metavar=("X", "Y", "YAW_DEG"),
                    help="where the drone spawns in the world: x, y, heading [deg]")
    ap.add_argument("--no-tags", action="store_true", help="no ArUco tags")
    ap.add_argument("--tag-corners", type=int, nargs="+", default=[0, 1, 2, 3], choices=[0, 1, 2, 3],
                    help="corners that get a tag: 0 top-left, 1 top-right, 2 bottom-right, 3 bottom-left (approach view)")
    ap.add_argument("--tag-gates", type=int, nargs="+", default=None, help="gates that get tags (default all)")
    ap.add_argument("--tag-size", type=float, default=0.30, help="side of the black marker [m], white margin is 1/6 of it")
    ap.add_argument("--tag-gap", type=float, default=0.02, help="gap between frame and tag panel, only if the bar is narrower than the panel [m]")
    ap.add_argument("--model-dir", nargs="?", const=SIM_MODELS, default=None, metavar="DIR",
                    help=f"write the model into DIR/race_track/ (overwrites) instead of --out; without DIR: {SIM_MODELS}")
    ap.add_argument("--out", default="track_out")
    a = ap.parse_args()

    panel = a.tag_size * 8 / 6                       # marker plus a one-cell white margin
    t = a.bar if a.bar is not None else (0.12 if a.no_tags else round(panel + 0.05, 3))   # bar width
    n, W, d = a.gates, a.opening, 0.12               # d = depth along travel
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

    # tags, in the gate frame (x along travel, y left, z up)
    tag_gates = range(n) if a.tag_gates is None else a.tag_gates
    tags = []                                            # (id, gate, corner, cy, cz)
    ts = a.tag_size
    x_front = -d / 2 - 0.007                             # marker surface
    if not a.no_tags:
        try:
            import cv2
            tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        except (ImportError, AttributeError):
            raise SystemExit("ArUco tags need OpenCV with the aruco module: pip install opencv-contrib-python "
                             "(or pass --no-tags)")
        for g in tag_gates:
            if not 0 <= g < n:
                raise SystemExit(f"--tag-gates: no gate {g}")
            for cidx in sorted(set(a.tag_corners)):
                ky, kz = [(1, 1), (-1, 1), (-1, -1), (1, -1)][cidx]
                off = W / 2 + (t / 2 if t >= panel else t + a.tag_gap + panel / 2)   # on the frame corner, else outside
                tags.append((4 * g + cidx, g, cidx, ky * off, kz * off))
                zlow = gates[g][2] + kz * off - panel / 2
                if zlow < 0.05:
                    print(f"warning: tag {4 * g + cidx} (gate {g}, corner {cidx}) reaches z = {zlow:.2f} m, close to the ground")

    def flat(c):
        r, g_, b = c
        return (f"<material><ambient>{r} {g_} {b} 1</ambient><diffuse>{r} {g_} {b} 1</diffuse>"
                f"<specular>0 0 0 1</specular><emissive>{r} {g_} {b} 1</emissive></material>")

    def tag_visuals(gate, cy, cz, tid):
        bits = (cv2.aruco.generateImageMarker(tag_dict, tid, 6) == 0)      # 6x6 cells, True = black
        c = ts / 6
        out = [f'      <visual name="tag{tid}_panel"><pose>{-d / 2 - 0.003:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>'
               f'<geometry><box><size>0.006 {panel:.4f} {panel:.4f}</size></box></geometry>{flat((1, 1, 1))}</visual>']
        for r in range(6):
            col = 0
            while col < 6:
                if not bits[r, col]:
                    col += 1
                    continue
                end = col
                while end + 1 < 6 and bits[r, end + 1]:
                    end += 1
                u = ((col + end + 1) / 2 - 3) * c                          # offset to the viewer's right
                yy, zz = cy - u, cz - ((r + 0.5) - 3) * c                  # viewer's right is -y
                out.append(f'      <visual name="tag{tid}_r{r}_{col}"><pose>{-d / 2 - 0.007:.4f} {yy:.4f} {zz:.4f} 0 0 0</pose>'
                           f'<geometry><box><size>0.002 {(end - col + 1) * c:.4f} {c:.4f}</size></box></geometry>'
                           f'{flat((0, 0, 0))}</visual>')
                col = end + 1
        return out

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
        for (tid, g, cidx, cy, cz) in tags:
            if g == i:
                v += tag_visuals(i, cy, cz, tid)
        links.append(f'    <link name="gate_{i}">\n      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>\n'
                     + "\n".join(v) + "\n    </link>")

    sdf = ('<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="race_track">\n    <static>true</static>\n'
           + "\n".join(links) + "\n  </model>\n</sdf>\n")
    cfg = ('<?xml version="1.0"?>\n<model>\n  <name>race_track</name>\n  <version>1.0</version>\n'
           '  <sdf version="1.9">model.sdf</sdf>\n'
           f'  <description>Circle of {n} gates at different heights (visual only).</description>\n</model>\n')

    mdir = os.path.join(a.model_dir or a.out, "race_track")
    os.makedirs(mdir, exist_ok=True)
    os.makedirs(a.out, exist_ok=True)
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

    if tags:
        def to_world(g, ly, lz):
            gx, gy, gz, gyaw, _ = gates[g]
            return (gx - math.sin(gyaw) * ly + math.cos(gyaw) * (x_front), gy + math.cos(gyaw) * ly + math.sin(gyaw) * (x_front),
                    gz + lz)

        def to_start(p):
            dx, dy = p[0] - sx, p[1] - sy
            return (c * dx - s * dy, s * dx + c * dy, p[2])

        hdr = ["id", "gate", "corner", "size"] + ["cx", "cy", "cz"] + [f"{k}{j}" for j in range(4) for k in "xyz"]
        with open(os.path.join(a.out, "tags_world.csv"), "w", newline="") as fw, \
                open(os.path.join(a.out, "tags_start_frame.csv"), "w", newline="") as fs:
            ww, ws = csv.writer(fw), csv.writer(fs)
            ww.writerow(hdr)
            ws.writerow(hdr)
            for (tid, g, cidx, cy, cz) in tags:
                # corners in ArUco order: TL, TR, BR, BL
                loc = [(cy + ts / 2, cz + ts / 2), (cy - ts / 2, cz + ts / 2), (cy - ts / 2, cz - ts / 2), (cy + ts / 2, cz - ts / 2)]
                pw = [to_world(g, ly, lz) for ly, lz in loc]
                cw = to_world(g, cy, cz)
                for wr, pts, ctr in ((ww, pw, cw), (ws, [to_start(p) for p in pw], to_start(cw))):
                    wr.writerow([tid, g, cidx, f"{ts:.3f}"] + [f"{v:.4f}" for v in ctr] + [f"{v:.4f}" for p in pts for v in p])
        pdir = os.path.join(a.out, "tags_png")
        os.makedirs(pdir, exist_ok=True)
        for (tid, *_rest) in tags:
            img = cv2.aruco.generateImageMarker(tag_dict, tid, 6 * 40)
            img = cv2.copyMakeBorder(img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
            cv2.imwrite(os.path.join(pdir, f"tag_{tid}.png"), img)
        print(f"{len(tags)} ArUco tags (ids {min(t_[0] for t_ in tags)}..{max(t_[0] for t_ in tags)}), marker {ts * 100:.0f} cm, "
              f"panel {panel * 100:.0f} cm; printable PNGs in {pdir}")

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