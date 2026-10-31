#!/usr/bin/env python3
"""Piecewise-polynomial trajectory through racing gates (numpy only, no ROS).

    python3 trajectory.py waypoints_start_frame.csv --degree 5 --speed 2.0 --laps 1

Input: CSV from make_track.py. Use the world-frame file for GPS, the start-frame file for VIO.
Path: start (at rest) -> gates x laps -> finish (at rest), one odd-degree polynomial per segment,
position and velocity fixed at every gate (speed x gate normal), so the path is C^k continuous.
--speed-profile sets each gate's speed from path curvature, within --max-acc and --max-jerk.
Checks gate crossings, clearance to frames, ground clearance and acceleration.
Outputs <out>.csv (t, pos, vel, acc, jerk, yaw) and <out>.png.
"""
import argparse
import csv
import math
import sys
from math import factorial

import numpy as np

GATE_DEPTH = 0.12             # gate depth along travel (as in make_track.py)
POLE_HALF = 0.03              # 0.06 m square pole under each gate (as in make_track.py)


# input
def load_gates(path, opening, bar):
    gates = []
    with open(path) as f:
        for r in csv.DictReader(f):
            yaw = math.radians(float(r["yaw_deg"]))
            gates.append(dict(
                id=int(r["gate"]),
                p=np.array([float(r["x"]), float(r["y"]), float(r["z"])]),
                yaw=yaw,
                n=np.array([math.cos(yaw), math.sin(yaw), 0.0]),
                W=float(r["opening"]) if r.get("opening") else opening,
                t=float(r["bar"]) if r.get("bar") else bar,
            ))
    if len(gates) < 1:
        raise SystemExit(f"no gates in {path}")
    return gates


# polynomial segments
def segment_coeffs(b0, b1, T, N):
    """Coefficients C[j] per axis of p(s) = sum_j C[j] s^j, s = t/T in [0,1].

    b0, b1: [k+1, 3] derivatives of order 0..k (real time) at the start and the end."""
    k = (N - 1) // 2
    A = np.zeros((N + 1, N + 1))
    rhs = np.zeros((N + 1, 3))
    r = 0
    for m in range(k + 1):
        A[r, m] = factorial(m)                                   # d^m/ds^m at s = 0 picks s^m
        rhs[r] = b0[m] * T ** m
        for j in range(m, N + 1):
            A[r + 1, j] = factorial(j) / factorial(j - m)        # at s = 1
        rhs[r + 1] = b1[m] * T ** m
        r += 2
    return np.linalg.solve(A, rhs)


def eval_segment(C, T, N, ts, orders=4):
    """Derivatives of order 0..orders-1 (real time) at times ts in [0, T]."""
    s = ts / T
    out = []
    for m in range(orders):
        val = np.zeros((len(ts), 3))
        for j in range(m, N + 1):
            val += np.outer(s ** (j - m), factorial(j) / factorial(j - m) * C[j])
        out.append(val / T ** m)
    return out


def build_points(gates, laps, speed, start, finish):
    pts = []
    if start is not None:
        pts.append(dict(p=np.array(start, float), v=np.zeros(3), label="start"))
    for lap in range(laps):
        for g in gates:
            pts.append(dict(p=g["p"], v=speed * g["n"], label=f"g{g['id']}", gate=g, lap=lap))
    if finish is not None:
        pts.append(dict(p=np.array(finish, float), v=np.zeros(3), label="finish"))
    if len(pts) < 2:
        raise SystemExit("need at least two points (a start or finish, or several gates)")
    return pts


def generate(pts, degree, dt, time_scale, min_seg):
    k = (degree - 1) // 2
    t_pts = [0.0]
    segs = []
    for a, b in zip(pts[:-1], pts[1:]):
        dist = float(np.linalg.norm(b["p"] - a["p"]))
        vmean = 0.5 * (np.linalg.norm(a["v"]) + np.linalg.norm(b["v"]))
        T = max(min_seg, time_scale * dist / max(vmean, 1e-6))
        b0 = np.zeros((k + 1, 3)); b1 = np.zeros((k + 1, 3))
        b0[0], b1[0] = a["p"], b["p"]
        if k >= 1:
            b0[1], b1[1] = a["v"], b["v"]
        C = segment_coeffs(b0, b1, T, degree)
        segs.append((C, T))
        t_pts.append(t_pts[-1] + T)

    cols = [[] for _ in range(4)]
    tt = []
    for i, (C, T) in enumerate(segs):
        last = i == len(segs) - 1
        ts = np.arange(0.0, T, dt)
        if last:
            ts = np.append(ts, T)
        res = eval_segment(C, T, degree, ts)
        for c, r in zip(cols, res):
            c.append(r)
        tt.append(t_pts[i] + ts)
    t = np.concatenate(tt)
    p, v, a, j = (np.vstack(c) for c in cols)
    return t, p, v, a, j, np.array(t_pts), segs


# speed profile
def _angle(u, w):
    u = u / max(np.linalg.norm(u), 1e-9)
    w = w / max(np.linalg.norm(w), 1e-9)
    return float(math.acos(max(-1.0, min(1.0, float(u @ w)))))


def profile_speeds(pts, a_design, vmax):
    """Speed per point: 0 at start / finish, curvature-limited at gates, then braking and acceleration passes."""
    n = len(pts)
    head = [pt["gate"]["n"] if "gate" in pt else None for pt in pts]
    L = [float(np.linalg.norm(pts[i + 1]["p"] - pts[i]["p"])) for i in range(n - 1)]
    chord = [(pts[i + 1]["p"] - pts[i]["p"]) for i in range(n - 1)]

    def kappa(i):          # heading change along segment i -> i+1, per metre
        ang = 0.0
        if head[i] is not None:
            ang += _angle(head[i], chord[i])
        if head[i + 1] is not None:
            ang += _angle(chord[i], head[i + 1])
        return ang / max(L[i], 1e-6)

    kap = [kappa(i) for i in range(n - 1)]
    v = np.zeros(n)
    for i, pt in enumerate(pts):
        if "gate" not in pt:
            continue                                   # start / finish: at rest
        k = max(kap[i - 1] if i > 0 else 0.0, kap[i] if i < n - 1 else 0.0, 1e-6)
        v[i] = min(vmax, math.sqrt(a_design / k))
    for i in range(n - 1):                             # forward: can only speed up so fast
        v[i + 1] = min(v[i + 1], math.sqrt(v[i] ** 2 + 2 * a_design * L[i]))
    for i in range(n - 2, -1, -1):                     # backward: must be able to brake in time
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_design * L[i]))
    return v


def apply_profile(pts, v):
    for pt, s in zip(pts, v):
        pt["v"] = s * pt["gate"]["n"] if "gate" in pt else np.zeros(3)
    return pts


def fit_profile(pts, degree, time_scale, min_seg, max_acc, max_jerk, vmax):
    """Largest design acceleration whose polynomial path stays under max_acc and max_jerk."""
    def ok(ad):
        apply_profile(pts, profile_speeds(pts, ad, vmax))
        _, _, _, acc, jerk, _, _ = generate(pts, degree, 0.02, time_scale, min_seg)
        return (float(np.linalg.norm(acc, axis=1).max()) <= max_acc
                and float(np.linalg.norm(jerk, axis=1).max()) <= max_jerk)
    lo, hi = 0.02, max(max_acc * 3.0, 1.0)
    if not ok(lo):
        raise SystemExit(f"even a very gentle profile exceeds --max-acc {max_acc} / --max-jerk {max_jerk}: raise them, "
                         f"or the gates are too close together for this degree / start and finish at rest")
    if ok(hi):
        lo = hi                                        # capped by --max-speed, not by acceleration or jerk
    else:
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            if ok(mid):
                lo = mid
            else:
                hi = mid
    apply_profile(pts, profile_speeds(pts, lo, vmax))
    return lo


# checks using the gate size
def box_dist(pl, c, h):
    q = np.abs(pl - np.asarray(c)) - np.asarray(h)
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)


def gate_local(g, p):
    d = p - g["p"]
    n, l = g["n"], np.array([-math.sin(g["yaw"]), math.cos(g["yaw"]), 0.0])
    return np.stack([d @ n, d @ l, d[:, 2]], axis=1)


def clearance(gates, p):
    """Per sample: distance to the nearest gate frame bar or pole, over all gates."""
    best = np.full(len(p), np.inf)
    who = np.zeros(len(p), int)
    for gi, g in enumerate(gates):
        W, t = g["W"], g["t"]
        d = GATE_DEPTH
        zb = g["p"][2] - W / 2 - t
        solids = [
            ((0, 0, W / 2 + t / 2), (d / 2, (W + 2 * t) / 2, t / 2)),      # top
            ((0, 0, -(W / 2 + t / 2)), (d / 2, (W + 2 * t) / 2, t / 2)),   # bottom
            ((0, W / 2 + t / 2, 0), (d / 2, t / 2, W / 2)),                # left
            ((0, -(W / 2 + t / 2), 0), (d / 2, t / 2, W / 2)),             # right
            ((0, 0, zb / 2 - g["p"][2]), (POLE_HALF, POLE_HALF, max(zb, 0.0) / 2)),  # pole
        ]
        pl = gate_local(g, p)
        for c, h in solids:
            dist = box_dist(pl, c, h)
            m = dist < best
            best[m], who[m] = dist[m], gi
    return best, who


def gate_crossings(gates, p, radius, window=4.0):
    """Crossings of a gate plane near the gate: gate index, forward?, lateral, vertical, inside the opening?"""
    out = []
    for gi, g in enumerate(gates):
        pl = gate_local(g, p)
        x = pl[:, 0]
        idx = np.where((x[:-1] < 0) & (x[1:] >= 0) | (x[:-1] >= 0) & (x[1:] < 0))[0]
        if abs(x[0]) < 1e-9 and x[1] != x[0]:      # path starts exactly on the plane (--no-start): count it
            half = g["W"] / 2 - radius
            out.append(dict(gate=gi, forward=bool(x[1] > x[0]), lat=float(pl[0, 1]), vert=float(pl[0, 2]),
                            ok=bool(abs(pl[0, 1]) <= half and abs(pl[0, 2]) <= half)))
        for i in idx:
            w = x[i] / (x[i] - x[i + 1])
            y = pl[i, 1] + w * (pl[i + 1, 1] - pl[i, 1])
            z = pl[i, 2] + w * (pl[i + 1, 2] - pl[i, 2])
            if abs(y) > window or abs(z) > window:
                continue                      # far from the gate: irrelevant
            half = g["W"] / 2 - radius
            out.append(dict(gate=gi, forward=bool(x[i + 1] > x[i]), lat=float(y), vert=float(z),
                            ok=bool(abs(y) <= half and abs(z) <= half)))
    return out


# main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("gates_csv")
    ap.add_argument("--degree", type=int, default=5, help="polynomial degree per segment, odd: 3, 5, 7, 9")
    ap.add_argument("--speed", type=float, default=2.0, help="speed through each gate [m/s]")
    ap.add_argument("--speed-profile", action="store_true",
                    help="per-gate speeds from the track geometry and --max-acc (--speed is then ignored)")
    ap.add_argument("--max-speed", type=float, default=6.0, help="with --speed-profile: cap on the gate speed [m/s]")
    ap.add_argument("--max-jerk", type=float, default=15.0,
                    help="peak jerk [m/s^3]. ArduPilot's position controller smooths its setpoint with PSC_JERK_XY / "
                         "PSC_JERK_Z (default 5, max 20); a path with more jerk than that is followed late and overshoots")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--start", type=float, nargs=3, default=[0.0, 0.0, 3.0], metavar=("X", "Y", "Z"),
                    help="where the drone hovers before the run (same frame as the CSV)")
    ap.add_argument("--finish", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="where it stops after the last gate (default: the start point)")
    ap.add_argument("--no-start", action="store_true", help="begin at gate 0 flying at --speed")
    ap.add_argument("--no-finish", action="store_true", help="end at the last gate flying at --speed")
    ap.add_argument("--time-scale", type=float, default=1.0, help=">1 = slower segments")
    ap.add_argument("--min-seg", type=float, default=0.5, help="shortest allowed segment duration [s]")
    ap.add_argument("--dt", type=float, default=0.01, help="output sample period [s]")
    ap.add_argument("--start-yaw", type=float, default=0.0, help="heading while at rest at the start [deg]")
    ap.add_argument("--opening", type=float, default=1.8, help="gate opening if the CSV has no column")
    ap.add_argument("--bar", type=float, default=0.12, help="gate bar thickness if the CSV has no column")
    ap.add_argument("--drone-radius", type=float, default=0.30, help="margin used by the checks [m]")
    ap.add_argument("--min-z", type=float, default=0.5, help="lowest allowed height of the drone centre [m]")
    ap.add_argument("--max-acc", type=float, default=5.0, help="warn above this acceleration [m/s^2]")
    ap.add_argument("--out", default="trajectory")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    if a.degree < 3 or a.degree % 2 == 0 or a.degree > 9:
        raise SystemExit("--degree must be odd and between 3 and 9 (an even degree leaves a free coefficient "
                         "that needs an optimiser, e.g. min-snap; that is a later step)")

    gates = load_gates(a.gates_csv, a.opening, a.bar)
    start = None if a.no_start else a.start
    finish = None if a.no_finish else (a.finish if a.finish is not None else a.start)
    pts = build_points(gates, a.laps, a.speed, start, finish)
    a_design = None
    if a.speed_profile:
        a_design = fit_profile(pts, a.degree, a.time_scale, a.min_seg, a.max_acc, a.max_jerk, a.max_speed)
    t, p, v, acc, jerk, t_pts, segs = generate(pts, a.degree, a.dt, a.time_scale, a.min_seg)

    # check that the polynomials hit the boundary values (ill-conditioning guard)
    err = 0.0
    for i, (C, T) in enumerate(segs):
        for end, pt in ((0.0, pts[i]), (T, pts[i + 1])):
            r = eval_segment(C, T, a.degree, np.array([end]), orders=2)
            err = max(err, float(np.abs(r[0][0] - pt["p"]).max()), float(np.abs(r[1][0] - pt["v"]).max()))

    speed = np.linalg.norm(v, axis=1)
    amag = np.linalg.norm(acc, axis=1)
    jmag = np.linalg.norm(jerk, axis=1)

    yaw = np.zeros(len(t))
    held = math.radians(a.start_yaw)
    for i in range(len(t)):
        if speed[i] > 0.3:
            held = math.atan2(v[i, 1], v[i, 0])
        yaw[i] = held
    yaw = np.unwrap(yaw)

    clr, who = clearance(gates, p)
    clr_net = clr - a.drone_radius
    cross = gate_crossings(gates, p, a.drone_radius)

    gate_pts = [i for i, q in enumerate(pts) if "gate" in q]
    expected = {}
    for q in pts:
        if "gate" in q:
            expected[q["gate"]["id"]] = expected.get(q["gate"]["id"], 0) + 1
    problems = []
    for gi, g in enumerate(gates):
        mine = [c for c in cross if c["gate"] == gi]
        good = [c for c in mine if c["forward"] and c["ok"]]
        bad = [c for c in mine if not (c["forward"] and c["ok"])]
        if len(good) != expected.get(g["id"], 0) or bad:
            problems.append(f"gate {g['id']}: {len(good)} good crossings (expected {expected.get(g['id'], 0)})"
                            + "".join(f", BAD crossing lat={c['lat']:+.2f} vert={c['vert']:+.2f} "
                                      f"{'forward' if c['forward'] else 'BACKWARD'}" for c in bad))
    if clr_net.min() < 0:
        problems.append(f"too close to a gate frame/pole: clearance {clr_net.min():+.2f} m (after drone radius) "
                        f"near gate {gates[who[int(np.argmin(clr_net))]]['id']} at t={t[int(np.argmin(clr_net))]:.2f} s")
    if p[:, 2].min() < a.min_z:
        problems.append(f"flies below min height {a.min_z:.2f} m: lowest z = {p[:, 2].min():.2f} m")
    if amag.max() > a.max_acc:
        problems.append(f"acceleration {amag.max():.1f} m/s^2 exceeds --max-acc {a.max_acc:.1f}")
    if jmag.max() > a.max_jerk:
        problems.append(f"jerk {jmag.max():.1f} m/s^3 exceeds --max-jerk {a.max_jerk:.0f}: ArduPilot (PSC_JERK_XY/Z) "
                        f"will follow this late and overshoot")

    print("\n================ trajectory ================")
    if a_design is None:
        print(f"gates {len(gates)} x {a.laps} lap(s), degree {a.degree} (C^{(a.degree - 1) // 2} continuous), "
              f"speed at gates {a.speed:.1f} m/s")
    else:
        print(f"gates {len(gates)} x {a.laps} lap(s), degree {a.degree} (C^{(a.degree - 1) // 2} continuous), "
              f"speed PROFILE: design accel {a_design:.2f} m/s^2 -> peak accel <= {a.max_acc:.1f}, jerk <= {a.max_jerk:.0f}, cap {a.max_speed:.1f} m/s")
        print("speed at gates [m/s]    : " + " ".join(f"{pt['label']}={np.linalg.norm(pt['v']):.1f}"
                                                      for pt in pts if "gate" in pt))
    print(f"duration                : {t[-1]:.1f} s    path length {np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)):.1f} m")
    print(f"speed  max / mean       : {speed.max():.2f} / {speed.mean():.2f} m/s")
    print(f"accel  max / mean       : {amag.max():.2f} / {amag.mean():.2f} m/s^2")
    print(f"jerk   max              : {jmag.max():.1f} m/s^3")
    print(f"lowest z                : {p[:, 2].min():.2f} m")
    print(f"min clearance to frames : {clr.min():.2f} m  ({clr_net.min():+.2f} m after the {a.drone_radius:.2f} m drone radius)")
    print(f"boundary fit error      : {err:.1e}")
    if problems:
        print("PROBLEMS:")
        for s in problems:
            print("  -", s)
    else:
        print("checks                  : all gates crossed forwards through the opening, no frame/ground/accel problems")
    print("============================================\n")

    arr = np.column_stack([t, p, v, acc, jerk, yaw])
    np.savetxt(a.out + ".csv", arr, delimiter=",", fmt="%.5f",
               header="t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw", comments="")
    print("trajectory written to", a.out + ".csv")

    if a.no_plot:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
    except ImportError:
        print("matplotlib missing, skipping the plot")
        return

    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    seg_xy = np.stack([p[:-1, :2], p[1:, :2]], axis=1)
    lc = LineCollection(seg_xy, array=speed[:-1], cmap="viridis", lw=2.5)
    ax[0, 0].add_collection(lc)
    for g in gates:
        lat = np.array([-math.sin(g["yaw"]), math.cos(g["yaw"])])
        c = g["p"][:2]
        ax[0, 0].plot(*zip(c - lat * g["W"] / 2, c + lat * g["W"] / 2), color="tab:orange", lw=5, solid_capstyle="butt")
        ax[0, 0].annotate(str(g["id"]), c, textcoords="offset points", xytext=(6, 6), fontsize=9)
    ax[0, 0].plot(*pts[0]["p"][:2], "ks")
    ax[0, 0].autoscale()
    ax[0, 0].set_aspect("equal", adjustable="datalim")
    fig.colorbar(lc, ax=ax[0, 0], label="speed [m/s]")
    ax[0, 0].set_title("top-down (orange = gate openings)")
    ax[0, 0].set_xlabel("x [m]"); ax[0, 0].set_ylabel("y [m]"); ax[0, 0].grid(alpha=0.3)

    ax[0, 1].plot(t, p[:, 2], label="trajectory")
    for i in gate_pts:
        ax[0, 1].plot(t_pts[i], pts[i]["p"][2], "o", color="tab:orange")
        ax[0, 1].annotate(pts[i]["label"], (t_pts[i], pts[i]["p"][2]), textcoords="offset points", xytext=(4, 6), fontsize=7)
    ax[0, 1].axhline(a.min_z, color="r", ls=":", label=f"min z {a.min_z} m")
    ax[0, 1].set_title("height, gate centres marked"); ax[0, 1].set_xlabel("t [s]"); ax[0, 1].set_ylabel("z [m]")
    ax[0, 1].legend(); ax[0, 1].grid(alpha=0.3)

    ax[1, 0].plot(t, speed, label="speed [m/s]")
    ax[1, 0].plot(t, amag, label="|accel| [m/s^2]")
    ax[1, 0].axhline(a.max_acc, color="r", ls=":", label="--max-acc")
    for i in gate_pts:
        ax[1, 0].axvline(t_pts[i], color="0.85", lw=0.8)
    ax[1, 0].set_title("speed and acceleration (grey lines = gates)"); ax[1, 0].set_xlabel("t [s]")
    ax[1, 0].legend(); ax[1, 0].grid(alpha=0.3)

    ax[1, 1].plot(t, clr, label="distance to nearest frame / pole")
    ax[1, 1].axhline(a.drone_radius, color="r", ls=":", label=f"drone radius {a.drone_radius} m")
    ax[1, 1].set_title("clearance to gate structures"); ax[1, 1].set_xlabel("t [s]"); ax[1, 1].set_ylabel("[m]")
    ax[1, 1].legend(); ax[1, 1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(a.out + ".png", dpi=110)
    print("plot written to", a.out + ".png")


if __name__ == "__main__":
    sys.exit(main())