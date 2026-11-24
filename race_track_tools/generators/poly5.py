"""Piecewise polynomials with fixed gate velocities.

One odd-degree polynomial per segment (default 5), position and velocity fixed at every gate
(speed x gate normal), higher derivatives 0, so the path is C^k continuous.
--speed-profile sets each gate's speed from path curvature, within --max-acc and --max-jerk.
"""
import math
from math import factorial

import numpy as np

HELP = "polynomials of fixed degree (default 5), position and velocity set at each gate"


def add_args(ap):
    g = ap.add_argument_group("poly5 options")
    g.add_argument("--degree", type=int, default=5, help="polynomial degree per segment, odd: 3, 5, 7, 9")
    g.add_argument("--speed-profile", action="store_true",
                   help="per-gate speeds from the track geometry and --max-acc (--speed is then ignored)")


def check_args(a):
    if a.degree < 3 or a.degree % 2 == 0 or a.degree > 9:
        raise SystemExit("--degree must be odd and between 3 and 9 (an even degree leaves a free coefficient "
                         "that needs an optimiser, e.g. min-snap; that is a later step)")


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


def sample_path(pts, degree, dt, time_scale, min_seg):
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
        _, _, _, acc, jerk, _, _ = sample_path(pts, degree, 0.02, time_scale, min_seg)
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


def generate(pts, a):
    a_design = None
    if a.speed_profile:
        a_design = fit_profile(pts, a.degree, a.time_scale, a.min_seg, a.max_acc, a.max_jerk, a.max_speed)
    t, p, v, acc, jerk, t_pts, segs = sample_path(pts, a.degree, a.dt, a.time_scale, a.min_seg)

    # check that the polynomials hit the boundary values (ill-conditioning guard)
    err = 0.0
    for i, (C, T) in enumerate(segs):
        for end, pt in ((0.0, pts[i]), (T, pts[i + 1])):
            r = eval_segment(C, T, a.degree, np.array([end]), orders=2)
            err = max(err, float(np.abs(r[0][0] - pt["p"]).max()), float(np.abs(r[1][0] - pt["v"]).max()))

    head = f"degree {a.degree} (C^{(a.degree - 1) // 2} continuous), "
    if a_design is None:
        info = [head + f"speed at gates {a.speed:.1f} m/s"]
    else:
        info = [head + f"speed PROFILE: design accel {a_design:.2f} m/s^2 -> peak accel <= {a.max_acc:.1f}, "
                       f"jerk <= {a.max_jerk:.0f}, cap {a.max_speed:.1f} m/s",
                "speed at gates [m/s]    : " + " ".join(f"{pt['label']}={np.linalg.norm(pt['v']):.1f}"
                                                      for pt in pts if "gate" in pt)]
    return dict(t=t, p=p, v=v, acc=acc, jerk=jerk, t_pts=t_pts, err=err, info=info)
