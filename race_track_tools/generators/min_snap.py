"""Minimum-snap piecewise polynomials (trajectory.py --method min_snap).

One polynomial of degree 7 (or 9) per segment, C^3 (C^4) continuous. Positions are fixed at the gate centres,
everything else is chosen to minimise the integral of |snap|^2 (Mellinger & Kumar 2011; closed form as in
Richter, Bry & Roy 2013). Segment times are split for minimal snap, then scaled uniformly to the fastest path
within the limits.
With ArduPilot the lean angle binds, e.g. for the fly_track.sh sim:
    --max-tilt 42.8 --max-thrust 1.6 --max-acc 99 --max-jerk 80 --max-speed 9 --max-speed-z 5 --max-acc-z 7
"""
import math
from math import factorial

import numpy as np

HELP = "minimum snap (default degree 7), speeds at gates and time split optimised, fitted to the limits"


def add_args(ap):
    g = ap.add_argument_group("min_snap options")
    g.add_argument("--degree", type=int, default=7, help="polynomial degree per segment, odd: 7 (C^3) or 9 (C^4)")
    g.add_argument("--gate-vel", choices=["climb", "normal", "free", "fixed"], default="climb",
                   help="velocity at gates: climb = horizontally along the gate normal, speed and vertical speed "
                        "optimised (default; crosses gates climbing / descending, much less vertical acceleration); "
                        "normal = exactly along the (horizontal) normal, vz = 0; "
                        "free = any direction (may cross gates at an angle); fixed = --speed x normal. "
                        "--speed also sets the initial time split and the speed at a gate that starts / ends "
                        "the path (--no-start / --no-finish)")
    g.add_argument("--max-tilt", type=float, default=None,
                   help="peak lean angle [deg] needed for the acceleration (thrust along a + g); ArduPilot caps it "
                        "at ANGLE_MAX (fly_track.sh: 45). Off by default")
    g.add_argument("--max-thrust", type=float, default=None,
                   help="peak thrust needed, |a + g| / g (1 = hover); traj_prof_7 needed 1.6 and flew fine. "
                        "Off by default. Use it with --max-tilt: climbing lets the tilt limit buy a lot of "
                        "horizontal acceleration")
    g.add_argument("--max-speed-z", type=float, default=None,
                   help="peak vertical speed [m/s] (WPNAV_SPEED_UP / _DN, fly_track.sh: 5). Off by default")
    g.add_argument("--max-acc-z", type=float, default=None,
                   help="peak vertical acceleration [m/s^2] (WPNAV_ACCEL_Z, fly_track.sh: 7). Off by default")
    g.add_argument("--no-limits", action="store_true",
                   help="keep the times from --speed (and the time optimisation), do not rescale to the limits")
    g.add_argument("--no-time-opt", action="store_true", help="keep the initial split of time between segments")
    g.add_argument("--time-opt-iters", type=int, default=60)


def check_args(a):
    if a.degree not in (7, 9):
        raise SystemExit("--degree must be 7 or 9 (min-snap needs C^3 joints, i.e. degree >= 7)")


SNAP = 4                      # derivative order whose squared integral is minimised


# single segment: polynomial from end-point derivatives
class Basis:
    """Fixed matrices for degree N = 2k+1 in normalised time s = t/T in [0, 1]."""

    def __init__(self, degree, order=SNAP):
        self.N = degree
        self.k = (degree - 1) // 2
        n = degree + 1
        # A @ c = [p(0), p'(0), .., p^(k)(0), p(1), .., p^(k)(1)]  (derivatives w.r.t. s)
        A = np.zeros((n, n))
        for m in range(self.k + 1):
            A[m, m] = factorial(m)
            for j in range(m, n):
                A[self.k + 1 + m, j] = factorial(j) / factorial(j - m)
        self.Ainv = np.linalg.inv(A)
        # int_0^1 (d^r p / ds^r)^2 ds = c^T Q c
        Q = np.zeros((n, n))
        for i in range(order, n):
            for j in range(order, n):
                Q[i, j] = (factorial(i) / factorial(i - order)) * (factorial(j) / factorial(j - order)) \
                          / (i + j - 2 * order + 1)
        self.Qd = self.Ainv.T @ Q @ self.Ainv      # cost in terms of s-derivatives at both ends
        self.order = order
        self.pow = np.concatenate([np.arange(self.k + 1)] * 2)

    def scale(self, T):
        """Real-time end derivatives -> s-derivatives: multiply order m by T^m."""
        return T ** self.pow.astype(float)

    def hessian(self, T):
        """Cost int_0^T |d^r p/dt^r|^2 dt = d^T H d, d = real-time end derivatives [start 0..k, end 0..k]."""
        S = self.scale(T)
        return (self.Qd * np.outer(S, S)) / T ** (2 * self.order - 1)

    def hessian_dT(self, T):
        """d/dT of hessian(T): entry (a, b) goes as T^(pow_a + pow_b - 2r + 1)."""
        e = self.pow[:, None] + self.pow[None, :] - (2 * self.order - 1)
        return self.hessian(T) * e / T

    def coeffs(self, d, T):
        """d: [2(k+1), 3] real-time end derivatives -> s-polynomial coefficients [N+1, 3]."""
        return self.Ainv @ (d * self.scale(T)[:, None])


def eval_segment(C, T, ts, orders=4):
    """Derivatives of order 0..orders-1 (real time) at times ts in [0, T]."""
    N = len(C) - 1
    s = ts / T
    out = []
    for m in range(orders):
        val = np.zeros((len(ts), 3))
        for j in range(m, N + 1):
            val += np.outer(s ** (j - m), factorial(j) / factorial(j - m) * C[j])
        out.append(val / T ** m)
    return out


# whole path: joint derivatives as an affine function of the free variables
class Problem:
    """Joint derivatives D[point, order, axis] = D0 + B @ x, x = free variables."""

    def __init__(self, pts, basis, gate_vel, speed):
        self.pts, self.basis = pts, basis
        k = basis.k
        n = len(pts)
        self.shape = (n, k + 1, 3)
        size = n * (k + 1) * 3

        def idx(i, m, ax):
            return (i * (k + 1) + m) * 3 + ax

        D0 = np.zeros(size)
        cols = []
        for i, pt in enumerate(pts):
            for ax in range(3):
                D0[idx(i, 0, ax)] = pt["p"][ax]
            end = i == 0 or i == n - 1
            if "gate" not in pt:
                continue                                    # start / finish: at rest, all derivatives zero
            nrm = pt["gate"]["n"]
            if end or gate_vel == "fixed":                  # path starts / ends at a gate (--no-start/--no-finish)
                for ax in range(3):
                    D0[idx(i, 1, ax)] = speed * nrm[ax]
            elif gate_vel in ("normal", "climb"):
                c = np.zeros(size)
                for ax in range(3):
                    c[idx(i, 1, ax)] = nrm[ax]
                cols.append(c)
                if gate_vel == "climb":                     # plus a free vertical speed (normal is horizontal)
                    c = np.zeros(size); c[idx(i, 1, 2)] = 1.0; cols.append(c)
            else:
                for ax in range(3):
                    c = np.zeros(size); c[idx(i, 1, ax)] = 1.0; cols.append(c)
            if end:
                continue                                    # higher derivatives zero at the ends, as in poly5
            for m in range(2, k + 1):
                for ax in range(3):
                    c = np.zeros(size); c[idx(i, m, ax)] = 1.0; cols.append(c)
        self.D0 = D0
        self.B = np.array(cols).T if cols else np.zeros((size, 0))
        self.idx = idx
        # per segment: indices into the flat vector, [start 0..k, end 0..k] x axis
        self.seg_idx = []
        for i in range(n - 1):
            rows = [[idx(j, m, ax) for ax in range(3)] for j in (i, i + 1) for m in range(k + 1)]
            self.seg_idx.append(np.array(rows))
        # per segment, rows flattened as (end-point order, axis): the free variables touching it (B is sparse:
        # each segment only sees its two joints), so the reduced system is assembled from 24 x c blocks
        self.seg_rows, self.seg_cols, self.seg_B = [], [], []
        for rows in self.seg_idx:
            r = rows.reshape(-1)
            Bs = self.B[r]
            c = np.nonzero(np.abs(Bs).sum(axis=0))[0]
            self.seg_rows.append(r); self.seg_cols.append(c); self.seg_B.append(Bs[:, c])
        self.I3 = np.eye(3)

    def solve(self, T):
        """Minimum-snap joint derivatives for segment times T. Returns (D [n, k+1, 3], cost).
        Minimises sum_seg d_seg^T H_seg d_seg over d = D0 + B x (reduced normal equations, assembled per segment)."""
        nx = self.B.shape[1]
        Hs = [np.kron(self.basis.hessian(Ti), self.I3) for Ti in T]      # 3 axes, same (order, axis) layout
        if nx:
            A = np.zeros((nx, nx))
            b = np.zeros(nx)
            for H, r, c, Bs in zip(Hs, self.seg_rows, self.seg_cols, self.seg_B):
                HB = H @ Bs
                A[np.ix_(c, c)] += Bs.T @ HB
                b[c] -= HB.T @ self.D0[r]
            try:
                x = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                x = np.linalg.lstsq(A, b, rcond=None)[0]
            D = self.D0 + self.B @ x
        else:
            D = self.D0.copy()
        cost = sum(float(D[r] @ H @ D[r]) for H, r in zip(Hs, self.seg_rows))
        return D.reshape(self.shape), cost

    def cost_grad(self, T):
        """Optimal cost and its gradient w.r.t. the segment times. The free variables are at their optimum,
        so (envelope theorem) only the explicit dependence of each segment's cost on its own T counts."""
        D, J = self.solve(T)
        Df = D.reshape(-1)
        g = np.zeros(len(T))
        for i, (Ti, rows) in enumerate(zip(T, self.seg_idx)):
            dh = self.basis.hessian_dT(Ti)
            d = Df[rows]                                    # [2(k+1), 3]
            g[i] = float(np.sum(d * (dh @ d)))
        return J, g

    def segments(self, D, T):
        k = self.basis.k
        segs = []
        for i, Ti in enumerate(T):
            d = np.vstack([D[i, :k + 1], D[i + 1, :k + 1]])
            segs.append((self.basis.coeffs(d, Ti), Ti))
        return segs


# time allocation
def initial_times(pts, speed, min_seg):
    T = []
    for a, b in zip(pts[:-1], pts[1:]):
        dist = float(np.linalg.norm(b["p"] - a["p"]))
        va = speed if "gate" in a else 0.0
        vb = speed if "gate" in b else 0.0
        T.append(max(min_seg, dist / max(0.5 * (va + vb), 1e-6)))
    return np.array(T)


def optimise_split(prob, T0, iters, min_frac):
    """Redistribute the total time between segments for minimal snap, total time fixed."""
    total = float(T0.sum())
    n = len(T0)
    if n < 2 or iters <= 0:
        return T0

    def times(w):
        e = np.exp(w - w.max())
        T = total * e / e.sum()
        return np.maximum(T, min_frac * total)

    def f_grad(w):
        """log cost and its gradient w.r.t. the softmax weights w (T = total * softmax(w))."""
        T = times(w)
        J, gT = prob.cost_grad(T)
        p = T / total
        gw = T * gT - p * float(gT @ T)
        return math.log(max(J, 1e-300)), gw / max(J, 1e-300)

    w = np.log(T0 / total)
    fw, g = f_grad(w)
    step = 0.5
    for _ in range(iters):
        gn = float(np.linalg.norm(g))
        if gn < 1e-6:
            break
        improved = False
        while step > 1e-5:
            w_new = w - step * g / gn
            f_new, g_new = f_grad(w_new)
            if f_new < fw - 1e-4 * step * gn:
                w, fw, g = w_new, f_new, g_new
                step *= 1.5
                improved = True
                break
            step *= 0.5
        if not improved:
            break
    return times(w)


def sample(segs, dt, orders=4):
    cols = [[] for _ in range(orders)]
    tt = []
    t0 = 0.0
    t_pts = [0.0]
    for i, (C, T) in enumerate(segs):
        ts = np.arange(0.0, T, dt)
        if i == len(segs) - 1:
            ts = np.append(ts, T)
        for c, r in zip(cols, eval_segment(C, T, ts, orders)):
            c.append(r)
        tt.append(t0 + ts)
        t0 += T
        t_pts.append(t0)
    return (np.concatenate(tt), *(np.vstack(c) for c in cols), np.array(t_pts))


G = 9.81


def tilt_deg(acc):
    """Lean angle [deg] needed for acceleration acc (ENU, z up): thrust along acc + g."""
    return np.degrees(np.arctan2(np.linalg.norm(acc[:, :2], axis=1), G + acc[:, 2]))


def thrust_g(acc):
    """Thrust needed, in multiples of the hover thrust: |acc + g| / g."""
    return np.linalg.norm(acc + np.array([0.0, 0.0, G]), axis=1) / G


def limit_values(v, a, j):
    """Peak values of one sampled piece that the limits act on."""
    return dict(v=float(np.linalg.norm(v, axis=1).max()), a=float(np.linalg.norm(a, axis=1).max()),
                j=float(np.linalg.norm(j, axis=1).max()), vz=float(np.abs(v[:, 2]).max()),
                az=float(np.abs(a[:, 2]).max()), tilt=float(tilt_deg(a).max()), thrust=float(thrust_g(a).max()))


def peaks(prob, T, dt=0.02):
    """Peak values that the limits act on."""
    D, _ = prob.solve(T)
    _, _, v, a, j, _ = sample(prob.segments(D, T), dt)
    return limit_values(v, a, j)


def fit_limits(prob, T, lim):
    """Scale all segment times by one factor: the fastest trajectory within the limits (dict, None = off)."""
    def ok(alpha):
        pk = peaks(prob, T * alpha)
        return all(pk[k] <= v for k, v in lim.items() if v is not None)
    feas = 1.0                                              # bracket: ok(feas), not ok(infeas), infeas < feas
    if ok(feas):
        while ok(feas * 0.5) and feas > 1e-3:
            feas *= 0.5
    else:
        while not ok(feas) and feas < 1e3:
            feas *= 2.0
        if not ok(feas):
            raise SystemExit("cannot meet the speed / accel / jerk / tilt limits even very slowly (fixed gate speeds?)")
    infeas = feas * 0.5
    for _ in range(30):
        mid = math.sqrt(feas * infeas)
        if ok(mid):
            feas = mid
        else:
            infeas = mid
        if feas / infeas < 1.001:
            break
    return T * feas, feas


def generate(pts, a):
    basis = Basis(a.degree)
    prob = Problem(pts, basis, a.gate_vel, a.speed)
    T = initial_times(pts, a.speed, a.min_seg)
    cost0 = prob.solve(T)[1]
    if not a.no_time_opt:
        T = optimise_split(prob, T, a.time_opt_iters, min_frac=0.2 * a.min_seg / T.sum())
    cost1 = prob.solve(T)[1] * (T.sum() / initial_times(pts, a.speed, a.min_seg).sum()) ** 7
    alpha = 1.0
    if not a.no_limits:
        lim = dict(v=a.max_speed, a=a.max_acc, j=a.max_jerk, vz=a.max_speed_z, az=a.max_acc_z, tilt=a.max_tilt,
                   thrust=a.max_thrust)
        T, alpha = fit_limits(prob, T, lim)
    T = T * a.time_scale

    D, cost = prob.solve(T)
    segs = prob.segments(D, T)
    t, p, v, acc, jerk, t_pts = sample(segs, a.dt)
    p[0], p[-1] = pts[0]["p"], pts[-1]["p"]    # exact ends (float noise ~1e-11 hides an end exactly on a gate plane)

    # boundary check (ill-conditioning guard): positions at the points and continuity at the joints
    err = 0.0
    for i, (C, Ti) in enumerate(segs):
        r0 = eval_segment(C, Ti, np.array([0.0]), basis.k + 1)
        r1 = eval_segment(C, Ti, np.array([Ti]), basis.k + 1)
        for m in range(basis.k + 1):
            err = max(err, float(np.abs(r0[m][0] - D[i, m]).max()), float(np.abs(r1[m][0] - D[i + 1, m]).max()))
        err = max(err, float(np.abs(r0[0][0] - pts[i]["p"]).max()), float(np.abs(r1[0][0] - pts[i + 1]["p"]).max()))

    gate_pts = [i for i, q in enumerate(pts) if "gate" in q]
    speed = np.linalg.norm(v, axis=1)
    tilt = tilt_deg(acc)
    thrust = thrust_g(acc)
    vz, az = np.abs(v[:, 2]).max(), np.abs(acc[:, 2]).max()

    problems = []
    gate_speed = [float(D[i, 1] @ pts[i]["gate"]["n"]) for i in gate_pts]
    if min(gate_speed) <= 0:
        problems.append(f"negative speed along a gate normal ({min(gate_speed):+.2f} m/s): flies through backwards")
    if speed.max() > a.max_speed * 1.01:
        problems.append(f"speed {speed.max():.1f} m/s exceeds --max-speed {a.max_speed:.1f}")
    if a.max_tilt is not None and tilt.max() > a.max_tilt * 1.01:
        problems.append(f"lean angle {tilt.max():.1f} deg exceeds --max-tilt {a.max_tilt:.0f}")
    if a.max_thrust is not None and thrust.max() > a.max_thrust * 1.01:
        problems.append(f"thrust {thrust.max():.2f} x hover exceeds --max-thrust {a.max_thrust:.2f}")
    if a.max_speed_z is not None and vz > a.max_speed_z * 1.01:
        problems.append(f"vertical speed {vz:.1f} m/s exceeds --max-speed-z {a.max_speed_z:.1f}")
    if a.max_acc_z is not None and az > a.max_acc_z * 1.01:
        problems.append(f"vertical accel {az:.1f} m/s^2 exceeds --max-acc-z {a.max_acc_z:.1f}")

    info = [f"degree {a.degree} (C^{basis.k} continuous), min-snap, gate velocity: {a.gate_vel}"]
    if not a.no_time_opt:
        info.append(f"time split optimisation : snap cost x{cost1 / max(cost0, 1e-300):.2f} of the initial split")
    if not a.no_limits:
        extra = "".join(f", {n} <= {x:g}" for n, x in (("tilt", a.max_tilt), ("thrust", a.max_thrust),
                                                        ("vz", a.max_speed_z), ("az", a.max_acc_z)) if x is not None)
        info.append(f"limits                  : speed <= {a.max_speed:.1f}, accel <= {a.max_acc:.1f}, "
                    f"jerk <= {a.max_jerk:.0f}{extra} (time x{alpha:.2f}"
                    + (f", then x{a.time_scale:.2f}" if a.time_scale != 1.0 else "") + ")")
    info.append("speed at gates [m/s]    : " + " ".join(f"{pts[i]['label']}={np.linalg.norm(D[i, 1]):.1f}"
                                                      for i in gate_pts))
    stats = [f"vertical speed / accel  : {vz:.2f} m/s / {az:.2f} m/s^2",
             f"lean angle max          : {tilt.max():.1f} deg    thrust max {thrust.max():.2f} x hover",
             f"snap cost               : {cost:.3g}"]
    return dict(t=t, p=p, v=v, acc=acc, jerk=jerk, t_pts=t_pts, err=err, info=info, stats=stats, problems=problems)
