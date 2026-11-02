#!/usr/bin/env python3
"""Correct VIO drift with the ArUco tags on the gates.

Each camera frame: detect tags, solve the camera pose from their known corners (PnP), compare it with the VIO pose
at the image time and low-pass the difference into an offset map = Rz(psi) * vio + t. Bad fixes are rejected
(small marker, high reprojection error, ambiguous single tag, offset jump).

Observe only (default): reads /mavros/vision_pose/pose, logs the fixes and the offset, changes nothing.
Correcting (tmux_vio.sh correct): the relay publishes to /vio/pose_raw and this node publishes the corrected pose
to /mavros/vision_pose/pose. The offset is applied at slew_pos m/s and slew_yaw deg/s so the EKF never sees a step.
"""
import csv
import math
import os
import time
from bisect import bisect_left
from collections import deque

import cv2
import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
except ImportError:           # the core below works without ROS
    rclpy = None

CH = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def roll_of(R):
    return math.atan2(R[2, 1], R[2, 2])


def pitch_of(R):
    """Rotation angle about +y (nose down positive)."""
    return -math.asin(max(-1.0, min(1.0, R[2, 0])))


def find_tags(path):
    """The CSV as given, else in the vio_launcher share/config/track folder, else next to this script."""
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [path, os.path.join(here, path), os.path.join(here, "track_out", os.path.basename(path))]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.insert(1, os.path.join(get_package_share_directory("vio_launcher"), "config", "track", os.path.basename(path)))
    except Exception:
        pass
    for p in cands:
        if os.path.exists(os.path.expanduser(p)):
            return os.path.expanduser(p)
    raise SystemExit(f"{path} not found (tried {cands}): run make_track.py and rebuild, or pass -p tags_csv:=<file>")


def load_tags(path):
    tags = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            pts = np.array([[float(r[f"{k}{j}"]) for k in "xyz"] for j in range(4)])      # TL, TR, BR, BL
            tags[int(r["id"])] = pts
    if not tags:
        raise SystemExit(f"no tags in {path}")
    return tags


class Fixer:
    """Image -> body pose in the tag frame."""

    def __init__(self, tags, cam_pitch=-0.349, cam_ahead=0.135, f=554.383, cx=320.0, cy=240.0,
                 min_px=25.0, max_reproj=3.0, ambig=2.0, min_tags=1):
        self.tags = tags
        self.K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])
        p = cam_pitch      # rad, positive = camera looks up
        # columns: optical x (right), y (down), z (forward) in the body frame (x forward, y left, z up)
        self.R_bc = np.array([[0, math.sin(p), math.cos(p)], [-1, 0, 0], [0, -math.cos(p), math.sin(p)]])
        self.ahead = cam_ahead
        self.min_px, self.max_reproj, self.ambig, self.min_tags = min_px, max_reproj, ambig, min_tags
        prm = cv2.aruco.DetectorParameters()
        prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100), prm)

    def fix(self, gray):
        """Returns (fix or None, reason). The fix holds the body pose p, R, yaw and quality (n, side, reproj, dist)."""
        corners, ids, _ = self.det.detectMarkers(gray)
        if ids is None:
            return None, "no tags"
        use = []
        for i, c in zip(ids.ravel(), corners):
            i, e = int(i), c.reshape(4, 2)
            side = float(np.mean([np.linalg.norm(e[j] - e[(j + 1) % 4]) for j in range(4)]))
            if i in self.tags and side >= self.min_px:
                use.append((i, e, side))
        if len(use) < self.min_tags or not use:
            return None, "tags too small or unknown"
        obj = np.concatenate([self.tags[i] for i, _, _ in use]).astype(np.float64)
        img = np.concatenate([e for _, e, _ in use]).astype(np.float64)
        if len(use) == 1:
            n, rvs, tvs, err = cv2.solvePnPGeneric(obj, img, self.K, None, flags=cv2.SOLVEPNP_IPPE)
            if n == 0:
                return None, "pnp failed"
            err = np.ravel(err)
            b = int(np.argmin(err))
            if len(err) > 1 and np.max(err) < self.ambig * max(err[b], 1e-6):
                return None, "ambiguous single tag"
            rv, tv = rvs[b], tvs[b]
        else:
            ok, rv, tv = cv2.solvePnP(obj, img, self.K, None, flags=cv2.SOLVEPNP_SQPNP)
            if not ok:
                return None, "pnp failed"
        rv, tv = cv2.solvePnPRefineLM(obj, img, self.K, None, rv, tv)
        proj, _ = cv2.projectPoints(obj, rv, tv, self.K, None)
        reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
        if reproj > self.max_reproj:
            return None, "high reprojection error"
        Rcw, _ = cv2.Rodrigues(rv)
        R_mb = Rcw.T @ self.R_bc.T
        c = (-Rcw.T @ tv).ravel()
        p = c - R_mb @ np.array([self.ahead, 0, 0])
        dist = float(np.mean([np.linalg.norm(self.tags[i].mean(axis=0) - c) for i, _, _ in use]))
        return dict(p=p, R=R_mb, yaw=yaw_of(R_mb), n=len(use), ids=[i for i, _, _ in use],
                    side=min(s for _, _, s in use), reproj=reproj, dist=dist), "ok"


class OffsetFilter:
    """map = Rz(psi) * vio + t, low-passed from the fixes."""

    def __init__(self, tau=5.0, gate_t=0.5, gate_psi=math.radians(5.0), full_px=60.0):
        self.tau, self.gate_t, self.gate_psi, self.full_px = tau, gate_t, gate_psi, full_px
        self.psi, self.t = 0.0, np.zeros(3)
        self.last = None
        self.n_acc = self.n_rej = 0

    def update(self, stamp, fix, vio_p, vio_yaw):
        """Returns (accepted, psi_meas, t_meas)."""
        psi_m = wrap(fix["yaw"] - vio_yaw)
        t_m = fix["p"] - Rz(self.psi) @ vio_p
        if abs(wrap(psi_m - self.psi)) > self.gate_psi or np.linalg.norm(t_m - self.t) > self.gate_t:
            self.n_rej += 1
            return False, psi_m, t_m
        q = min(1.0, fix["side"] / self.full_px) * (1.0 if fix["n"] >= 2 else 0.5)
        dt = 1.0 / 20 if self.last is None else min(max(stamp - self.last, 0.0), 1.0)
        a = min(1.0, dt / self.tau) * q
        if self.last is None:
            a = 1.0 * q
        self.psi += a * wrap(psi_m - self.psi)
        self.t = self.t + a * (t_m - self.t)
        self.last = stamp
        self.n_acc += 1
        return True, psi_m, t_m


class Pose:
    def __init__(self, t, p, R):
        self.t, self.p, self.R = t, p, R


def interp(buf, stamp, max_gap=0.1):
    """VIO (position, yaw, roll, pitch) at the image time, linear between the samples, or None."""
    if not buf:
        return None
    ts = [b.t for b in buf]
    if stamp > ts[-1]:
        b = buf[-1]
        return (b.p, yaw_of(b.R), roll_of(b.R), pitch_of(b.R)) if stamp - ts[-1] < max_gap else None
    i = bisect_left(ts, stamp)
    if i == 0:
        b = buf[0]
        return (b.p, yaw_of(b.R), roll_of(b.R), pitch_of(b.R)) if ts[0] - stamp < max_gap else None
    a, b = buf[i - 1], buf[i]
    w = (stamp - a.t) / max(b.t - a.t, 1e-6)
    ya, yb = yaw_of(a.R), yaw_of(b.R)
    return (a.p + w * (b.p - a.p), ya + w * wrap(yb - ya),
            roll_of(a.R) + w * wrap(roll_of(b.R) - roll_of(a.R)), pitch_of(a.R) + w * (pitch_of(b.R) - pitch_of(a.R)))


if rclpy is not None:
    class Correction(Node):
        def __init__(self):
            super().__init__("aruco_correction")
            P = self.declare_parameter
            P("image_topic", "/iris_rgb/camera_image")
            P("vio_topic", "/mavros/vision_pose/pose")
            P("out_topic", "")
            P("tags_csv", "tags_start_frame.csv")
            P("cam_pitch", -0.349)
            P("cam_ahead", 0.135)
            P("min_px", 25.0)
            P("max_reproj", 3.0)
            P("min_tags", 1)
            P("tau", 5.0)
            P("filter_min_px", 45.0)       # smaller markers are logged but not used
            P("gate_t", 0.5)
            P("gate_yaw_deg", 5.0)
            P("slew_pos", 0.2)
            P("slew_yaw", 2.0)
            P("log", "")
            g = lambda k: self.get_parameter(k).value
            self.fixer = Fixer(load_tags(find_tags(g("tags_csv"))), g("cam_pitch"), g("cam_ahead"),
                               min_px=g("min_px"), max_reproj=g("max_reproj"), min_tags=g("min_tags"))
            self.filt = OffsetFilter(g("tau"), g("gate_t"), math.radians(g("gate_yaw_deg")))
            self.filter_min_px = g("filter_min_px")
            self.slew_pos, self.slew_yaw = g("slew_pos"), math.radians(g("slew_yaw"))
            self.buf = deque(maxlen=400)
            self.applied_psi, self.applied_t, self.last_pub = 0.0, np.zeros(3), None
            self.out = None
            if g("out_topic"):
                self.out = self.create_publisher(PoseStamped, g("out_topic"), 10)
            self.create_subscription(PoseStamped, g("vio_topic"), self.on_vio, qos_profile_sensor_data)
            self.create_subscription(Image, g("image_topic"), self.on_image, qos_profile_sensor_data)
            self.log = None
            prefix = g("log") or os.path.join(os.path.expanduser("~/ros2_ws/flight_logs"),
                                              "aruco_fix_" + time.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(os.path.dirname(prefix) or ".", exist_ok=True)
            self.log = open(prefix + ".csv", "w", newline="", buffering=1)
            self.csv = csv.writer(self.log)
            self.csv.writerow(["t", "n_tags", "side_px", "dist", "reproj", "fix_x", "fix_y", "fix_z", "fix_yaw",
                               "vio_x", "vio_y", "vio_z", "vio_yaw", "psi_meas", "tx_meas", "ty_meas", "tz_meas",
                               "accepted", "psi", "tx", "ty", "tz", "droll", "dpitch"])
            self.cam_pitch = g("cam_pitch")
            self.rp = deque(maxlen=300)         # fix - VIO roll / pitch, for the mount check
            self.frames = 0
            self.reasons = {}
            self.next_report = time.monotonic() + 2.0
            self.get_logger().info(f"{'correcting' if self.out else 'observing only'}; log {prefix}.csv")

        def on_vio(self, m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            q, p = m.pose.orientation, m.pose.position
            R = quat_to_R(q.x, q.y, q.z, q.w)
            pv = np.array([p.x, p.y, p.z])
            self.buf.append(Pose(t, pv, R))
            if self.out is None:
                return
            dt = 0.0 if self.last_pub is None else min(max(t - self.last_pub, 0.0), 0.2)
            self.last_pub = t
            dpsi = wrap(self.filt.psi - self.applied_psi)
            self.applied_psi += max(-self.slew_yaw * dt, min(self.slew_yaw * dt, dpsi))
            dt3 = self.filt.t - self.applied_t
            n = np.linalg.norm(dt3)
            if n > 1e-9:
                self.applied_t = self.applied_t + dt3 * min(1.0, self.slew_pos * dt / n)
            Rc = Rz(self.applied_psi)
            o = PoseStamped()
            o.header = m.header
            pc = Rc @ pv + self.applied_t
            o.pose.position.x, o.pose.position.y, o.pose.position.z = map(float, pc)
            o.pose.orientation.x, o.pose.orientation.y, o.pose.orientation.z, o.pose.orientation.w = map(float, R_to_quat(Rc @ R))
            self.out.publish(o)

        def on_image(self, m):
            if m.encoding not in CH:
                return
            ch = CH[m.encoding]
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * ch].reshape(m.height, m.width, ch)
            if ch == 1:
                gray = a[:, :, 0]
            else:
                code = {"rgb8": cv2.COLOR_RGB2GRAY, "bgr8": cv2.COLOR_BGR2GRAY, "rgba8": cv2.COLOR_RGBA2GRAY,
                        "bgra8": cv2.COLOR_BGRA2GRAY}[m.encoding]
                gray = cv2.cvtColor(np.ascontiguousarray(a), code)
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.frames += 1
            fx, why = self.fixer.fix(gray)
            if fx is None:
                self.reasons[why] = self.reasons.get(why, 0) + 1
                return
            v = interp(self.buf, stamp)
            if v is None:
                self.reasons["no vio pose at image time"] = self.reasons.get("no vio pose at image time", 0) + 1
                return
            vp, vy, vr, vpit = v
            if fx["side"] < self.filter_min_px:
                ok, psi_m, t_m = False, wrap(fx["yaw"] - vy), fx["p"] - Rz(self.filt.psi) @ vp
                self.reasons["marker too small for the filter"] = self.reasons.get("marker too small for the filter", 0) + 1
            else:
                ok, psi_m, t_m = self.filt.update(stamp, fx, vp, vy)
                if not ok:
                    self.reasons["offset jump"] = self.reasons.get("offset jump", 0) + 1
            dr, dpit = wrap(roll_of(fx["R"]) - vr), pitch_of(fx["R"]) - vpit
            if ok:
                self.rp.append((dr, dpit))
            self.csv.writerow([f"{stamp:.3f}", fx["n"], f"{fx['side']:.1f}", f"{fx['dist']:.2f}", f"{fx['reproj']:.2f}",
                               *[f"{x:.4f}" for x in fx["p"]], f"{fx['yaw']:.4f}", *[f"{x:.4f}" for x in vp], f"{vy:.4f}",
                               f"{psi_m:.4f}", *[f"{x:.4f}" for x in t_m], int(ok), f"{self.filt.psi:.4f}",
                               *[f"{x:.4f}" for x in self.filt.t], f"{dr:.4f}", f"{dpit:.4f}"])
            now = time.monotonic()
            if now >= self.next_report:
                self.log.flush()
                rej = ", ".join(f"{k}: {n}" for k, n in sorted(self.reasons.items())) or "none"
                print(f"frames {self.frames}  accepted {self.filt.n_acc}  offset psi {math.degrees(self.filt.psi):+.2f} deg  "
                      f"t ({self.filt.t[0]:+.2f}, {self.filt.t[1]:+.2f}, {self.filt.t[2]:+.2f}) m   skipped: {rej}", flush=True)
                if len(self.rp) >= 30:
                    mr, mp = np.median(np.array(self.rp), axis=0)
                    print(f"  camera mount check: fix - VIO  roll {math.degrees(mr):+.2f} deg, pitch {math.degrees(mp):+.2f} deg "
                          f"(~0 if cam_pitch is right, else try {self.cam_pitch - mp:.3f} rad)", flush=True)
                self.next_report = now + 2.0


def main():
    rclpy.init()
    node = Correction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.log.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    if rclpy is None:
        raise SystemExit("ROS 2 python (rclpy) not found: source /opt/ros/humble/setup.bash")
    main()