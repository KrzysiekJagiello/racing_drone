#!/usr/bin/env python3
"""Play a trajectory CSV (made by trajectory.py) to ArduPilot through MAVROS.

    ros2 run vio_launcher traj_follower.py --ros-args -p use_sim_time:=true -p traj:=/path/traj.csv

The drone must already be armed, in GUIDED and hovering at the first point of the trajectory
(fly_track.sh does that). The node then streams, at rate_hz, to /mavros/setpoint_raw/local:
position + velocity + acceleration feed-forward + yaw, sampled from the CSV on the sim clock.
Values are ENU in the frame the EKF uses (MAVROS converts them to NED), so the CSV must be in that frame:
  - EKF on GPS / compass : the world-frame trajectory
  - EKF switched to VIO  : the start-frame trajectory

Parameters
  traj          path of the trajectory CSV (t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw)
  rate_hz       setpoint rate                                   (default 50)
  hold_s        keep holding the last point for this long       (default 2)
  start_tol     abort if the drone is farther than this [m] from the first point   (default 0.5)
  yaw_rate_max  limit on how fast the commanded yaw may turn [rad/s]   (default 1.5)
  contour_gain  shift the setpoint sideways, against the horizontal cross-path error, by gain x error
                (0 = off, the old behaviour; try 0.5, then 1.0)               (default 0)
  contour_max   limit on that shift [m]                                (default 0.5)
  contour_min_speed  no correction below this reference speed [m/s]    (default 1.0)
  search_s      the nearest path point is searched within +-search_s of the reference time   (default 1.0)
  actual_topic  odometry used for the tracking error             (default /mavros/local_position/odom)
  log           output prefix for <log>.csv with reference vs actual (default <flight_logs>/track_log_<time>,
                flight_logs = $FLIGHT_LOGS or ~/ros2_ws/flight_logs)

Exit code 0 = finished, 1 = aborted (wrong start position, bad file, interrupted).
The log also has the setpoint actually sent (cmd_x,y,z) and the error split into along / across the path.
The tracking error is measured against the EKF estimate. With the EKF on VIO that is the VIO itself, so it
shows how well the drone follows what it believes, not true accuracy (that needs the Gazebo ground truth).
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def default_log_prefix():
    """<flight_logs>/track_log_<time>, flight_logs = $FLIGHT_LOGS or ~/ros2_ws/flight_logs."""
    d = os.path.expanduser(os.environ.get("FLIGHT_LOGS", "~/ros2_ws/flight_logs"))
    return os.path.join(d, time.strftime("track_log_%Y%m%d_%H%M%S"))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Reference:
    """Trajectory samples with linear interpolation (the CSV is dense, 100 Hz)."""

    def __init__(self, path):
        d = np.loadtxt(path, delimiter=",", skiprows=1)
        if d.ndim != 2 or d.shape[1] < 14 or len(d) < 2:
            raise ValueError(f"{path}: expected a CSV with 14 columns t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw")
        self.t = d[:, 0]
        self.d = d
        self.T = float(self.t[-1])
        self.p0 = d[0, 1:4].copy()
        self.p_end = d[-1, 1:4].copy()

    def at(self, t):
        t = min(max(t, 0.0), self.T)
        c = [np.interp(t, self.t, self.d[:, k]) for k in range(1, 14)]
        return np.array(c[0:3]), np.array(c[3:6]), np.array(c[6:9]), float(c[12])


    def nearest(self, pos, t, window):
        """Path sample nearest to pos within +-window s of t. Returns (horizontal unit tangent, cross-path vector
        (horizontal, from the path to pos)) or None when the path direction is undefined (nearly vertical)."""
        i0 = int(np.searchsorted(self.t, max(t - window, 0.0)))
        i1 = int(np.searchsorted(self.t, min(t + window, self.T))) + 1
        P = self.d[i0:i1, 1:4]
        if len(P) == 0:
            return None
        i = i0 + int(np.argmin(np.sum((P - pos) ** 2, axis=1)))
        th = self.d[i, 4:6]
        n = float(np.linalg.norm(th))
        if n < 1e-3:
            return None
        th = th / n
        e = pos[:2] - self.d[i, 1:3]
        return th, e - (e @ th) * th


class Follower(Node):
    def __init__(self):
        super().__init__("traj_follower")
        self.declare_parameter("traj", "")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("hold_s", 2.0)
        self.declare_parameter("start_tol", 0.5)
        self.declare_parameter("yaw_rate_max", 1.5)
        self.declare_parameter("contour_gain", 0.0)
        self.declare_parameter("contour_max", 0.5)
        self.declare_parameter("contour_min_speed", 1.0)
        self.declare_parameter("search_s", 1.0)
        self.declare_parameter("actual_topic", "/mavros/local_position/odom")
        self.declare_parameter("log", "")
        path = self.get_parameter("traj").value
        if not path:
            raise SystemExit("give the trajectory: --ros-args -p traj:=/path/to/traj.csv")
        self.ref = Reference(path)
        self.hold_s = float(self.get_parameter("hold_s").value)
        self.start_tol = float(self.get_parameter("start_tol").value)
        self.yaw_rate_max = float(self.get_parameter("yaw_rate_max").value)
        self.log_prefix = os.path.expanduser(self.get_parameter("log").value or default_log_prefix())
        rate = float(self.get_parameter("rate_hz").value)
        self.c_gain = float(self.get_parameter("contour_gain").value)
        self.c_max = float(self.get_parameter("contour_max").value)
        self.c_min_speed = float(self.get_parameter("contour_min_speed").value)
        self.search_s = float(self.get_parameter("search_s").value)

        self.pos = None
        self.yaw = 0.0
        self.t0 = None
        self.last_now = None
        self.ycmd = 0.0
        self.rows = []
        self.done = False
        self.exit_code = 0
        self.next_report = 0.0
        self.next_wait_msg = 0.0

        self.pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.create_subscription(Odometry, self.get_parameter("actual_topic").value, self.on_odom,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"{path}: {self.ref.T:.1f} s, first point {np.round(self.ref.p0, 2).tolist()}, "
                               f"contour gain {self.c_gain} (max {self.c_max} m)")

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.done:
            return
        now = self.now()
        if self.pos is None:
            if now >= self.next_wait_msg:
                self.get_logger().info("waiting for odometry...")
                self.next_wait_msg = now + 2.0
            return

        if self.t0 is None:
            dist = float(np.linalg.norm(self.pos - self.ref.p0))
            if dist > self.start_tol:
                self.get_logger().error(
                    f"drone is {dist:.2f} m from the first trajectory point {np.round(self.ref.p0, 2).tolist()} "
                    f"(at {np.round(self.pos, 2).tolist()}), tolerance {self.start_tol} m. Wrong frame (world vs "
                    f"start-frame file) or not hovering at the start? Aborting without moving.")
                self.finish(1)
                return
            self.t0 = now
            self.last_now = now
            self.ycmd = self.yaw
            self.get_logger().info("start following")

        t = now - self.t0
        p, v, a, yref = self.ref.at(t)
        running = t <= self.ref.T
        if not running:                       # past the end: hold the last point at rest
            p, v, a = self.ref.p_end, np.zeros(3), np.zeros(3)

        pref = p                                  # reference position, before any correction
        lat = along = 0.0
        if running and np.linalg.norm(v[:2]) >= self.c_min_speed:
            nr = self.ref.nearest(self.pos, t, self.search_s)
            if nr is not None:
                th, cross = nr
                along = float((self.pos[:2] - pref[:2]) @ th)         # ahead (+) or behind (-) the time reference
                lat = float(np.linalg.norm(cross))
                if self.c_gain > 0.0 and lat > 1e-6:
                    shift = min(self.c_gain * lat, self.c_max) * cross / lat
                    p = p.copy()
                    p[:2] = p[:2] - shift                     # push back against the sideways error
        dt = max(now - self.last_now, 0.0)
        self.last_now = now
        step = self.yaw_rate_max * dt
        self.ycmd += max(-step, min(step, wrap(yref - self.ycmd)))

        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED      # MAVROS converts ENU -> NED
        m.type_mask = PositionTarget.IGNORE_YAW_RATE             # use position, velocity, acceleration and yaw
        m.position.x, m.position.y, m.position.z = float(p[0]), float(p[1]), float(p[2])
        m.velocity.x, m.velocity.y, m.velocity.z = float(v[0]), float(v[1]), float(v[2])
        m.acceleration_or_force.x, m.acceleration_or_force.y, m.acceleration_or_force.z = (
            float(a[0]), float(a[1]), float(a[2]))
        m.yaw = float(wrap(self.ycmd))
        self.pub.publish(m)

        if running:
            self.rows.append([t, *pref, *self.pos, *p, lat, along])
            if now >= self.next_report:
                e = float(np.linalg.norm(self.pos - pref))
                self.get_logger().info(f"t={t:5.1f}/{self.ref.T:.1f} s   ref speed {np.linalg.norm(v):.2f} m/s   "
                                       f"position error {e:.2f} m")
                self.next_report = now + 2.0

        if t > self.ref.T + self.hold_s:
            self.finish(0)

    def finish(self, code):
        if self.done:
            return
        self.done = True
        self.exit_code = code
        if len(self.rows) > 10:
            r = np.array(self.rows)
            err = r[:, 4:7] - r[:, 1:4]
            e3 = np.linalg.norm(err, axis=1)
            eh = np.linalg.norm(err[:, :2], axis=1)
            os.makedirs(os.path.dirname(os.path.abspath(self.log_prefix)), exist_ok=True)
            np.savetxt(self.log_prefix + ".csv", r, delimiter=",", fmt="%.5f",
                       header="t,ref_x,ref_y,ref_z,act_x,act_y,act_z,cmd_x,cmd_y,cmd_z,cross,along", comments="")
            print("\n================ trajectory tracking ================")
            print(f"followed {r[-1, 0]:.1f} s of {self.ref.T:.1f} s")
            print(f"position error RMS (3D)  : {np.sqrt(np.mean(e3 ** 2)):.3f} m")
            print(f"  horizontal / vertical  : {np.sqrt(np.mean(eh ** 2)):.3f} m / "
                  f"{np.sqrt(np.mean(err[:, 2] ** 2)):.3f} m")
            print(f"max error                : {e3.max():.3f} m")
            mv = r[:, 10] > 0                      # rows where the path direction was defined (moving)
            if mv.any():
                print(f"across the path (horiz.) : mean {r[mv, 10].mean():.3f} m  p95 {np.percentile(r[mv, 10], 95):.3f} m")
                print(f"along the path (horiz.)  : mean |.| {np.abs(r[mv, 11]).mean():.3f} m  signed {r[mv, 11].mean():+.3f} m")
            print("(error vs the EKF position estimate, not vs Gazebo ground truth)")
            print("=====================================================")
            print("log saved to", self.log_prefix + ".csv\n")


def main():
    rclpy.init()
    node = Follower()
    errors = 0
    try:
        while rclpy.ok() and not node.done:
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
                errors = 0
            except Exception as e:                       # never lose the log to a spin error
                errors += 1
                if node.done or errors > 20:
                    print("spin_once keeps failing:", e)
                    node.finish(1)
                    break
    except KeyboardInterrupt:
        node.finish(1)
    code = node.exit_code
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(code)


if __name__ == "__main__":
    main()