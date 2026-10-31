#!/usr/bin/env python3
"""Compare OpenVINS odometry with a reference trajectory (ground truth).

Record (ROS 2 workspace sourced, sim and OpenVINS running):
    python3 vio_vs_gt.py --gt /ground_truth/odom           # Gazebo truth
    python3 vio_vs_gt.py --gt /mavros/local_position/odom  # EKF, not truth
Stop with Ctrl-C: saves vio_run_<time>.npz and .png in $FLIGHT_LOGS (default ~/ros2_ws/flight_logs)
and prints the metrics.

Re-analyse a saved run (no ROS needed):
    python3 vio_vs_gt.py --analyze vio_run_XXXX.npz

Alignment: the VIO frame differs from the reference by a yaw rotation and a translation (4 DoF).
Only those are fitted, no scale, so a scale error stays in the error and is reported separately.
"""
import argparse
import os
import signal
import sys
import time

import numpy as np


def log_dir():
    """$FLIGHT_LOGS or ~/ros2_ws/flight_logs, created if missing."""
    d = os.path.expanduser(os.environ.get("FLIGHT_LOGS", "~/ros2_ws/flight_logs"))
    os.makedirs(d, exist_ok=True)
    return d


def align_yaw_translation(vio, gt):
    """Yaw rotation R and translation t minimising |R @ vio + t - gt|. Also returns the yaw and the scale (1 = correct)."""
    a = vio - vio.mean(0)
    b = gt - gt.mean(0)
    yaw = np.arctan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]),
                     np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    ra = a @ R.T
    scale = float(np.sum(b * ra) / max(np.sum(ra * ra), 1e-12))
    t = gt.mean(0) - R @ vio.mean(0)
    return R, t, float(yaw), scale


def associate(t_v, p_v, t_g, p_g):
    """Interpolate the reference at the VIO timestamps, where both overlap."""
    order = np.argsort(t_g)
    t_g, p_g = t_g[order], p_g[order]
    keep = np.concatenate(([True], np.diff(t_g) > 0))
    t_g, p_g = t_g[keep], p_g[keep]
    lo, hi = max(t_v[0], t_g[0]), min(t_v[-1], t_g[-1])
    if hi <= lo:
        raise SystemExit(
            "No time overlap between the two trajectories.\n"
            f"  VIO       : {t_v[0]:.2f} .. {t_v[-1]:.2f}\n"
            f"  reference : {t_g[0]:.2f} .. {t_g[-1]:.2f}\n"
            "They are probably on different clocks. Re-run recording with --stamp arrival, "
            "or use the bridged gz ground truth, which shares the sim clock.")
    m = (t_v >= lo) & (t_v <= hi)
    g = np.stack([np.interp(t_v[m], t_g, p_g[:, k]) for k in range(3)], axis=1)
    return t_v[m], p_v[m], g


def analyse(vio, gt, out_prefix):
    t_v, p_v = vio[:, 0], vio[:, 1:4]
    t_g, p_g = gt[:, 0], gt[:, 1:4]
    if len(t_v) < 20:
        raise SystemExit(f"Only {len(t_v)} VIO samples. Did OpenVINS initialise and publish /ov_msckf/odomimu?")
    if len(t_g) < 20:
        raise SystemExit(f"Only {len(t_g)} reference samples. Check the --gt topic.")
    t, v, g = associate(t_v, p_v, t_g, p_g)
    R, tr, yaw, scale = align_yaw_translation(v, g)
    v_al = v @ R.T + tr
    err = v_al - g
    e3 = np.linalg.norm(err, axis=1)
    eh = np.linalg.norm(err[:, :2], axis=1)
    path = float(np.sum(np.linalg.norm(np.diff(g, axis=0), axis=1)))
    vpath = float(np.sum(np.linalg.norm(np.diff(v, axis=0), axis=1)))

    print("\n================ VIO vs reference ================")
    print(f"samples compared        : {len(t)}  over {t[-1] - t[0]:.1f} s")
    print(f"reference path length   : {path:.2f} m   (VIO path length: {vpath:.2f} m)")
    print(f"ATE RMSE (3D)           : {np.sqrt(np.mean(e3 ** 2)):.3f} m")
    print(f"  horizontal / vertical : {np.sqrt(np.mean(eh ** 2)):.3f} m / {np.sqrt(np.mean(err[:, 2] ** 2)):.3f} m")
    print(f"max error               : {e3.max():.3f} m")
    print(f"scale needed            : {scale:.3f}   (1.000 = metrically correct)")
    print(f"frame yaw offset        : {np.degrees(yaw):.1f} deg   (expected: VIO heading is arbitrary)")
    if path < 3.0:
        print("NOTE: under ~3 m of travel the yaw alignment is poorly conditioned; fly a longer path "
              "(10-20 m, with a turn) for a trustworthy number.")
    print("===================================================\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping the plot (pip install matplotlib).")
        return
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    ax[0].plot(g[:, 0], g[:, 1], label="reference", lw=2)
    ax[0].plot(v_al[:, 0], v_al[:, 1], label="OpenVINS (aligned)", lw=1.5)
    ax[0].plot(g[0, 0], g[0, 1], "go")
    ax[0].plot(g[-1, 0], g[-1, 1], "rs")
    ax[0].set_aspect("equal", adjustable="datalim")
    ax[0].set_title("top-down XY  (green = start, red = end)")
    ax[0].set_xlabel("x [m]")
    ax[0].set_ylabel("y [m]")
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[1].plot(t - t[0], g[:, 2], label="reference")
    ax[1].plot(t - t[0], v_al[:, 2], label="OpenVINS (aligned)")
    ax[1].set_title("height")
    ax[1].set_xlabel("time [s]")
    ax[1].set_ylabel("z [m]")
    ax[1].legend()
    ax[1].grid(alpha=0.3)
    ax[2].plot(t - t[0], e3, label="3D error")
    ax[2].plot(t - t[0], eh, label="horizontal")
    ax[2].set_title("error over time")
    ax[2].set_xlabel("time [s]")
    ax[2].set_ylabel("[m]")
    ax[2].legend()
    ax[2].grid(alpha=0.3)
    fig.tight_layout()
    png = out_prefix + ".png"
    fig.savefig(png, dpi=130)
    print("plot saved to", png)


def record(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    # sim clock, so --stamp arrival matches the simulator
    node = Node("vio_vs_gt", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    vio, gt = [], []

    def make_cb(buf, min_dt):
        last = [-1e18]

        def cb(m):
            if args.stamp == "arrival":
                ts = node.get_clock().now().nanoseconds * 1e-9
            else:
                ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if ts - last[0] < min_dt:
                return
            last[0] = ts
            p = m.pose.pose.position
            buf.append((ts, p.x, p.y, p.z))
        return cb

    # best-effort QoS receives from reliable and best-effort publishers
    node.create_subscription(Odometry, args.vio, make_cb(vio, 0.0), qos_profile_sensor_data)
    node.create_subscription(Odometry, args.gt, make_cb(gt, 0.01), qos_profile_sensor_data)

    stem = args.out or os.path.join(log_dir(), time.strftime("vio_run_%Y%m%d_%H%M%S"))
    stop = [False]
    # Ctrl-C only sets a flag: KeyboardInterrupt inside a message conversion crashes rclpy
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))

    def checkpoint():
        if vio and gt:
            np.savez(stem + ".npz", vio=np.array(vio), gt=np.array(gt), gt_topic=args.gt)

    print(f"recording  VIO: {args.vio}   reference: {args.gt}   (Ctrl-C to stop; autosave every 15 s)")
    t0 = time.monotonic()
    next_report = t0 + 5.0
    next_save = t0 + 15.0
    errors = 0
    while not stop[0]:
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
            errors = 0
        except Exception as e:  # keep the recording on spin errors
            errors += 1
            if stop[0] or errors > 20:
                if not stop[0]:
                    print("spin_once keeps failing, stopping:", e)
                break
            continue
        now = time.monotonic()
        if now >= next_report:
            print(f"  {now - t0:5.0f}s   VIO samples: {len(vio):5d}   reference samples: {len(gt):5d}")
            if now - t0 < 6.5:
                if not vio:
                    print("  ! nothing on", args.vio, "- has OpenVINS initialised yet?")
                if not gt:
                    print("  ! nothing on", args.gt, "- is that topic bridged?")
            next_report = now + 5.0
        if now >= next_save:
            checkpoint()
            next_save = now + 15.0
        if args.duration and now - t0 >= args.duration:
            break
    print("\nstopping, saving...")
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass

    vio_a, gt_a = np.array(vio), np.array(gt)
    if len(vio_a) == 0 or len(gt_a) == 0:
        raise SystemExit("Nothing recorded on one of the topics; nothing to compare.")
    np.savez(stem + ".npz", vio=vio_a, gt=gt_a, gt_topic=args.gt)
    print("raw data saved to", stem + ".npz")
    analyse(vio_a, gt_a, stem)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vio", default="/ov_msckf/odomimu", help="OpenVINS odometry topic")
    ap.add_argument("--gt", default="/ground_truth/odom", help="reference odometry topic")
    ap.add_argument("--stamp", choices=["header", "arrival"], default="header",
                    help="use message header stamps, or arrival time on the sim clock")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until Ctrl-C)")
    ap.add_argument("--out", default="", help="output file prefix (default: <flight_logs>/vio_run_<time>)")
    ap.add_argument("--analyze", default="", help="analyse a saved .npz instead of recording")
    args = ap.parse_args()
    if args.analyze:
        d = np.load(args.analyze)
        analyse(d["vio"], d["gt"], args.out or args.analyze.rsplit(".", 1)[0])
    else:
        record(args)


if __name__ == "__main__":
    sys.exit(main())