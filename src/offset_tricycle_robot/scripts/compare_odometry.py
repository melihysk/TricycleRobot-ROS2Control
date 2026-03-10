#!/usr/bin/env python3
"""
Compare 4 odometry sources using evo (APE, RPE, evo_traj, evo_res).

Topics:
  - /kiss/odometry           (KISS-ICP LiDAR odometry)
  - /odom_scan_matcher      (laser_scan_matcher)
  - /odom_rf2o              (RF2O laser odometry)
  - /model/forklift/odometry (Gazebo ground truth – reference)

Requires: pip install evo
For saving trajectory/result plot PDFs: pip install PyQt6 (or use --no_plot to skip plots)

Usage:
  # 1) Simülasyonu ve robotu çalıştır, sonra kayıt (ör. 60 saniye veya Ctrl+C ile durdur)
  ros2 run offset_tricycle_robot compare_odometry.py record --duration 60 --output_dir ./odom_compare

  # 2) Sadece mevcut TUM dosyaları ile değerlendirme
  ros2 run offset_tricycle_robot compare_odometry.py evaluate --input_dir ./odom_compare

  # 3) Kayıt bitince otomatik değerlendirme (record modunda --evaluate ekle)
  ros2 run offset_tricycle_robot compare_odometry.py record --duration 60 --evaluate
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


# Topic -> short name for filenames; reference is ground truth
ODOM_TOPICS = {
    "/model/forklift/odometry": "ground_truth",
    "/kiss/odometry": "kiss_odometry",
    "/odom_scan_matcher": "odom_scan_matcher",
    "/odom_rf2o": "odom_rf2o",
}
REFERENCE_TOPIC = "/model/forklift/odometry"
ESTIMATE_TOPICS = [t for t in ODOM_TOPICS if t != REFERENCE_TOPIC]


def stamp_to_seconds(stamp) -> float:
    """Convert ROS2 builtin_interfaces/Time to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def write_tum(path: Path, rows: list[tuple]) -> None:
    """Write TUM format: timestamp x y z qx qy qz qw (one line per pose)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        for t, x, y, z, qx, qy, qz, qw in sorted(rows, key=lambda r: r[0]):
            f.write(f"{t:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")


def run_cmd(cmd: list[str], cwd: Path | None = None, env: dict | None = None) -> bool:
    """Run command; return True on success."""
    try:
        run_env = os.environ.copy()
        if env:
            run_env.update(env)
        # Headless Qt so evo can save PDFs without opening a window (requires PyQt6)
        run_env.setdefault("QT_QPA_PLATFORM", "offscreen")
        subprocess.run(cmd, cwd=cwd or None, env=run_env, check=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {' '.join(cmd)}", file=sys.stderr)
        return False
    except FileNotFoundError:
        print("evo not found. Install with: pip install evo", file=sys.stderr)
        return False


class OdomRecorder(Node):
    """Subscribe to 4 odometry topics and buffer poses for TUM export."""

    def __init__(self, output_dir: Path):
        super().__init__("compare_odometry_recorder")
        self.output_dir = Path(output_dir)
        self.data: dict[str, list[tuple]] = {name: [] for name in ODOM_TOPICS.values()}
        for topic, name in ODOM_TOPICS.items():
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, n=name: self._cb(msg, n),
                10,
            )
        self.get_logger().info(
            f"Recording odometry to {self.output_dir}: {list(ODOM_TOPICS.keys())}"
        )

    def _cb(self, msg: Odometry, name: str) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = stamp_to_seconds(msg.header.stamp)
        self.data[name].append((t, p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    def save_tum(self) -> dict[str, Path]:
        """Write buffered poses to TUM files. Returns map name -> path."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        paths = {}
        for name, rows in self.data.items():
            if not rows:
                print(f"No data for {name}", file=sys.stderr)
                continue
            path = self.output_dir / f"{name}.tum"
            write_tum(path, rows)
            paths[name] = path
            print(f"Saved {len(rows)} poses -> {path}")
        return paths


def main_record(args) -> Path | None:
    """Record odometry to TUM files; optionally run evaluate afterwards."""
    rclpy.init()
    node = OdomRecorder(Path(args.output_dir))
    try:
        if args.duration and args.duration > 0:
            import time
            node.get_logger().info(f"Recording for {args.duration} s...")
            end = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.5)
        else:
            node.get_logger().info("Recording until Ctrl+C...")
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    paths = node.save_tum()
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass  # context may already be shutting down after Ctrl+C
    if not paths:
        print("No data recorded.", file=sys.stderr)
        return None
    if args.evaluate:
        return Path(args.output_dir)
    return None


def main_evaluate(args) -> None:
    """Run evo_ape, evo_rpe, evo_traj, evo_res on TUM files in input_dir."""
    base = Path(args.input_dir).resolve()
    ref_path = base / "ground_truth.tum"
    if not ref_path.exists():
        print(f"Reference trajectory not found: {ref_path}", file=sys.stderr)
        sys.exit(1)

    estimate_names = [ODOM_TOPICS[t] for t in ESTIMATE_TOPICS]
    estimate_paths = [base / f"{n}.tum" for n in estimate_names]
    missing = [p for p in estimate_paths if not p.exists()]
    if missing:
        print(f"Missing trajectories: {missing}", file=sys.stderr)
        sys.exit(1)

    # 1) evo_traj: plot all trajectories (--save_plot only, no -p to avoid PyQt6 GUI)
    traj_cmd = [
        "evo_traj", "tum", str(ref_path),
        *[str(p) for p in estimate_paths],
        "--ref", str(ref_path),
        "--save_plot", str(base / "trajectories.pdf"),
    ]
    if args.no_plot:
        traj_cmd = ["evo_traj", "tum", str(ref_path), *[str(p) for p in estimate_paths]]
    print("Running: evo_traj ...")
    run_cmd(traj_cmd)

    # 2) evo_ape for each estimate
    ape_zips = []
    for est_path in estimate_paths:
        name = est_path.stem
        zip_path = base / f"ape_{name}.zip"
        ape_cmd = [
            "evo_ape", "tum", str(ref_path), str(est_path),
            "--align",
            "--save_results", str(zip_path),
        ]
        print(f"Running: evo_ape (ref vs {name}) ...")
        if run_cmd(ape_cmd):
            ape_zips.append(zip_path)

    # 3) evo_rpe for each estimate
    rpe_zips = []
    for est_path in estimate_paths:
        name = est_path.stem
        zip_path = base / f"rpe_{name}.zip"
        rpe_cmd = [
            "evo_rpe", "tum", str(ref_path), str(est_path),
            "--pose_relation", "angle_deg",
            "--delta", "0.25", "--delta_unit", "m",
            "--save_results", str(zip_path),
        ]
        print(f"Running: evo_rpe (ref vs {name}) ...")
        if run_cmd(rpe_cmd):
            rpe_zips.append(zip_path)

    # 4) evo_res: compare APE and RPE results (needs PyQt6 for PDF; use --no_plot to skip)
    if not args.no_plot:
        if ape_zips:
            print("Running: evo_res (APE comparison) ...")
            run_cmd(
                ["evo_res", *[str(z) for z in ape_zips], "--save_plot", str(base / "ape_comparison.pdf")]
            )
        if rpe_zips:
            print("Running: evo_res (RPE comparison) ...")
            run_cmd(
                ["evo_res", *[str(z) for z in rpe_zips], "--save_plot", str(base / "rpe_comparison.pdf")]
            )

    print(f"Results in: {base}")


def main():
    parser = argparse.ArgumentParser(
        description="Compare 4 odometry sources with evo (APE, RPE, evo_traj, evo_res)."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    rec = sub.add_parser("record", help="Record odometry from ROS topics to TUM files")
    rec.add_argument("--output_dir", default="./odom_compare", help="Output directory for TUM files")
    rec.add_argument("--duration", type=float, default=0, help="Record duration in seconds (0 = until Ctrl+C)")
    rec.add_argument("--evaluate", action="store_true", help="Run evo evaluation after recording")
    rec.add_argument("--no_plot", action="store_true", help="Skip evo_traj/evo_res plots (when using --evaluate)")
    rec.set_defaults(func=main_record)

    ev = sub.add_parser("evaluate", help="Run evo on existing TUM files")
    ev.add_argument("--input_dir", default="./odom_compare", help="Directory containing ground_truth.tum and others")
    ev.add_argument("--no_plot", action="store_true", help="Skip evo_traj plot")
    ev.set_defaults(func=main_evaluate)

    args = parser.parse_args()
    if args.command == "record":
        out_dir = args.func(args)
        if out_dir and getattr(args, "evaluate", False):
            main_evaluate(argparse.Namespace(input_dir=str(out_dir), no_plot=getattr(args, "no_plot", False)))
    else:
        args.func(args)


if __name__ == "__main__":
    main()
