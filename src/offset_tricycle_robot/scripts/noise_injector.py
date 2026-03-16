#!/usr/bin/env python3
"""
Sensor noise injector for offline odometry experiments.

Subscribes to clean sensor topics from bag replay and republishes
with configurable Gaussian noise on separate topics, so odometry
algorithms can be tested with different noise levels without re-running
Gazebo.

Input topics  (from bag):
  /imu/data   -> /imu/data/noisy
  /scan       -> /scan/noisy

Parameters:
  imu_gyro_noise_stddev   (double, default 0.0): angular velocity noise [rad/s]
  imu_accel_noise_stddev  (double, default 0.0): linear acceleration noise [m/s²]
  scan_range_noise_stddev (double, default 0.0): range noise per beam [m]
  use_sim_time            (bool,   default true)

Set all stddev values to 0.0 for a noise-free pass-through (baseline experiment).
"""

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


class NoiseInjector(Node):
    def __init__(self):
        super().__init__('noise_injector')

        self.declare_parameter('imu_gyro_noise_stddev',   0.0)
        self.declare_parameter('imu_accel_noise_stddev',  0.0)
        self.declare_parameter('scan_range_noise_stddev', 0.0)

        self.gyro_std  = self.get_parameter('imu_gyro_noise_stddev').value
        self.accel_std = self.get_parameter('imu_accel_noise_stddev').value
        self.scan_std  = self.get_parameter('scan_range_noise_stddev').value

        self.imu_pub  = self.create_publisher(Imu,       '/imu/data/noisy',  10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan/noisy',      10)

        self.create_subscription(Imu,       '/imu/data', self._cb_imu,  10)
        self.create_subscription(LaserScan, '/scan',     self._cb_scan, 10)

        self.get_logger().info(
            f'NoiseInjector started — '
            f'gyro_std={self.gyro_std:.4f} rad/s  '
            f'accel_std={self.accel_std:.4f} m/s²  '
            f'scan_std={self.scan_std:.4f} m'
        )

    def _cb_imu(self, msg: Imu) -> None:
        out = copy.deepcopy(msg)
        if self.gyro_std > 0.0:
            out.angular_velocity.x += float(np.random.normal(0.0, self.gyro_std))
            out.angular_velocity.y += float(np.random.normal(0.0, self.gyro_std))
            out.angular_velocity.z += float(np.random.normal(0.0, self.gyro_std))
        if self.accel_std > 0.0:
            out.linear_acceleration.x += float(np.random.normal(0.0, self.accel_std))
            out.linear_acceleration.y += float(np.random.normal(0.0, self.accel_std))
            out.linear_acceleration.z += float(np.random.normal(0.0, self.accel_std))
        self.imu_pub.publish(out)

    def _cb_scan(self, msg: LaserScan) -> None:
        out = copy.deepcopy(msg)
        if self.scan_std > 0.0:
            ranges = list(out.ranges)
            noise  = np.random.normal(0.0, self.scan_std, len(ranges))
            for i, (r, n) in enumerate(zip(ranges, noise)):
                if msg.range_min <= r <= msg.range_max:
                    noisy_r = r + float(n)
                    ranges[i] = max(msg.range_min, min(msg.range_max, noisy_r))
            out.ranges = ranges
        self.scan_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = NoiseInjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
