#!/usr/bin/env python3
"""
Sensor noise injector for offline odometry experiments.

Subscribes to clean sensor topics from bag replay and republishes
with configurable Gaussian noise on separate topics, so odometry
algorithms can be tested with different noise levels without re-running
Gazebo.

Input topics  (from bag):
  /imu/data                          -> /imu/data/noisy
  /scan                              -> /scan/noisy
  /offset_tricycle_controller/odom   -> /offset_tricycle_controller/odom/noisy
                                     -> TF: odom_wheel_noisy -> base_link

For wheel odometry noise the clean velocity is corrupted with Gaussian noise and
then integrated via dead reckoning so the published pose drifts realistically.
Kinematic-ICP should use wheel_odom_frame: odom_wheel_noisy to receive this
noisy constraint instead of the clean odom_wheel TF from the bag.

Parameters:
  imu_gyro_noise_stddev   (double, default 0.0): angular velocity noise [rad/s]
  imu_accel_noise_stddev  (double, default 0.0): linear acceleration noise [m/s²]
  scan_range_noise_stddev (double, default 0.0): range noise per beam [m]
  odom_vx_stddev          (double, default 0.0): wheel odom forward velocity noise [m/s]
  odom_vyaw_stddev        (double, default 0.0): wheel odom yaw rate noise [rad/s]
  use_sim_time            (bool,   default true)

Set all stddev values to 0.0 for a noise-free pass-through (baseline experiment).
"""

import copy
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster


def _yaw_from_quat(q) -> float:
    """Extract yaw from a quaternion (assumes 2D motion, rotation around Z)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw: float):
    """Return (x, y, z, w) quaternion for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class NoiseInjector(Node):
    def __init__(self):
        super().__init__('noise_injector')

        self.declare_parameter('imu_gyro_noise_stddev',   0.0)
        self.declare_parameter('imu_accel_noise_stddev',  0.0)
        self.declare_parameter('scan_range_noise_stddev', 0.0)
        self.declare_parameter('odom_vx_stddev',          0.0)
        self.declare_parameter('odom_vyaw_stddev',        0.0)

        self.gyro_std     = self.get_parameter('imu_gyro_noise_stddev').value
        self.accel_std    = self.get_parameter('imu_accel_noise_stddev').value
        self.scan_std     = self.get_parameter('scan_range_noise_stddev').value
        self.odom_vx_std  = self.get_parameter('odom_vx_stddev').value
        self.odom_vyaw_std = self.get_parameter('odom_vyaw_stddev').value

        self.imu_pub  = self.create_publisher(Imu,       '/imu/data/noisy',  10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan/noisy',      10)
        self.odom_pub = self.create_publisher(
            Odometry, '/offset_tricycle_controller/odom/noisy', 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        # Dead reckoning state for noisy wheel odometry integration
        self._last_stamp_sec: float | None = None
        self._noisy_x   = 0.0
        self._noisy_y   = 0.0
        self._noisy_yaw = 0.0

        self.create_subscription(Imu,       '/imu/data', self._cb_imu,  10)
        self.create_subscription(LaserScan, '/scan',     self._cb_scan, 10)
        self.create_subscription(
            Odometry,
            '/offset_tricycle_controller/odom',
            self._cb_odom,
            10,
        )

        self.get_logger().info(
            f'NoiseInjector started — '
            f'gyro_std={self.gyro_std:.4f} rad/s  '
            f'accel_std={self.accel_std:.4f} m/s²  '
            f'scan_std={self.scan_std:.4f} m  '
            f'odom_vx_std={self.odom_vx_std:.4f} m/s  '
            f'odom_vyaw_std={self.odom_vyaw_std:.4f} rad/s'
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

    def _cb_odom(self, msg: Odometry) -> None:
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ── Noise-free pass-through ──────────────────────────────────────────
        if self.odom_vx_std == 0.0 and self.odom_vyaw_std == 0.0:
            self.odom_pub.publish(copy.deepcopy(msg))
            self._publish_odom_tf(msg.header.stamp, msg.pose.pose)
            self._last_stamp_sec = stamp_sec
            return

        # ── First message: initialise dead reckoning state ───────────────────
        if self._last_stamp_sec is None:
            self._noisy_x   = msg.pose.pose.position.x
            self._noisy_y   = msg.pose.pose.position.y
            self._noisy_yaw = _yaw_from_quat(msg.pose.pose.orientation)
            self._last_stamp_sec = stamp_sec
            self.odom_pub.publish(copy.deepcopy(msg))
            self._publish_odom_tf(msg.header.stamp, msg.pose.pose)
            return

        dt = stamp_sec - self._last_stamp_sec
        self._last_stamp_sec = stamp_sec
        if dt <= 0.0:
            return

        # ── Corrupt velocities and integrate (dead reckoning) ────────────────
        noisy_vx   = msg.twist.twist.linear.x  + float(np.random.normal(0.0, self.odom_vx_std))
        noisy_vyaw = msg.twist.twist.angular.z + float(np.random.normal(0.0, self.odom_vyaw_std))

        self._noisy_x   += noisy_vx * math.cos(self._noisy_yaw) * dt
        self._noisy_y   += noisy_vx * math.sin(self._noisy_yaw) * dt
        self._noisy_yaw += noisy_vyaw * dt

        qx, qy, qz, qw = _quat_from_yaw(self._noisy_yaw)

        out = copy.deepcopy(msg)
        out.pose.pose.position.x      = self._noisy_x
        out.pose.pose.position.y      = self._noisy_y
        out.pose.pose.orientation.x   = qx
        out.pose.pose.orientation.y   = qy
        out.pose.pose.orientation.z   = qz
        out.pose.pose.orientation.w   = qw
        out.twist.twist.linear.x      = noisy_vx
        out.twist.twist.angular.z     = noisy_vyaw
        self.odom_pub.publish(out)
        self._publish_odom_tf(msg.header.stamp, out.pose.pose)

    def _publish_odom_tf(self, stamp, pose) -> None:
        """Publish odom_wheel_noisy -> base_link TF from the given pose."""
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'odom_wheel_noisy'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation      = pose.orientation
        self._tf_broadcaster.sendTransform(t)

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
