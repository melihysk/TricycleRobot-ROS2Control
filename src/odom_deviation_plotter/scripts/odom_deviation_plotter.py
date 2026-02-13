#!/usr/bin/env python3
"""
Real-time odometry deviation plotter.

Subscribes to:
  - Ground truth odometry (Gazebo)
  - Wheel odometry (controller)
  - EKF filtered odometry
  - cmd_vel (TwistStamped)

Plots:
  1. cmd_vel linear & angular velocity over time
  2. Euclidean position deviation from ground truth
  3. Per-axis (dx, dy) position deviation
  4. Yaw (heading) deviation from ground truth
"""

import math
import threading
from collections import deque

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


# ---------------------------------------------------------------------------
# Quaternion -> Yaw helper (avoids tf_transformations dependency)
# ---------------------------------------------------------------------------
def quaternion_to_yaw(q):
    """Extract yaw (rotation about Z) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------
class OdomDeviationPlotter(Node):
    def __init__(self):
        super().__init__('odom_deviation_plotter')

        # -- Declare parameters --
        self.declare_parameter('wheel_odom_topic', '/tricycle_controller/odom')
        self.declare_parameter('cmd_vel_topic', '/tricycle_controller/cmd_vel')
        self.declare_parameter('ground_truth_topic', '/model/gebot/odometry')
        self.declare_parameter('ekf_odom_topic', '/odometry/filtered')
        self.declare_parameter('window_size', 2500)

        wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        gt_topic = self.get_parameter('ground_truth_topic').value
        ekf_topic = self.get_parameter('ekf_odom_topic').value
        win = self.get_parameter('window_size').value

        self.get_logger().info(f'Wheel odom topic : {wheel_odom_topic}')
        self.get_logger().info(f'EKF odom topic   : {ekf_topic}')
        self.get_logger().info(f'Ground truth topic: {gt_topic}')
        self.get_logger().info(f'cmd_vel topic    : {cmd_vel_topic}')
        self.get_logger().info(f'Window size      : {win}')

        # -- Data buffers (thread-safe via deque) --
        self.lock = threading.Lock()

        # Time reference (first ground truth timestamp)
        self._t0 = None

        # cmd_vel
        self.t_cmd = deque(maxlen=win)
        self.cmd_lin = deque(maxlen=win)
        self.cmd_ang = deque(maxlen=win)

        # Wheel odom deviation
        self.t_wheel = deque(maxlen=win)
        self.wheel_eucl = deque(maxlen=win)
        self.wheel_dx = deque(maxlen=win)
        self.wheel_dy = deque(maxlen=win)
        self.wheel_dyaw = deque(maxlen=win)

        # EKF deviation
        self.t_ekf = deque(maxlen=win)
        self.ekf_eucl = deque(maxlen=win)
        self.ekf_dx = deque(maxlen=win)
        self.ekf_dy = deque(maxlen=win)
        self.ekf_dyaw = deque(maxlen=win)

        # Latest ground truth pose
        self._gt_x = 0.0
        self._gt_y = 0.0
        self._gt_yaw = 0.0
        self._gt_received = False

        # Throttle: minimum interval between recorded samples (seconds)
        self._min_dt = 0.02  # 50 Hz max record rate
        self._last_wheel_t = 0.0
        self._last_ekf_t = 0.0
        self._last_cmd_t = 0.0

        # -- Subscribers --
        self.create_subscription(Odometry, gt_topic, self._gt_cb, 10)
        self.create_subscription(Odometry, wheel_odom_topic, self._wheel_cb, 10)
        self.create_subscription(Odometry, ekf_topic, self._ekf_cb, 10)
        self.create_subscription(TwistStamped, cmd_vel_topic, self._cmd_cb, 10)

        self.get_logger().info('Odometry deviation plotter node started.')

    # ---- helpers ----
    def _stamp_to_sec(self, stamp):
        """Convert a ROS stamp to seconds relative to t0."""
        t = stamp.sec + stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    # ---- callbacks ----
    def _gt_cb(self, msg: Odometry):
        with self.lock:
            self._gt_x = msg.pose.pose.position.x
            self._gt_y = msg.pose.pose.position.y
            self._gt_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            self._gt_received = True

    def _wheel_cb(self, msg: Odometry):
        with self.lock:
            if not self._gt_received:
                return
            t = self._stamp_to_sec(msg.header.stamp)
            if t - self._last_wheel_t < self._min_dt:
                return
            self._last_wheel_t = t

            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            oyaw = quaternion_to_yaw(msg.pose.pose.orientation)

            dx = ox - self._gt_x
            dy = oy - self._gt_y
            eucl = math.sqrt(dx * dx + dy * dy)
            dyaw = math.degrees(normalize_angle(oyaw - self._gt_yaw))

            self.t_wheel.append(t)
            self.wheel_eucl.append(eucl)
            self.wheel_dx.append(dx)
            self.wheel_dy.append(dy)
            self.wheel_dyaw.append(dyaw)

    def _ekf_cb(self, msg: Odometry):
        with self.lock:
            if not self._gt_received:
                return
            t = self._stamp_to_sec(msg.header.stamp)
            if t - self._last_ekf_t < self._min_dt:
                return
            self._last_ekf_t = t

            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            oyaw = quaternion_to_yaw(msg.pose.pose.orientation)

            dx = ox - self._gt_x
            dy = oy - self._gt_y
            eucl = math.sqrt(dx * dx + dy * dy)
            dyaw = math.degrees(normalize_angle(oyaw - self._gt_yaw))

            self.t_ekf.append(t)
            self.ekf_eucl.append(eucl)
            self.ekf_dx.append(dx)
            self.ekf_dy.append(dy)
            self.ekf_dyaw.append(dyaw)

    def _cmd_cb(self, msg: TwistStamped):
        with self.lock:
            t = self._stamp_to_sec(msg.header.stamp)
            if t - self._last_cmd_t < self._min_dt:
                return
            self._last_cmd_t = t

            self.t_cmd.append(t)
            self.cmd_lin.append(msg.twist.linear.x)
            self.cmd_ang.append(msg.twist.angular.z)

    # ---- snapshot for plotting (copies under lock) ----
    def snapshot(self):
        with self.lock:
            return {
                't_cmd': list(self.t_cmd),
                'cmd_lin': list(self.cmd_lin),
                'cmd_ang': list(self.cmd_ang),
                't_wheel': list(self.t_wheel),
                'wheel_eucl': list(self.wheel_eucl),
                'wheel_dx': list(self.wheel_dx),
                'wheel_dy': list(self.wheel_dy),
                'wheel_dyaw': list(self.wheel_dyaw),
                't_ekf': list(self.t_ekf),
                'ekf_eucl': list(self.ekf_eucl),
                'ekf_dx': list(self.ekf_dx),
                'ekf_dy': list(self.ekf_dy),
                'ekf_dyaw': list(self.ekf_dyaw),
            }


# ---------------------------------------------------------------------------
# Matplotlib real-time plot
# ---------------------------------------------------------------------------
def run_plot(node: OdomDeviationPlotter):
    """Create and animate a matplotlib figure with 4 subplots."""

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Odometry Deviation Plotter', fontsize=14, fontweight='bold')
    fig.subplots_adjust(hspace=0.35, top=0.93, bottom=0.07)

    # --- Subplot 1: cmd_vel ---
    ax_cmd = axes[0]
    ax_cmd.set_title('cmd_vel Komutları')
    ax_cmd.set_ylabel('Hız')
    line_cmd_lin, = ax_cmd.plot([], [], 'b-', linewidth=1.2, label='Lineer x (m/s)')
    ax_cmd_twin = ax_cmd.twinx()
    line_cmd_ang, = ax_cmd_twin.plot([], [], 'r-', linewidth=1.2, label='Açısal z (rad/s)')
    ax_cmd.set_ylabel('Lineer (m/s)', color='b')
    ax_cmd_twin.set_ylabel('Açısal (rad/s)', color='r')
    ax_cmd.tick_params(axis='y', labelcolor='b')
    ax_cmd_twin.tick_params(axis='y', labelcolor='r')
    # Combined legend
    lines_cmd = [line_cmd_lin, line_cmd_ang]
    ax_cmd.legend(lines_cmd, [l.get_label() for l in lines_cmd], loc='upper left', fontsize=8)

    # --- Subplot 2: Euclidean position deviation ---
    ax_eucl = axes[1]
    ax_eucl.set_title('Pozisyon Sapması (Euclidean)')
    ax_eucl.set_ylabel('Mesafe (m)')
    line_w_eucl, = ax_eucl.plot([], [], 'r-', linewidth=1.2, label='Wheel Odom')
    line_e_eucl, = ax_eucl.plot([], [], 'g-', linewidth=1.2, label='EKF Filtered')
    ax_eucl.legend(loc='upper left', fontsize=8)

    # --- Subplot 3: Per-axis deviation ---
    ax_dxy = axes[2]
    ax_dxy.set_title('Eksen Bazlı Pozisyon Sapması')
    ax_dxy.set_ylabel('Sapma (m)')
    line_w_dx, = ax_dxy.plot([], [], 'r-', linewidth=1.0, label='Wheel dx')
    line_w_dy, = ax_dxy.plot([], [], 'r--', linewidth=1.0, label='Wheel dy')
    line_e_dx, = ax_dxy.plot([], [], 'g-', linewidth=1.0, label='EKF dx')
    line_e_dy, = ax_dxy.plot([], [], 'g--', linewidth=1.0, label='EKF dy')
    ax_dxy.legend(loc='upper left', fontsize=8, ncol=2)

    # --- Subplot 4: Yaw deviation ---
    ax_yaw = axes[3]
    ax_yaw.set_title('Yaw (Heading) Sapması')
    ax_yaw.set_ylabel('Sapma (°)')
    ax_yaw.set_xlabel('Zaman (s)')
    line_w_yaw, = ax_yaw.plot([], [], 'r-', linewidth=1.2, label='Wheel Odom')
    line_e_yaw, = ax_yaw.plot([], [], 'g-', linewidth=1.2, label='EKF Filtered')
    ax_yaw.legend(loc='upper left', fontsize=8)

    # Grid for all
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 10)

    def _update(_frame):
        snap = node.snapshot()

        # cmd_vel
        t_c = snap['t_cmd']
        if t_c:
            line_cmd_lin.set_data(t_c, snap['cmd_lin'])
            line_cmd_ang.set_data(t_c, snap['cmd_ang'])
            ax_cmd.relim(); ax_cmd.autoscale_view(scalex=False)
            ax_cmd_twin.relim(); ax_cmd_twin.autoscale_view(scalex=False)

        # Wheel odom deviation
        t_w = snap['t_wheel']
        if t_w:
            line_w_eucl.set_data(t_w, snap['wheel_eucl'])
            line_w_dx.set_data(t_w, snap['wheel_dx'])
            line_w_dy.set_data(t_w, snap['wheel_dy'])
            line_w_yaw.set_data(t_w, snap['wheel_dyaw'])

        # EKF deviation
        t_e = snap['t_ekf']
        if t_e:
            line_e_eucl.set_data(t_e, snap['ekf_eucl'])
            line_e_dx.set_data(t_e, snap['ekf_dx'])
            line_e_dy.set_data(t_e, snap['ekf_dy'])
            line_e_yaw.set_data(t_e, snap['ekf_dyaw'])

        # Auto-adjust x-axis for all subplots — show ALL accumulated data
        all_times = t_c + t_w + t_e
        if all_times:
            t_max = max(all_times)
            for ax in axes:
                ax.set_xlim(0, t_max + 2)

        # Auto-adjust y-axis
        for ax in [ax_eucl, ax_dxy, ax_yaw]:
            ax.relim()
            ax.autoscale_view(scalex=False)

        return (line_cmd_lin, line_cmd_ang,
                line_w_eucl, line_e_eucl,
                line_w_dx, line_w_dy, line_e_dx, line_e_dy,
                line_w_yaw, line_e_yaw)

    _anim = FuncAnimation(fig, _update, interval=250, blit=False, cache_frame_data=False)
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def _spin_node(node):
    """Spin ROS node, suppressing ExternalShutdownException on Ctrl+C."""
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    node = OdomDeviationPlotter()

    # Spin ROS in a background thread so matplotlib can own the main thread
    spin_thread = threading.Thread(target=_spin_node, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
