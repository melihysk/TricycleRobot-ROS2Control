#!/usr/bin/env python3
"""
Direct Joystick Teleop for Offset Tricycle

Xbox 360 Controller:
  - Left Stick Y (axis 1): Forward/Backward velocity (linear.x)
  - Right Stick X (axis 3): Steering angle

When velocity is 0 but steering is non-zero:
  - Send small angular.z to make controller turn wheels
  - This allows steering in place without moving

Right stick right -> turn right (positive steering)
Right stick left -> turn left (negative steering)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import math


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        
        # Parameters
        self.declare_parameter('max_linear_velocity', 0.8)
        self.declare_parameter('max_steering_angle', 1.5708)
        self.declare_parameter('wheelbase', 0.975)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('throttle_axis', 1)
        self.declare_parameter('steering_axis', 3)
        
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.deadzone = self.get_parameter('deadzone').value
        self.throttle_axis = self.get_parameter('throttle_axis').value
        self.steering_axis = self.get_parameter('steering_axis').value
        
        # Publisher - TwistStamped!
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/offset_tricycle_controller/cmd_vel', 10)
        
        # Subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # State
        self.linear_vel = 0.0
        self.steering_input = 0.0  # -1 to 1
        
        # Timer for publishing
        self.timer = self.create_timer(0.02, self.publish_cmd)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Joystick Teleop for Offset Tricycle')
        self.get_logger().info('  Left Stick Y: Forward/Backward')
        self.get_logger().info('  Right Stick X: Steering (right=right turn)')
        self.get_logger().info('  Steering works even when stopped!')
        self.get_logger().info('='*60)
    
    def apply_deadzone(self, val):
        if abs(val) < self.deadzone:
            return 0.0
        return (val - math.copysign(self.deadzone, val)) / (1.0 - self.deadzone)
    
    def joy_callback(self, msg):
        # Throttle from left stick Y
        if len(msg.axes) > self.throttle_axis:
            self.linear_vel = self.apply_deadzone(msg.axes[self.throttle_axis]) * self.max_linear_vel
        
        # Steering from right stick X
        # Invert so right stick right = positive angle = turn right
        if len(msg.axes) > self.steering_axis:
            raw_steering = -msg.axes[self.steering_axis]  # Invert
            self.steering_input = self.apply_deadzone(raw_steering)
    
    def publish_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = self.linear_vel
        
        # Calculate desired steering angle
        steering_angle = self.steering_input * self.max_steering
        
        if abs(self.linear_vel) > 0.01:
            # Moving: compute angular velocity from steering
            # angular_vel = linear_vel * tan(steering) / wheelbase
            if abs(steering_angle) > 0.01:
                # Clamp steering to avoid tan(90) = infinity
                clamped_steering = max(-1.55, min(1.55, steering_angle))
                msg.twist.angular.z = max(-0.7, min(0.7, -(self.linear_vel * math.tan(clamped_steering) / self.wheelbase)))
            else:
                msg.twist.angular.z = 0.0
        else:
            # Not moving but steering requested
            # Send a "spin" command to make controller turn wheels
            # Controller interprets Vx=0, theta_dot!=0 as spin-in-place
            # and sets steering to ±90°
            if abs(self.steering_input) > 0.01:
                # Scale angular velocity by steering input
                # This tells controller to turn wheels
                msg.twist.angular.z = max(-0.7, min(0.7, -self.steering_input * 2.0))
            else:
                msg.twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(msg)
        
        # Debug
        if abs(self.linear_vel) > 0.01 or abs(self.steering_input) > 0.01:
            self.get_logger().info(
                f'Vel: {self.linear_vel:+5.2f} m/s | '
                f'SteerIn: {self.steering_input:+5.2f} | '
                f'Ang: {msg.twist.angular.z:+5.2f} rad/s',
                throttle_duration_sec=0.2
            )


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        node.cmd_vel_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
