#!/usr/bin/env python3
"""
Relay /cmd_vel (Twist) -> /offset_tricycle_controller/cmd_vel (TwistStamped).
Controller TwistStamped beklediği için dönüşüm yapılıyor.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self.pub = self.create_publisher(
            TwistStamped, "/offset_tricycle_controller/cmd_vel", 10
        )
        self.create_subscription(Twist, "/cmd_vel", self.cb, 10)

    def cb(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
