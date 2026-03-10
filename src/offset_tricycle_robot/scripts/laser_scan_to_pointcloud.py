#!/usr/bin/env python3
"""
2D LaserScan (sensor_msgs/LaserScan) → 3D PointCloud2 (sensor_msgs/PointCloud2) dönüştürücü.

KISS-ICP gibi 3D LiDAR odometri algoritmaları PointCloud2 bekler.
Bu node, tek düzlemdeki 2D taramayı aynı düzlemde z=0 ile 3D nokta bulutuna çevirir
(lidar_link frame'inde).
"""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def laser_scan_to_pointcloud2(scan: LaserScan) -> PointCloud2:
    """LaserScan mesajını PointCloud2'ye dönüştürür (lidar düzleminde z=0)."""
    points = []
    angle = float(scan.angle_min)
    for r in scan.ranges:
        if not (math.isfinite(r) and scan.range_min <= r <= scan.range_max):
            angle += scan.angle_increment
            continue
        # Lidar frame: x öne, y sola (ROS), z yukarı. Yatay düzlemde: x = r*cos(angle), y = r*sin(angle)
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        z = 0.0
        points.append((x, y, z))
        angle += scan.angle_increment

    # PointCloud2: x,y,z float32
    msg = PointCloud2()
    msg.header = scan.header
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = True
    msg.point_step = 12  # 3 * 4 bytes
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = b''.join(struct.pack('fff', x, y, z) for x, y, z in points)
    return msg


class LaserScanToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laser_scan_to_pointcloud')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('output_topic', 'lidar/points')
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.cb_scan,
            10,
        )
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.get_logger().info(
            f'LaserScan -> PointCloud2: {scan_topic} -> {output_topic}'
        )

    def cb_scan(self, msg: LaserScan):
        cloud = laser_scan_to_pointcloud2(msg)
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
