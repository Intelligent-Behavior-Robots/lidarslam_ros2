#!/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d


class PointCloudSubscriberNode(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'map',
            self.point_cloud_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def point_cloud_callback(self, msg):
        # Convert PointCloud2 message to numpy array
        point_cloud_np = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        
        length = len(point_cloud_np)
        self.get_logger().info("Received PointCloud2 message with %d points" % length)

        # Create Open3D PointCloud object from numpy array
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud_np[:, :3])

        # Save PointCloud to PCD file
        o3d.io.write_point_cloud('map.pcd', pcd)
        self.get_logger().info("Saved PointCloud to map.pcd")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()