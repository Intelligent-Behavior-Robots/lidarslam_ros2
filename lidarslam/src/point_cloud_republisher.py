#!/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2

class PointCloudRepublisher(Node):

    def __init__(self):
        super().__init__("point_cloud_republisher")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            depth=10
        )        
        self.publisher_ = self.create_publisher(PointCloud2, "/republished_points", qos_profile)
        self.subscription_ = self.create_subscription(PointCloud2, "/points", self.callback, qos_profile)

    def callback(self, msg):
        # Establece el tiempo del mensaje al momento de la publicación
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
