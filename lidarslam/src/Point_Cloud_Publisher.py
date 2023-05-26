#!/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import open3d as o3d


class PointCloudPublisherNode(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_point_cloud)

    def publish_point_cloud(self):
        # Leer el archivo PCD
        pcd = o3d.io.read_point_cloud('map.pcd')

        # Obtener los datos de la nube de puntos como un array de NumPy
        point_cloud_np = np.asarray(pcd.points)

        # Crear el mensaje PointCloud2
        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = point_cloud_np.shape[0]
        msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = point_cloud_np.astype(np.float32).tobytes()
        msg.is_dense = True

        num = 16*msg.width*msg.height
        self.get_logger().info("msg.data size: %d" % num)
        self.get_logger().info("longitud mensaje: %d" % len(msg.data))
        

        # Publicar el mensaje
        self.publisher.publish(msg)
        self.get_logger().info("Published PointCloud2 message")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
