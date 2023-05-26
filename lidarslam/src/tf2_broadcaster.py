#!/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import rclpy.duration


class Tf2BroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf2_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.1, self.publish_transform)

    def publish_transform(self):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = (self.get_clock().now() + rclpy.duration.Duration(seconds=0.15)).to_msg() #para que coincida con el tiempo del mensaje de la nube de puntos
        #transform.header.stamp = (self.get_clock().now()).to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_data_frame'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = Tf2BroadcasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
