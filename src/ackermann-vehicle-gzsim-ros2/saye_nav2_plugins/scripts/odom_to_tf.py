#!/usr/bin/env python3
"""
可靠的 odom→saye TF 发布器。
启动时立即 (__init__ 中) 发 static identity, 确保第一条 scan 到达前 TF 已就绪。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)

        # 立即发静态 identity, 覆盖 t=0→∞, 零延迟
        sbr = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id = 'odom'
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = 'saye'
        t.transform.rotation.w = 1.0
        sbr.sendTransform(t)
        self.get_logger().info('Static odom->saye identity sent')

        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)

    def callback(self, msg):
        t = TransformStamped()
        t.header.frame_id = 'odom'
        t.header.stamp = msg.header.stamp
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomToTF())


if __name__ == '__main__':
    main()
