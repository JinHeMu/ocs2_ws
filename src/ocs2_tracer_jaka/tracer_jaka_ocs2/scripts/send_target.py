#!/usr/bin/env python3
# =============================================================================
#  send_target.py
#
#  快速发一个 PoseStamped 目标到 /target_pose, 用于命令行测试 OCS2 MPC.
#  示例 :
#    ros2 run tracer_jaka_ocs2 send_target.py 0.8 0.0 0.6 0 0 0
# =============================================================================
import sys
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


def main():
    if len(sys.argv) < 7:
        print("Usage: send_target.py x y z roll pitch yaw")
        sys.exit(1)

    x, y, z, r, p, yw = [float(v) for v in sys.argv[1:7]]

    rclpy.init()
    node = Node('send_target')
    pub = node.create_publisher(PoseStamped, '/target_pose', 10)

    msg = PoseStamped()
    msg.header.frame_id = 'odom'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    q = quaternion_from_euler(r, p, yw)
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    # 发几次保证收到
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(
        f'Sent target: pos=({x:.3f}, {y:.3f}, {z:.3f}) rpy=({r:.3f}, {p:.3f}, {yw:.3f})')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
