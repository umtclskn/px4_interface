#!/usr/bin/env python3
"""Publish a straight nav_msgs/Path for goal_follower axis-mapping tests."""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TestPathPublisher(Node):
    def __init__(self, axis: str, length: float, world_frame: str, body_frame: str):
        super().__init__('publish_test_path')
        self._axis = axis
        self._length = length
        self._world_frame = world_frame
        self._body_frame = body_frame
        self._pub = self.create_publisher(Path, 'path', 1)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(1.0, self._on_timer)

    def _on_timer(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._body_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'TF unavailable: {exc}')
            return

        x0 = tf.transform.translation.x
        y0 = tf.transform.translation.y
        z0 = tf.transform.translation.z
        dx, dy = 0.0, 0.0
        if self._axis == 'x':
            dx = self._length
        elif self._axis == 'y':
            dy = self._length
        elif self._axis == '-x':
            dx = -self._length
        elif self._axis == '-y':
            dy = -self._length
        else:
            self.get_logger().error(f'unsupported axis {self._axis}')
            return

        step = 0.25
        n = max(1, int(math.ceil(self._length / step)))
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._world_frame
        for i in range(n + 1):
            t = min(1.0, (i * step) / self._length) if self._length > 0.0 else 1.0
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x0 + t * dx
            pose.pose.position.y = y0 + t * dy
            pose.pose.position.z = z0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._pub.publish(path)
        self.get_logger().info(
            f'published path axis={self._axis} length={self._length:.2f} '
            f'points={len(path.poses)}')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--axis', default='x', choices=['x', 'y', '-x', '-y'])
    parser.add_argument('--length', type=float, default=2.0)
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--body-frame', default='body')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TestPathPublisher(
        args.axis, args.length, args.world_frame, args.body_frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
