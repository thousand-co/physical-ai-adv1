#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ColorPickDemo(Node):
    def __init__(self):
        super().__init__('color_pick_demo')
        self.declare_parameter('target_color', 'blue')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('target_point_topic', '/target_point')
        self.declare_parameter('task_state_topic', '/task_state')
        self.declare_parameter('min_contour_area', 500.0)

        self.bridge = CvBridge()
        self.target_color = self.get_parameter('target_color').value.lower()
        self.min_contour_area = float(self.get_parameter('min_contour_area').value)

        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            10,
        )
        self.point_pub = self.create_publisher(
            PointStamped,
            self.get_parameter('target_point_topic').value,
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            self.get_parameter('task_state_topic').value,
            10,
        )

        self.get_logger().info(
            f'Color delivery demo initialized. target_color={self.target_color}, '
            f'image_topic={self.get_parameter("image_topic").value}'
        )

    def image_callback(self, msg: Image):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pragma: no cover - defensive guard
            self.get_logger().warn(f'Failed to decode image: {exc}')
            return

        detection = detect_target_color(image_bgr, self.target_color, self.min_contour_area)
        if detection is None:
            self.state_pub.publish(String(data='waiting_for_object'))
            return

        cx, cy, rect = detection
        point = PointStamped()
        point.header = msg.header
        point.point.x = float(cx)
        point.point.y = float(cy)
        point.point.z = 0.0
        self.point_pub.publish(point)

        self.state_pub.publish(String(data='target_detected'))
        self.get_logger().info(
            f'target {self.target_color} detected at center=({cx}, {cy}) '
            f'box={rect}'
        )


def detect_target_color(image_bgr, target_color: str, min_contour_area: float):
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    ranges = {
        'blue': [((90, 60, 60), (130, 255, 255))],
        'red': [((0, 80, 80), (15, 255, 255)), ((165, 80, 80), (180, 255, 255))],
        'green': [((35, 60, 60), (90, 255, 255))],
        'yellow': [((15, 60, 60), (35, 255, 255))],
    }
    if target_color not in ranges:
        raise ValueError(f'Unsupported color: {target_color}')

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges[target_color]:
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(
                hsv,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            ),
        )

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_contour_area:
            x, y, w, h = cv2.boundingRect(contour)
            candidates.append((area, x + w // 2, y + h // 2, (x, y, w, h)))

    if not candidates:
        return None

    _, cx, cy, rect = max(candidates, key=lambda item: item[0])
    return cx, cy, rect


def main(args=None):
    rclpy.init(args=args)
    node = ColorPickDemo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
