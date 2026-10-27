#!/usr/bin/env python3

from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from .learned_color import LearnedColorClassifier
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


HSV_RANGES = {
    'blue': [((95, 60, 40), (115, 255, 255))],
    'orange': [((12, 140, 70), (22, 255, 255))],
    'green': [((40, 70, 40), (80, 255, 255))],
    'red': [((0, 100, 50), (4, 255, 255)), ((176, 100, 50), (180, 255, 255))],
    'purple': [((125, 45, 40), (155, 255, 255))],
    'brown': [((5, 60, 25), (10, 255, 185))],
    'pink': [((155, 35, 70), (175, 255, 255))],
    'gray': [((0, 0, 35), (180, 45, 155))],
    'olive': [((25, 60, 40), (45, 255, 255))],
    'cyan': [((80, 50, 50), (100, 255, 255))],
}


class GazeboDeliveryDemo(Node):
    def __init__(self):
        super().__init__('gazebo_delivery_demo')
        self.declare_parameter('target_color', 'blue')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('target_point_topic', '/target_point')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('min_contour_area', 80.0)
        self.declare_parameter('hover_height', 0.15)

        self.target_color = self.get_parameter('target_color').value.lower()
        if self.target_color not in HSV_RANGES:
            raise ValueError(
                f'target_color must be one of: {", ".join(sorted(HSV_RANGES))}'
            )
        self.min_contour_area = float(self.get_parameter('min_contour_area').value)
        self.hover_height = float(self.get_parameter('hover_height').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.bridge = CvBridge()
        self.classifier = LearnedColorClassifier(
            Path(get_package_share_directory('final_project_ros')) / 'models/color_tree.yml')
        self.object_detected = None
        self.last_detection_count = 0
        self.target_point_topic = self.get_parameter('target_point_topic').value

        self.image_sub = self.create_subscription(Image, self.get_parameter('image_topic').value, self.image_callback, 10)
        self.point_pub = self.create_publisher(PointStamped, self.target_point_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.get_logger().info(
            'Learned RGB decision tree ready; Gazebo delivery demo: target_color=%s image_topic=%s min_contour_area=%s' % (
                self.target_color,
                self.get_parameter('image_topic').value,
                self.min_contour_area,
            )
        )

    def image_callback(self, msg: Image):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Image decode failed: {exc}')
            return

        detections = detect_targets(image_bgr, self.target_color, self.min_contour_area, self.classifier)
        self.object_detected = bool(detections)
        if not detections:
            self.last_detection_count = 0
            return

        for center_x, center_y, _rect in detections:
            point = PointStamped()
            point.header = msg.header
            point.point.x = float(center_x)
            point.point.y = float(center_y)
            point.point.z = 0.0
            self.point_pub.publish(point)

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(center_x)
            pose.pose.position.y = float(center_y)
            pose.pose.position.z = float(self.hover_height)
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)

        if len(detections) != self.last_detection_count:
            self.get_logger().info(
                f'Detected {len(detections)} {self.target_color} target(s) in camera image'
            )
        self.last_detection_count = len(detections)


def detect_targets(
    image_bgr: np.ndarray,
    target_color: str,
    min_contour_area: float,
    classifier=None,
) -> list[Tuple[int, int, tuple]]:
    if target_color not in HSV_RANGES:
        raise ValueError(f'Unsupported target_color: {target_color}')

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in HSV_RANGES[target_color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)))

    if classifier is not None:
        mask = classifier.mask(image_bgr, target_color)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_contour_area:
            x, y, w, h = cv2.boundingRect(contour)
            candidates.append((area, x + w // 2, y + h // 2, (x, y, w, h)))

    candidates.sort(key=lambda item: (item[2], item[1]))
    return [(cx, cy, rect) for _area, cx, cy, rect in candidates]


def detect_target(
    image_bgr: np.ndarray,
    target_color: str,
    min_contour_area: float,
) -> Optional[Tuple[int, int, tuple]]:
    detections = detect_targets(image_bgr, target_color, min_contour_area)
    return max(
        detections,
        key=lambda item: item[2][2] * item[2][3],
        default=None,
    )


def main(args=None):
    rclpy.init(args=args)
    node = GazeboDeliveryDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
