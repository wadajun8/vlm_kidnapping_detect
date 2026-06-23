#!/usr/bin/python3

# SPDX-FileCopyrightText: 2026 Junya Wada
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Superposition(Node):
    def __init__(self):
        super().__init__("superposition")
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.image_pub = self.create_publisher(Image, '/vlm_context_image', 10)

        self.cv_bridge = CvBridge()
        self.get_logger().info('起動')

    def map_callback(self, msg):
        self.get_logger().info('マップ受信')
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        img = np.zeros((msg.info.height, msg.info.width, 3), dtype=np.uint8)

        img[grid == 0] = (255, 255, 255)
        img[grid == 100] = (0, 0, 0)
        img[grid == -1] = (200, 200, 200)

        map_image = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_pub.publish(map_image)

def main(args=None):
    rclpy.init(args=args)
    node = Superposition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
