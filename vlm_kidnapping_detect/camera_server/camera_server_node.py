#!/usr/bin/python3
# SPDX-FileCopyrightText: 2026 Junya Wada
# SPDX-License-Identifier: BSD-3-Clause

"""USBカメラの画像を /camera/color/image_raw へ配信するノード"""

import os

os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')

from typing import Optional  # noqa: E402

import cv2  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


class CameraServer(Node):
    """カメラから画像を取得して配信する。切断時は一定間隔で再接続を試みる"""

    def __init__(self) -> None:
        super().__init__('camera_server')

        # 使用するカメラは config/camera_server.yaml の video_device で指定する
        self.video_device: str = self.declare_parameter('video_device', '/dev/video0').value
        self.frame_id: str = self.declare_parameter('frame_id', 'camera_link').value
        publish_rate: float = self.declare_parameter('publish_rate', 10.0).value
        self.reconnect_interval: float = self.declare_parameter(
            'reconnect_interval_sec', 1.0).value

        self.publisher = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()

        self.cap: Optional[cv2.VideoCapture] = None
        # 接続状態（None: 未判定）。状態が変わった時だけログを出す
        self.connected: Optional[bool] = None
        self.last_open_time = self.get_clock().now()

        self.get_logger().info(
            f'カメラサーバを起動しました (video_device={self.video_device}, '
            f'publish_rate={publish_rate}Hz)')
        self._open()
        self.timer = self.create_timer(1.0 / publish_rate, self._on_timer)

    def _open(self) -> None:
        if self.cap is not None:
            self.cap.release()
        self.last_open_time = self.get_clock().now()
        self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        self._set_connected(self.cap.isOpened())

    def _set_connected(self, connected: bool) -> None:
        if connected == self.connected:
            return
        self.connected = connected
        if connected:
            self.get_logger().info(f'カメラと接続しました ({self.video_device})')
        else:
            self.get_logger().error(
                f'カメラと接続できません ({self.video_device})。'
                f'接続とvideo_deviceパラメータを確認してください')

    def _on_timer(self) -> None:
        if not self.connected:
            elapsed = (self.get_clock().now() - self.last_open_time).nanoseconds * 1e-9
            if elapsed < self.reconnect_interval:
                return
            self._open()
            if not self.connected:
                return

        ok, frame = self.cap.read()
        self._set_connected(ok)
        if not ok:
            return

        # VLMのVRAM節約が必要なら、ここで事前にリサイズしてから配信してもよい
        # frame = cv2.resize(frame, (224, 224))
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def destroy_node(self) -> None:
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
