#!/usr/bin/python3
# SPDX-FileCopyrightText: 2026 Junya Wada
# SPDX-License-Identifier: BSD-3-Clause

import os

# 再接続のたびにOpenCVが出すWARNを抑える（cv2のimportより前に設定する必要がある）
os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # 画像を配信するパブリッシャーを作成
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        
        # 1秒間に何回画像を配信するか（例：10Hz = 0.1秒ごと）
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # OpenCVとROS 2の画像形式を変換するブリッジ
        self.bridge = CvBridge()
        
        self.get_logger().info('カメラサーバを起動しました')

        # カメラの起動（適宜番号変えてくださいな）
        self.camera_index = 0
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        # 接続状態（None: 未判定, True: 接続中, False: 切断中）。状態が変わった時だけログを出す
        self.connected = None

        # 切断中に再接続を試みる間隔[秒]
        self.reconnect_interval = 1.0
        self.last_reconnect_time = self.get_clock().now()

        self.update_connection_state(self.cap.isOpened())

    def update_connection_state(self, connected):
        if connected == self.connected:
            return
        self.connected = connected
        if connected:
            self.get_logger().info('カメラと接続しました')
        else:
            self.get_logger().error('カメラと接続できません！接続を確認してください。')

    def try_reconnect(self):
        now = self.get_clock().now()
        if (now - self.last_reconnect_time).nanoseconds < self.reconnect_interval * 1e9:
            return
        self.last_reconnect_time = now
        self.cap.release()
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

    def timer_callback(self):
        if not self.connected:
            self.try_reconnect()
            if not self.cap.isOpened():
                return

        ret, frame = self.cap.read()
        self.update_connection_state(ret)

        if ret:
            # 💡 VLAのVRAM(4GB)節約のために、ここで事前に小さくリサイズしてパブリッシュするのもアリです！
            # frame = cv2.resize(frame, (224, 224)) 
            
            # OpenCVの画像(ndarray)をROS 2のImageメッセージに変換
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # 配信！
            self.publisher_.publish(msg)

    def destroy_node(self):
        # ノード終了時にカメラをしっかり解放する
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
