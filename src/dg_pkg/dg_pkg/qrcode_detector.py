"""
QR码检测节点 — 识别二维码数字内容，按奇偶判定 ClockWise / AntiClockWise 方向。

订阅话题:
  /image  (sensor_msgs/CompressedImage) — USB 单目摄像头压缩图像

发布话题:
  /sign_foxglove          (std_msgs/Int32)  — 上位机信号：3=ClockWise, 4=AntiClockWise, 0=未知
  /qr_code/data           (std_msgs/String) — 二维码原始内容
  /qr_code/direction      (std_msgs/String) — ClockWise / AntiClockWise / unknown

参数:
  camera_topic      — 压缩图像话题名 (默认 "/image")
  debounce_frames   — 连续 N 帧结果一致才确认方向 (默认 5)

方向判定规则:
  - 二维码内容为 1~9999 的整数
  - 奇数: ClockWise，发布 3
  - 偶数: AntiClockWise，发布 4
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from std_msgs.msg import String

from pyzbar.pyzbar import decode
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)


class QRCodeDetector(Node):

    def __init__(self):
        super().__init__('qrcode_detector')

        # 参数
        self.declare_parameter('camera_topic', '/image')
        self.declare_parameter('debounce_frames', 5)

        camera_topic = self.get_parameter(
            'camera_topic'
        ).value

        self.debounce_frames = self.get_parameter(
            'debounce_frames'
        ).value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            camera_topic,
            self.image_callback,
            sensor_qos
        )

        # 发布二维码识别结果
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(
            Int32,
            '/sign_foxglove',
            pub_qos
        )
        self.raw_data_publisher = self.create_publisher(
            String,
            '/qr_code/data',
            pub_qos
        )
        self.direction_publisher = self.create_publisher(
            String,
            '/qr_code/direction',
            pub_qos
        )

        self.last_text = None
        self.same_count = 0

        self.get_logger().info(
            f'QR码检测节点启动 — 订阅: {camera_topic}, '
            f'信号发布: /sign_foxglove, '
            f'原始数据发布: /qr_code/data, '
            f'去抖帧数: {self.debounce_frames}'
        )

    def publish_sign(self, value):
        msg = Int32()
        msg.data = value
        self.publisher.publish(msg)

    def publish_qr_result(self, raw_text, direction):
        raw_msg = String()
        raw_msg.data = raw_text
        self.raw_data_publisher.publish(raw_msg)

        direction_msg = String()
        direction_msg.data = direction
        self.direction_publisher.publish(direction_msg)

    def image_callback(self, msg):

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)

            frame = cv2.imdecode(
                np_arr,
                cv2.IMREAD_COLOR
            )

            if frame is None:
                return

        except Exception as e:
            self.get_logger().error(
                f'图像解码失败: {e}'
            )
            return

        qrcodes = decode(frame)

        if len(qrcodes) == 0:
            self.last_text = None
            self.same_count = 0
            return

        qrcode = max(
            qrcodes,
            key=lambda code: code.rect.width * code.rect.height
        )

        text = qrcode.data.decode(
            'utf-8',
            errors='ignore'
        ).strip()

        if text == self.last_text:
            self.same_count += 1
        else:
            self.last_text = text
            self.same_count = 1

        if self.same_count < self.debounce_frames:
            return

        direction = None
        signal_value = None

        if text.isdigit():
            number = int(text)

            if 1 <= number <= 9999:
                if number % 2 == 1:
                    direction = 'ClockWise'
                    signal_value = 3
                else:
                    direction = 'AntiClockWise'
                    signal_value = 4

        if signal_value is not None:

            self.publish_qr_result(text, direction)
            self.publish_sign(signal_value)

            self.get_logger().info(
                "\n========================================\n"
                f"🚩 QR识别结果: 【{direction}】\n"
                f'原始内容: "{text}" | 信号值: {signal_value}\n'
                "========================================"
            )

            self.same_count = 0

        else:

            self.publish_qr_result(text, 'unknown')
            self.publish_sign(0)

            self.get_logger().info(
                f'检测到二维码: "{text}" (未定义，需为1~9999整数)'
            )

            self.same_count = 0


def main(args=None):

    rclpy.init(args=args)

    node = QRCodeDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
