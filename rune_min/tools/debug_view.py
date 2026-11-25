#!/usr/bin/env python3
# 能量机关识别调试图播放器：订阅 /rune/debug/image/compressed，OpenCV 窗口显示。
#
#   source /opt/ros/humble/setup.bash
#   python3 debug_view.py
#
# 依赖：sudo apt install python3-opencv

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class DebugView(Node):
    def __init__(self):
        super().__init__('rune_debug_view')
        # best-effort：对 best-effort / reliable 的发布者都兼容
        self.create_subscription(
            CompressedImage, '/rune/debug/image/compressed',
            self.on_image, qos_profile_sensor_data)
        cv2.namedWindow('rune debug', cv2.WINDOW_AUTOSIZE)

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        cv2.imshow('rune debug', img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = DebugView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
