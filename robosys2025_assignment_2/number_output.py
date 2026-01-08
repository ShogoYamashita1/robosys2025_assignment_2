#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025-2026 Shogo Yamashita
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16

class Handnumber_node(Node):
    def __init__(self):
        super().__init__('number_node')
        self.sub = self.create_subscription(Int16MultiArray, "finger_close_state", self.cb, 10)
        self.pub = self.create_publisher(Int16, "number_result", 10)
        self.listened_data = None

    def cb(self, msg):
        self.listened_data = list(msg.data)
        self.get_logger().info(f"Listen: {msg.data}")

    def publish(self, janken_result):
        msg = Int16()
        msg.data = janken_result
        self.pub.publish(msg)


def judge(close_state):
        if close_state is None:
            return -1

        elif (close_state == [1, 1, 1, 1, 1]):
            return 0

        elif (close_state == [1, 0, 1, 1, 1]):
            return 1

        elif (close_state == [1, 0, 0, 1, 1]):
            return 2

        elif (close_state == [1, 0, 0, 0, 1]):
            return 3

        elif (close_state == [1, 0, 0, 0, 0]):
            return 4

        elif (close_state == [0, 0, 0, 0, 0]):
            return 5

        else:
            return -1


def main():
    rclpy.init()
    node = Handnumber_node()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.001)
        close_state = node.listened_data
        janken_result = judge(close_state)
        node.publish(janken_result)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
