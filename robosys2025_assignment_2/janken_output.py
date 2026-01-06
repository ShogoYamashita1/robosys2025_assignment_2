#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025-2026 Shogo Yamashita
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String

class Janken_node(Node):
    def __init__(self):
        super().__init__('janken_node')
        self.sub = self.create_subscription(Int16MultiArray, "finger_close_state", self.cb, 10)
        self.pub = self.create_publisher(String, "hand_result", 10)
        self.listened_data = None

    def cb(self, msg):
        self.listened_data = list(msg.data)
        self.get_logger().info(f"Listen: {msg.data}")

    def publish(self, janken_result):
        msg = String()
        msg.data = janken_result
        self.pub.publish(msg)


def judge(close_state):
        if close_state is None:
            return "None"

        if (all(i == 0 for i in close_state)):
            return "Paper"# パー

        elif (all(i == 1 for i in close_state)):
            return "Rock"# グー

        elif (close_state == [1, 0, 0, 1, 1]):
            return "Scissors"#チョキ

        else:
            return "None"


def main():
    rclpy.init()
    node = Janken_node()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.001)
        close_state = node.listened_data
        janken_result = judge(close_state)
        node.publish(janken_result)



if __name__ == "__main__":
    main()
