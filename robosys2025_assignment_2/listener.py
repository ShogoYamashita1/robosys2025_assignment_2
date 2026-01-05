import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class Listener():
    def __init__(self, nh):
        self.sub = nh.create_subscription(Int16MultiArray, "hand_result", self.cb, 10)
        self.nh = nh # ノードを属性に設定

    def cb(self, msg):
        data = msg.data
        self.nh.get_logger().info(f"Listen: {data}")# 設定したノードを使用

def main():
    rclpy.init()
    node = Node("listener")
    listener = Listener(node)
    while rclpy.ok():
        rclpy.spin_once(node)

if __name__ == "__main__":
    main()
