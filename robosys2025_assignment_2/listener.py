import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class Listener():
    def __init__(self, nh):
        self.sub = nh.create_subscription(Int16, "countup", self.cb, 10)
        self.nh = nh # ノードを属性に設定

    def cb(self, msg):
        self.nh.get_logger().info("Listen: %d" % msg.data)# 設定したノードを使用

def main():
    rclpy.init()
    node = Node("listener")
    listener = Listener(node)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
