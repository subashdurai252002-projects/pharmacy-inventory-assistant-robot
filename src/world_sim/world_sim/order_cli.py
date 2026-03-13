import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OrderCLI(Node):
    def __init__(self):
        super().__init__("order_cli")
        self.pub = self.create_publisher(String, "/orders", 10)
        self.get_logger().info("Order CLI ready. Example: paracetamol 1")

    def run(self):
        while rclpy.ok():
            try:
                s = input("Enter order (name qty): ").strip()
            except EOFError:
                break
            if not s:
                continue
            msg = String()
            msg.data = s
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = OrderCLI()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
