#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class VLMCLI(Node):
    def __init__(self):
        super().__init__('vlm_cli')
        self.pub = self.create_publisher(String, '/vlm/input', 10)
        self.get_logger().info("Type commands (e.g., 'Go forward') and press Enter.")

    def publish_line(self, line: str):
        msg = String()
        msg.data = line.strip()
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = VLMCLI()
    try:
        while rclpy.ok():
            line = sys.stdin.readline()
            if not line:
                break
            node.publish_line(line)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
