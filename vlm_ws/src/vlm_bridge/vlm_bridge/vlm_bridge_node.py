#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VLMBridge(Node):
    def __init__(self):
        super().__init__('vlm_bridge')
        self.sub = self.create_subscription(String, '/vlm/input', self.cb_command, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('VLM bridge ready.')

    def cb_command(self, msg: String):
        text = msg.data.strip().lower()
        self.get_logger().info(f"Command received: '{text}'")
        parsed = self.parse_with_gemini(text) or self.rule_parse(text)
        if parsed:
            self.execute(parsed)
        else:
            self.get_logger().warn("Could not parse command.")

    def parse_with_gemini(self, text: str):
        # If you set USE_GEMINI=1 and implement the client, this will try Gemini.
        if not os.environ.get('USE_GEMINI'):
            return None
        # TODO: Add actual Gemini Robotics 1.5 API call here and return structured dict.
        return None

    def rule_parse(self, text: str):
        if 'forward' in text or text.startswith('go '):
            return {'action':'move','direction':'forward','distance':1.0,'speed':0.2}
        if 'back' in text or 'reverse' in text:
            return {'action':'move','direction':'back','distance':1.0,'speed':0.2}
        if 'left' in text:
            return {'action':'turn','direction':'left','angle':0.6,'speed':0.5}
        if 'right' in text:
            return {'action':'turn','direction':'right','angle':0.6,'speed':0.5}
        if 'stop' in text:
            return {'action':'stop'}
        return None

    def execute(self, parsed):
        if parsed['action'] == 'stop':
            self.cmd_pub.publish(Twist())
            return
        if parsed['action'] == 'move':
            speed = parsed.get('speed', 0.2)
            dist = parsed.get('distance', 1.0)
            duration = max(0.1, dist / max(1e-3, speed))
            twist = Twist()
            twist.linear.x = speed if parsed['direction']=='forward' else -speed
            self._publish_for_duration(twist, duration)
        if parsed['action'] == 'turn':
            angular = parsed.get('speed', 0.5)
            angle = parsed.get('angle', 0.6)
            dur = max(0.1, angle / angular)
            twist = Twist()
            twist.angular.z = angular if parsed['direction']=='left' else -angular
            self._publish_for_duration(twist, dur)

    def _publish_for_duration(self, twist, duration):
        end = time.time() + duration
        rate = self.create_rate(10)
        while rclpy.ok() and time.time() < end:
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VLMBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
