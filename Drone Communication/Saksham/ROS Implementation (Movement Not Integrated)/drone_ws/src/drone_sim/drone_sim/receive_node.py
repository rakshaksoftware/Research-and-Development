#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os, sys

class ReceiveNode(Node):
    def __init__(self, drone_name, topic_name):
        super().__init__(f'{drone_name}_receive_node')
        self.log_file = os.path.expanduser(f"~/ros_{drone_name}_log.txt")
        
        # Clear log file at start
        with open(self.log_file, "w") as f:
            f.write("")

        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.callback,
            10
        )
        self.get_logger().info(f"[{drone_name}] Subscribed <- {topic_name}, logging to {self.log_file}")

    def callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Recv: {text}")
        with open(self.log_file, "a") as f:
            f.write(text + "\n")

def main(argv=None):
    rclpy.init()
    if argv is None:
        argv = sys.argv
    if len(argv) < 3:
        print("Usage: ros2 run drone_sim receive_node <drone_name> <topic>")
        return
    node = ReceiveNode(argv[1], argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
