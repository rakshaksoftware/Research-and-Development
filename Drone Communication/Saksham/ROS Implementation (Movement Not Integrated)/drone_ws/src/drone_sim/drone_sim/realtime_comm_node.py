#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import time

class RealtimeCommNode(Node):
    def __init__(self, drone_name, out_topic):
        super().__init__(f'{drone_name}_realtime_comm_node')
        self.drone_name = drone_name
        self.start_time = time.time()  # reference start time
        self.pub = self.create_publisher(String, out_topic, 10)
        self.get_logger().info(f"[{self.drone_name}] Ready for realtime input -> {out_topic}")

        # Thread for reading user input
        threading.Thread(target=self.read_input, daemon=True).start()

    def read_input(self):
        while True:
            try:
                text = input(f"[{self.drone_name}] Type message: ")
                timestamp = round(time.time() - self.start_time, 1)  # seconds since node started
                msg = String()
                msg.data = f"{timestamp} {text}"  # prepend timestamp
                self.pub.publish(msg)
                self.get_logger().info(f"Sent: '{msg.data}'")
            except EOFError:
                break

def main(argv=None):
    rclpy.init()
    if argv is None:
        argv = sys.argv
    if len(argv) < 3:
        print("Usage: ros2 run drone_sim realtime_comm_node <drone_name> <out_topic>")
        return
    node = RealtimeCommNode(argv[1], argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
