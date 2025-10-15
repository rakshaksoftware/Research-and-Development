#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time, sys

class CommNode(Node):
    """
    Publish timestamped strings read from a file at the scheduled times.
    Usage:
      ros2 run drone_sim comm_node <drone_name> <input_file> <out_topic>
    Example:
      ros2 run drone_sim comm_node drone_a ~/drone_ws/src/drone_sim/test_inputs/drone_a_input.txt /drone_A_to_B
    """
    def __init__(self, drone_name, input_file, out_topic):
        super().__init__(f'{drone_name}_comm_node')
        self.drone_name = drone_name
        self.infile = input_file
        self.out_topic = out_topic
        self.pub = self.create_publisher(String, out_topic, 10)
        self.get_logger().info(f"[{self.drone_name}] Publisher -> {self.out_topic} reading {self.infile}")

        # load commands list [(t_seconds, text), ...]
        self.commands = []
        with open(self.infile, 'r') as f:
            for lineno, raw in enumerate(f, start=1):
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(maxsplit=1)
                if len(parts) == 0:
                    continue
                try:
                    t = float(parts[0])
                    text = parts[1] if len(parts) > 1 else ''
                    self.commands.append((t, text))
                except Exception as e:
                    self.get_logger().warning(f"Skipping invalid line {lineno}: '{line}' ({e})")

        self.start_time = time.time()
        self.sent = [False]*len(self.commands)
        # timer checks 20 times per second
        self.timer = self.create_timer(0.05, self._timer_cb)

    def _timer_cb(self):
        now = time.time() - self.start_time
        for i, (t, text) in enumerate(self.commands):
            if not self.sent[i] and now >= t:
                msg = String()
                # include the original scheduled timestamp and actual publish system time
                msg.data = f"{t} {text}"
                self.pub.publish(msg)
                self.get_logger().info(f"Sent @ {now:.2f}s -> '{msg.data}'")
                self.sent[i] = True

def main(argv=None):
    rclpy.init()
    if argv is None:
        argv = sys.argv
    if len(argv) < 4:
        print("Usage: ros2 run drone_sim comm_node <drone_name> <input_file> <out_topic>")
        return
    node = CommNode(argv[1], argv[2], argv[3])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
