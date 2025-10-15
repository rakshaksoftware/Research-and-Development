#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time, sys

class MoveNode(Node):
    """
    Publish Twist commands read from a timestamped file.
    The node will publish to a few common topic names so you can try which one your model responds to.
    
    Usage:
      ros2 run drone_sim move_node <drone_name> <input_file>
    Example:
      ros2 run drone_sim move_node drone_a ~/drone_ws/src/drone_sim/test_inputs/drone_a_move.txt
    NOTE: For the model to actually move, the model/SDF must include a plugin or controller that listens to a cmd_vel topic.
    """
    def __init__(self, drone_name, input_file):
        super().__init__(f'{drone_name}_move_node')
        self.drone_name = drone_name
        self.infile = input_file

        # common candidate topics (some models / bridges expect one of these)
        self.topics = [
            f'/{drone_name}/cmd_vel',
            f'/model/{drone_name}/cmd_vel',
            f'/cmd_vel/{drone_name}',
            f'/model/{drone_name}/cmd_vel/ign'  # extra candidate
        ]

        # create publishers for all candidate topics
        self._publishers = [self.create_publisher(Twist, t, 10) for t in self.topics]
        self.get_logger().info(f"[{self.drone_name}] Move publishers: {', '.join(self.topics)}")

        # parse move file: lines "timestamp action" (action: forward/backward/up/down or velocity numbers)
        self.commands = []
        try:
            with open(self.infile, 'r') as f:
                for raw in f:
                    line = raw.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split(maxsplit=1)
                    if len(parts) < 1:
                        continue
                    try:
                        t = float(parts[0])
                        txt = parts[1] if len(parts) > 1 else ''
                        self.commands.append((t, txt))
                    except ValueError:
                        self.get_logger().warning(f"Skipping invalid move line: {line}")
        except FileNotFoundError:
            self.get_logger().error(f"Input file not found: {self.infile}")
            return

        self.start_time = time.time()
        self.sent = [False] * len(self.commands)
        self.timer = self.create_timer(0.05, self._timer_cb)

    def _timer_cb(self):
        now = time.time() - self.start_time
        for i, (t, txt) in enumerate(self.commands):
            if not self.sent[i] and now >= t:
                twist = Twist()
                txtl = txt.lower()
                # interpret text simply
                if 'forward' in txtl:
                    twist.linear.x = 0.8
                if 'backward' in txtl:
                    twist.linear.x = -0.8
                if 'left' in txtl:
                    twist.linear.y = 0.6
                if 'right' in txtl:
                    twist.linear.y = -0.6
                if 'up' in txtl:
                    twist.linear.z = 0.6
                if 'down' in txtl:
                    twist.linear.z = -0.6

                # publish to all candidate topics
                for pub, topic in zip(self._publishers, self.topics):
                    pub.publish(twist)

                self.get_logger().info(f"[{self.drone_name}] Publish move @ {now:.2f}s -> '{txt}'")
                self.sent[i] = True

def main(argv=None):
    rclpy.init()
    if argv is None:
        argv = sys.argv

    if len(argv) < 3:
        print("Usage: ros2 run drone_sim move_node <drone_name> <input_file>")
        return

    node = MoveNode(argv[1], argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
