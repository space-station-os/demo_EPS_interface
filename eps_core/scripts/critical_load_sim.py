#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random


class CriticalLoadSimulator(Node):
    def __init__(self):
        super().__init__('critical_load_simulator')
        self.publisher = self.create_publisher(Float64, '/eps/critical_load', 10)
        self.timer = self.create_timer(1.0, self.publish_critical_load)
        self.steps = 0  # Counter to track the number of steps
        self.max_steps = 30  # Maximum number of steps to simulate
        self.current_load = 0.0  # Current critical load value
        self.decaying = False  # Whether the load is in the decaying phase

    def publish_critical_load(self):
        if self.steps < self.max_steps and not self.decaying:
            # 80% probability of triggering a critical load
            if random.random() < 0.8:
                self.current_load = random.uniform(10, 20)  # Example range (10W to 20W)
                self.get_logger().info(f"Critical Load Triggered: {self.current_load:.2f} W")
            else:
                self.current_load = 0.0  # No critical load this step
                self.get_logger().info("No Critical Load This Step.")
            self.steps += 1
        elif self.steps >= self.max_steps and not self.decaying:
            # Start decaying phase after max_steps
            self.get_logger().info("Critical Load Simulation Completed. Beginning Decay to Disconnect.")
            self.decaying = True

        if self.decaying:
            # Gradually decay the critical load to zero
            decay_rate = 2.0  # Rate of decay per step
            self.current_load = max(0.0, self.current_load - decay_rate)
            if self.current_load == 0.0:
                self.get_logger().info("Critical Load Decayed to Zero. Disconnecting.")
                self.timer.cancel()  # Stop the timer once fully decayed

        # Publish the current critical load
        msg = Float64()
        msg.data = self.current_load
        self.publisher.publish(msg)
        self.get_logger().info(f"Current Critical Load Published: {self.current_load:.2f} W")


def main():
    rclpy.init()
    node = CriticalLoadSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
