#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from abc import ABC, abstractmethod
import random


# Abstract Sink Class
class PowerSink(ABC):
    """Abstract Power Sink Class."""
    def __init__(self, draw_rate, charge_rate, efficiency):
        self.draw_rate = draw_rate       # Energy drawn per timestep
        self.charge_rate = charge_rate  # Energy gained per timestep
        self.efficiency = efficiency    # Charging efficiency (percentage)

    @abstractmethod
    def update_state(self):
        """Update the internal state of the sink."""
        pass

    @abstractmethod
    def get_energy_draw(self):
        """Return the energy draw for the sink."""
        pass

    @abstractmethod
    def get_battery_status(self):
        """Return the current battery status."""
        pass


# Concrete Implementation: Robot Sink (e.g., INT-BALL)
class RobotSink(PowerSink):
    def __init__(self):
        super().__init__(draw_rate=5, charge_rate=2, efficiency=0.9)  # Example rates
        self.battery_level = 30  # Initial battery percentage (low)

    def update_state(self):
        # Simulate charging (with inefficiency)
        charge = self.charge_rate * self.efficiency
        self.battery_level += charge
        if self.battery_level > 100:
            self.battery_level = 100  # Cap battery at 100%

    def get_energy_draw(self):
        return self.draw_rate

    def get_battery_status(self):
        return self.battery_level


# Concrete Implementation: Docking Module Sink (e.g., Dragon)
class DockingModuleSink(PowerSink):
    def __init__(self):
        super().__init__(draw_rate=20, charge_rate=1, efficiency=0.85)  # Larger rates for docking module
        self.battery_level = 50  # Initial battery percentage (medium)

    def update_state(self):
        # Simulate charging (with inefficiency)
        charge = self.charge_rate * self.efficiency
        self.battery_level += charge
        if self.battery_level > 100:
            self.battery_level = 100  # Cap battery at 100%

    def get_energy_draw(self):
        return self.draw_rate

    def get_battery_status(self):
        return self.battery_level


class PowerSinkNode(Node):
    """ROS 2 Node for Power Sinks."""
    def __init__(self, sink: PowerSink):
        super().__init__('power_sink')
        self.sink = sink
        self.publisher = self.create_publisher(Float64, '/eps/power_sink', 10)
        self.timer = self.create_timer(1.0, self.publish_energy_draw)  # Publish every second
        self.get_logger().info(f"PowerSinkNode initialized with {type(self.sink).__name__}.")

    def publish_energy_draw(self):
        # Update sink state (e.g., charging battery)
        self.sink.update_state()

        # Ensure the energy draw is explicitly a float
        draw = float(self.sink.get_energy_draw())
        msg = Float64()
        msg.data = draw
        self.publisher.publish(msg)

        # Log sink status
        battery_status = self.sink.get_battery_status()
        self.get_logger().info(f"Energy Draw: {draw:.2f} W, Battery Status: {battery_status:.2f}%")


def main():
    rclpy.init()

    # Select the power sink type
    print("Select the type of power sink:")
    print("1. Robot (e.g., INT-BALL)")
    print("2. Docking Module (e.g., Dragon)")
    sink_type = input("Enter your choice (1/2): ").strip()

    if sink_type == "1":
        sink = RobotSink()
        print("Using Robot Sink (e.g., INT-BALL).")
    elif sink_type == "2":
        sink = DockingModuleSink()
        print("Using Docking Module Sink (e.g., Dragon).")
    else:
        print("Invalid choice. Defaulting to Robot Sink.")
        sink = RobotSink()

    node = PowerSinkNode(sink)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
