#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from abc import ABC, abstractmethod


# Abstract Power Sink
class PowerSink(ABC):
    def __init__(self, draw_rate, charge_rate, efficiency):
        self.draw_rate = draw_rate       # Energy drawn per timestep
        self.charge_rate = charge_rate  # Energy gained per timestep
        self.efficiency = efficiency    # Charging efficiency (percentage)
        self.battery_level = 30.0       # Initial battery percentage (low)
        self.connected = True           # Connection state

    @abstractmethod
    def update_state(self):
        """Update the internal state of the sink (e.g., battery level)."""
        pass

    @abstractmethod
    def get_energy_draw(self):
        """Return the energy draw."""
        pass

    @abstractmethod
    def get_battery_status(self):
        """Return the current battery level."""
        pass

    def disconnect(self):
        """Disconnect the sink when the battery is fully charged."""
        self.connected = False


# Robot Sink (e.g., INT-BALL)
class RobotSink(PowerSink):
    def __init__(self):
        super().__init__(draw_rate=5, charge_rate=4, efficiency=0.9)

    def update_state(self):
        if self.connected:
            charge = self.charge_rate * self.efficiency
            self.battery_level += charge
            if self.battery_level >= 100.0:
                self.battery_level = 100.0
                self.disconnect()

    def get_energy_draw(self):
        return float(self.draw_rate) if self.connected else 0.0

    def get_battery_status(self):
        return self.battery_level


# Docking Module Sink (e.g., Dragon)
class DockingModuleSink(PowerSink):
    def __init__(self):
        super().__init__(draw_rate=20, charge_rate=5, efficiency=0.85)

    def update_state(self):
        if self.connected:
            charge = self.charge_rate * self.efficiency
            self.battery_level += charge
            if self.battery_level >= 100.0:
                self.battery_level = 100.0
                self.disconnect()

    def get_energy_draw(self):
        return float(self.draw_rate) if self.connected else 0.0

    def get_battery_status(self):
        return self.battery_level


# Power Sink Node
class PowerSinkNode(Node):
    def __init__(self, sink_type):
        super().__init__('power_sink')
        self.sink = self._initialize_sink(sink_type)

        # Publishers
        self.energy_draw_publisher = self.create_publisher(Float64, '/eps/power_sink', 10)
        self.battery_status_publisher = self.create_publisher(Float64, '/power_sink/battery_level', 10)

        # Timer
        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f"PowerSinkNode initialized with {type(self.sink).__name__}.")

    def _initialize_sink(self, sink_type):
        if sink_type == 1:
            self.get_logger().info("Using Robot Sink (e.g., INT-BALL).")
            return RobotSink()
        elif sink_type == 2:
            self.get_logger().info("Using Docking Module Sink (e.g., Dragon).")
            return DockingModuleSink()
        else:
            self.get_logger().warn("Invalid sink type. Defaulting to Robot Sink.")
            return RobotSink()

    def publish_status(self):
        # Update sink state (e.g., charging battery)
        self.sink.update_state()

        # Check if disconnected
        if not self.sink.connected:
            self.get_logger().info("Battery fully charged. Power Sink disconnected.")
            return  # Stop publishing when disconnected

        # Publish energy draw
        draw_msg = Float64()
        draw_msg.data = self.sink.get_energy_draw()
        self.energy_draw_publisher.publish(draw_msg)

        # Publish battery level
        battery_msg = Float64()
        battery_msg.data = self.sink.get_battery_status()
        self.battery_status_publisher.publish(battery_msg)

        # Log the status
        self.get_logger().info(f"Energy Draw: {draw_msg.data:.2f} W, Battery Level: {battery_msg.data:.2f}%")


def main():
    rclpy.init()

    # Choose sink type numerically
    print("Select the type of power sink:")
    print("1. Robot (e.g., INT-BALL)")
    print("2. Docking Module (e.g., Dragon)")
    sink_type = int(input("Enter your choice (1/2): ").strip())

    node = PowerSinkNode(sink_type)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
