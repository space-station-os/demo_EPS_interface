#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from abc import ABC, abstractmethod
import random

# Abstract Factory and Concrete Generators
class PowerGenerator(ABC):
    """Abstract Power Generator Class."""
    def __init__(self, base_power, noise_range):
        self.base_power = base_power
        self.noise_range = noise_range

    @abstractmethod
    def generate_power(self):
        """Abstract method to generate power."""
        pass


class SolarPanelGenerator(PowerGenerator):
    """Concrete implementation of a solar panel power generator."""
    def generate_power(self):
        noise = random.uniform(-self.noise_range, self.noise_range)
        return max(0, self.base_power + noise)  # Ensure no negative power


class FuelCellGenerator(PowerGenerator):
    """Concrete implementation of a fuel cell power generator."""
    def generate_power(self):
        noise = random.uniform(-self.noise_range, self.noise_range)
        return max(0, self.base_power + noise)  # Ensure no negative power


class PowerSourceNode(Node):
    """ROS 2 Power Source Node."""
    def __init__(self, generator: PowerGenerator):
        super().__init__('power_source')
        self.generator = generator
        self.publisher = self.create_publisher(Float64, '/eps/power_source', 10)
        self.timer = self.create_timer(1.0, self.publish_power)  # Publish every second
        self.get_logger().info("PowerSourceNode initialized with generator.")

    def publish_power(self):
        power = self.generator.generate_power()
        msg = Float64()
        msg.data = power
        self.publisher.publish(msg)
        self.get_logger().info(f"Generated Power: {power:.2f} W")


def main():
    rclpy.init()

    # Select a power generator
    print("Select the type of power generator:")
    print("1. Solar Panel")
    print("2. Fuel Cell")
    generator_type = input("Enter your choice (1/2): ").strip()

    if generator_type == "1":
        generator = SolarPanelGenerator(base_power=15, noise_range=1)  # Smaller base power and noise range
        print("Using Solar Panel Generator (Base Power: 15 W, Noise Range: ±1 W)")
    elif generator_type == "2":
        generator = FuelCellGenerator(base_power=50, noise_range=0.5)  # Smaller base power and noise range
        print("Using Fuel Cell Generator (Base Power: 50 W, Noise Range: ±0.5 W)")
    else:
        print("Invalid choice. Defaulting to Solar Panel Generator.")
        generator = SolarPanelGenerator(base_power=15, noise_range=1)

    node = PowerSourceNode(generator)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
