#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading


class EPSVisualizer(Node):
    def __init__(self):
        super().__init__('eps_visualizer')

        # Subscriptions
        self.state_subscription = self.create_subscription(
            String,
            '/eps/state',
            self.state_callback,
            10)

        self.battery_subscription = self.create_subscription(
            Float64,
            '/power_sink/battery_level',
            self.battery_callback,
            10)

        # Data for visualization
        self.energy_stored = []
        self.available_power = []
        self.critical_load = []
        self.non_critical_load = []
        self.battery_level = []
        self.timestamps = []

    def state_callback(self, msg):
        print(f"Received: {msg.data}")
        try:
            state_parts = msg.data.replace("EPS State: ", "").split(', ')
            energy_stored = float(state_parts[0].split('=')[1].strip().split()[0])
            available_power = float(state_parts[1].split('=')[1].strip().split()[0])
            critical_load = float(state_parts[2].split('=')[1].strip().split()[0])
            non_critical_load = float(state_parts[3].split('=')[1].strip().split()[0])

            # Append parsed values
            self.energy_stored.append(energy_stored)
            self.available_power.append(available_power)
            self.critical_load.append(critical_load)
            self.non_critical_load.append(non_critical_load)
            self.timestamps.append(len(self.energy_stored))
        except Exception as e:
            self.get_logger().error(f"Error parsing state message: {e}")

    def battery_callback(self, msg):
        try:
            self.battery_level.append(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error parsing battery message: {e}")


def animate(i, visualizer, ax1, ax2, ax3, ax4):
    if not visualizer.timestamps:
        return

    # Clear all axes
    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()

    # Plot energy stored
    ax1.plot(visualizer.timestamps, visualizer.energy_stored, label="Energy Stored (W)", color="blue")
    ax1.set_title("Energy Stored Over Time")
    ax1.set_xlabel("Timesteps")
    ax1.set_ylabel("Energy (W)")
    ax1.legend()

    # Plot available power
    ax2.plot(visualizer.timestamps, visualizer.available_power, label="Available Power (W)", color="green")
    ax2.set_title("Available Power Over Time")
    ax2.set_xlabel("Timesteps")
    ax2.set_ylabel("Power (W)")
    ax2.legend()

    # Plot critical load
    ax3.plot(visualizer.timestamps, visualizer.critical_load, label="Critical Load (W)", color="red")
    ax3.set_title("Critical Load Over Time")
    ax3.set_xlabel("Timesteps")
    ax3.set_ylabel("Load (W)")
    ax3.legend()

    # Plot battery level
    ax4.plot(visualizer.timestamps[:len(visualizer.battery_level)], visualizer.battery_level, label="Battery Level (%)", color="purple")
    ax4.set_title("Battery Level Over Time")
    ax4.set_xlabel("Timesteps")
    ax4.set_ylabel("Battery (%)")
    ax4.legend()


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    visualizer = EPSVisualizer()

    # Start rclpy.spin() in a separate thread
    spin_thread = threading.Thread(target=ros_spin_thread, args=(visualizer,))
    spin_thread.start()

    # Set up Matplotlib for live plotting
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
    ani = FuncAnimation(fig, animate, fargs=(visualizer, ax1, ax2, ax3, ax4), interval=1000)
    plt.tight_layout()
    plt.show()

    # Cleanup
    visualizer.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
