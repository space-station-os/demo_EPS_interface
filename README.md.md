# SSOS EPS Demo Documentation

## Table of Contents
1. [Overview](#overview)
2. [Use Case Motivation](#use-case-motivation)
3. [System Architecture Design](#system-architecture-design)
4. [EPS Core Subsystem](#eps-core-subsystem)
5. [Simulation Script Descriptions](#simulation-script-descriptions)
6. [Data and Input/Output Values](#data-and-inputoutput-values)
7. [Assumptions](#assumptions)
8. [Initial Parameter Source/Reference](#initial-parameter-sourcereference)
9. [Setup Instructions](#setup-instructions)
10. [Execution and Outputs](#execution-and-outputs)
11. [Validation and Testing](#validation-and-testing)
12. [Future Work and Extensions](#future-work-and-extensions)
13. [Appendix](#appendix)
14. [References](#references)

---

## Overview
The SSOS EPS Demo demonstrates the functionality of the Electrical Power Subsystem (EPS) within the Space Station Operating System (SSOS). This demo includes components such as a power generator, power sink, visualizer, critical load simulation, and the EPS core. It showcases their integration in a ROS-based framework, highlighting modularity, extensibility, and real-time data sharing capabilities.

---

## Use Case Motivation
The demo addresses critical challenges in power management for space station systems:
- Ensuring stable power generation and distribution.
- Managing dynamic load demands and prioritizing critical loads.
- Visualizing power flow and system status in real time.
- Demonstrating fault tolerance and recovery mechanisms.

---

## System Architecture Design

### Overview Diagram
![System Architecture](system_architecture_diagram.png)

### ROS Nodes
1. **Power Generator Node (power_generator.py)**:
   - Simulates the power generation process.
   - Publishes generated power levels to `/power_output`.

2. **Power Sink Node (power_sink.py)**:
   - Simulates power consumption by various subsystems.
   - Subscribes to `/power_output` and adjusts consumption based on input.

3. **Visualizer Node (visualizer.py)**:
   - Displays power flow and system status.
   - Subscribes to `/power_output` and `/critical_load_status` for visualization.

4. **Critical Load Simulation Node (critical_load_sim.py)**:
   - Simulates the behavior of critical loads.
   - Publishes load demands to `/critical_load_demand` and monitors status via `/critical_load_status`.

5. **EPS Core Subsystem (eps_core.cpp)**:
   - Coordinates power generation, consumption, and load management.
   - Provides real-time state updates via `/eps/state`.
   - Manages interactions with other nodes using abstraction and encapsulation principles.

### Interfaces
- **Topics**:
  - `/power_output`: Generated power levels.
  - `/critical_load_demand`: Critical load requirements.
  - `/critical_load_status`: Status of critical load operations.
  - `/eps/state`: Current EPS status including energy stored and available power.
- **Services**:
  - `/reset_fault`: Resets detected faults in the system.

---

## EPS Core Subsystem

### Role of EPS Core
The EPS core acts as the central module, coordinating power generation, consumption, and critical load management:
- **Abstraction**: Provides generic interfaces for power management and load handling.
- **Inheritance**: Enables specialized nodes to extend core functionalities.
- **Encapsulation**: Hides internal logic, exposing only APIs necessary for node integration.

### Object-Oriented Design
- **EPSSubsystem Class**: Implements methods for handling power input, sinks, and critical loads.
- **Dynamic State Updates**: Uses timers to publish real-time state changes to `/eps/state`.
- **Noise and Variability Simulation**: Introduces random noise for realistic energy consumption and baseline load variability.

### Key Features
- Real-time energy management with dynamic critical load handling.
- Power distribution optimization based on available energy and priorities.
- Fault detection and recovery with configurable thresholds.

---

## Simulation Script Descriptions
1. **Power Generator Script (power_generator.py)**
   - Simulates power generation capabilities.
   - Publishes to `/power_output` topic.

2. **Power Sink Script (power_sink.py)**
   - Models subsystem power consumption.
   - Subscribes to `/power_output` to adjust load dynamically.

3. **Visualizer Script (visualizer.py)**
   - Displays system-wide power flow.
   - Processes `/power_output` and `/critical_load_status` for visualization.

4. **Critical Load Simulation Script (critical_load_sim.py)**
   - Emulates critical load requirements and operations.
   - Publishes to `/critical_load_demand` and `/critical_load_status`.

5. **EPS Core Implementation (eps_core.cpp)**
   - Manages overall power system state, including energy storage, critical load prioritization, and dynamic updates.
   - Handles interactions with the other nodes via defined topics and services.

---

## Data and Input/Output Values
- **Topics**:
  - `/power_output`: Reports current power generation levels.
  - `/critical_load_demand`: Specifies required power for critical loads.
  - `/critical_load_status`: Provides the operational status of critical loads.
  - `/eps/state`: Reports current EPS state including energy stored and available power.
- **Message Types**:
  - `std_msgs/Float64` for power values.
  - `std_msgs/String` for EPS state updates.
  - `std_msgs/Bool` for critical load statuses.

---

## Assumptions
- Power demand profiles are predefined.
- Nodes interact exclusively via ROS topics and services.
- Visualization updates occur at a fixed tick rate.
- Energy storage limits and decay rates are constant during simulations.

---

## Initial Parameter Source/Reference
- Power generation limits and efficiency metrics are derived from ESA technical documents.
- Critical load thresholds and priorities are based on NASA mission data.
- EPS core functionality is inspired by industry-standard power management systems.

---

## Setup Instructions
1. Install dependencies:
   ```bash
   sudo apt-get install ros-<distro>-desired-packages
   ```
2. Clone the repository:
   ```bash
   git clone <repository-url>
   ```
3. Build the workspace:
   ```bash
   cd <workspace>
   catkin_make
   ```
4. Launch the demo:
   ```bash
   roslaunch eps_demo eps_demo.launch
   ```

---

## Execution and Outputs
### Steps to Execute
1. Start ROS master:
   ```bash
   roscore
   ```
2. Launch the demo:
   ```bash
   roslaunch eps_demo eps_demo.launch
   ```

### Expected Outputs
- Logs showing power generation, critical load handling, and EPS state updates.
- Real-time visualizations of power flow and load demands.

---

## Validation and Testing
- **Validation Criteria**:
  - Verify power levels adjust dynamically to load changes.
  - Ensure critical load requirements are met under all conditions.
  - Simulate faults and observe system recovery behavior.
- **Example Test Case**:
  - Simulate a fault in power generation and observe EPS core handling.

---

## Future Work and Extensions
- Integration with thermal control subsystems.
- Development of predictive models for power consumption trends.
- Hardware-in-the-loop testing for real-time applications.
- Expand EPS core to handle renewable energy sources in simulations.

---

## Appendix
### Supplemental Materials
- Power generator flow diagram.
- EPS state machine description.
- Critical load operation timeline.

---

## References
1. NASA Systems Engineering Handbook.
2. ROS Documentation: https://www.ros.org
3. ESA Power Management Guidelines.
