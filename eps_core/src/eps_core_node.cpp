// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, the License is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>
#include <iostream>
#include <random>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class EPSSubsystem : public rclcpp::Node
{
public:
  EPSSubsystem()
  : Node("eps_subsystem"), max_capacity_(1000.0), energy_stored_(500.0), reserve_capacity_(300.0),
    critical_load_(0.0), non_critical_load_(0.0), baseline_consumption_(5.0)
  {
    // Publishers
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("/eps/state", 10);

    // Subscriptions
    power_source_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/eps/power_source", 10, std::bind(&EPSSubsystem::handle_power_input, this, std::placeholders::_1));

    power_sink_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/eps/power_sink", 10, std::bind(&EPSSubsystem::handle_power_sink, this, std::placeholders::_1));

    critical_load_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/eps/critical_load", 10, std::bind(&EPSSubsystem::handle_critical_load, this, std::placeholders::_1));

    // Timer for state updates
    state_timer_ = this->create_wall_timer(
      500ms, std::bind(&EPSSubsystem::publish_state, this));

    // Random noise generators
    std::random_device rd;
    gen_ = std::mt19937(rd());
    noise_dist_ = std::uniform_real_distribution<>(-0.2, 0.2);       // Noise range for energy updates
    consumption_dist_ = std::uniform_real_distribution<>(-1.0, 1.0); // Noise range for baseline consumption
  }

private:
  void handle_power_input(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double input_power = msg->data;
    double remaining_capacity = max_capacity_ - energy_stored_;

    if (input_power > remaining_capacity)
    {
      energy_stored_ = max_capacity_;  // Fill to max capacity
      RCLCPP_WARN(this->get_logger(),
                  "Power input of %.2f W exceeds remaining capacity of %.2f W. Only %.2f W accepted.",
                  input_power, remaining_capacity, remaining_capacity);
    }
    else
    {
      energy_stored_ += input_power;
      RCLCPP_INFO(this->get_logger(),
                  "Power input of %.2f W accepted. Energy stored: %.2f W", input_power, energy_stored_);
    }
  }

  void handle_power_sink(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double requested_power = msg->data;
    if (requested_power > energy_stored_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Power sink request of %.2f W exceeds available energy. Request partially fulfilled.",
                  requested_power);
      energy_stored_ = 0.0;
    }
    else
    {
      energy_stored_ -= requested_power;
      RCLCPP_INFO(this->get_logger(),
                  "Power sink request of %.2f W fulfilled. Energy stored: %.2f W", requested_power, energy_stored_);
    }
  }

  void handle_critical_load(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double critical_request = msg->data;

    // Dynamically calculate available power
    double available_power = energy_stored_ - critical_load_ - non_critical_load_;

    if (critical_request > available_power) {
        RCLCPP_WARN(this->get_logger(), "Critical load request of %.2f W exceeds available power. Partial fulfillment.", critical_request);
        critical_request = available_power; // Fulfill as much as possible
    }

    critical_load_ += critical_request;
    energy_stored_ -= critical_request;

    if (energy_stored_ < 0.0) {
        energy_stored_ = 0.0; // Prevent negative energy
    }

    RCLCPP_INFO(this->get_logger(), "Critical load of %.2f W handled. Remaining energy: %.2f W", critical_request, energy_stored_);
  }

  void decay_critical_load()
  {
    // Apply decay to critical load
    if (critical_load_ > 0.0) {
        double decay_rate = 2.0; // Amount to decay per timestep
        critical_load_ -= decay_rate;
        if (critical_load_ < 0.0) {
            critical_load_ = 0.0; // Prevent negative critical load
            RCLCPP_INFO(this->get_logger(), "Critical load fully decayed and disconnected.");
        }
    }
  }

  void publish_state()
  {
    // Simulate baseline energy consumption with variability
    double noise = consumption_dist_(gen_);
    double consumption = baseline_consumption_ + noise; // Baseline consumption with random noise

    energy_stored_ -= consumption;

    // Enforce boundaries for energy storage
    if (energy_stored_ < 0.0)
    {
      energy_stored_ = 0.0;
      RCLCPP_WARN(this->get_logger(), "Energy storage depleted!");
    }

    // Decay critical load
    decay_critical_load();

    // Calculate available power
    double available_power = energy_stored_ - critical_load_ - non_critical_load_;
    if (available_power < 0.0) {
        available_power = 0.0;
    }

    std::ostringstream state_stream;
    state_stream << "EPS State: Energy Stored = " << energy_stored_
                 << " W, Available Power = " << available_power
                 << " W, Critical Load = " << critical_load_
                 << " W, Non-Critical Load = " << non_critical_load_
                 << " W, Baseline Consumption = " << consumption << " W.";

    auto state_msg = std_msgs::msg::String();
    state_msg.data = state_stream.str();
    RCLCPP_INFO(this->get_logger(), "%s", state_msg.data.c_str());
    state_publisher_->publish(state_msg);

    // Terminal output for live reporting
    std::cout << "[EPS Report] " << state_msg.data << std::endl;
  }

  // ROS Infrastructure
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr power_source_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr power_sink_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr critical_load_subscriber_;

  // EPS Properties
  double max_capacity_;      // Maximum energy storage capacity in W
  double energy_stored_;     // Current energy level in W
  double reserve_capacity_;  // Reserved capacity for critical loads
  double critical_load_;     // Current critical subsystem load in W
  double non_critical_load_; // Current non-critical subsystem load in W
  double baseline_consumption_; // Baseline energy consumption in W per timestep

  // Random noise generators
  std::mt19937 gen_;
  std::uniform_real_distribution<> noise_dist_;       // Noise for general updates
  std::uniform_real_distribution<> consumption_dist_; // Noise for baseline consumption
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EPSSubsystem>());
  rclcpp::shutdown();
  return 0;
}
