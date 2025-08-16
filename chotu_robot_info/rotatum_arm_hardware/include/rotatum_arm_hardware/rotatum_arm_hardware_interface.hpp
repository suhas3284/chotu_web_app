// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROTATUM_ARM_HARDWARE_HPP_
#define ROTATUM_ARM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


namespace rotatum_arm
{
class RotatumArmHardware : public rclcpp::Node, public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RotatumArmHardware)

  RotatumArmHardware()
  : Node("RotatumArmHardware")
  {
    node1 = rclcpp::Node::make_shared("node1");
  }

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_ = nullptr;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_ = nullptr;

rclcpp::executors::SingleThreadedExecutor executor;

std::shared_ptr<rclcpp::Node> node1;


private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  std::vector<double> joint_actual_positions_;


};

}  // namespace rotatum_arm

#endif  // ROTATUM_ARM_HARDWARE_HPP_