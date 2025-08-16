#include "rotatum_arm_hardware/rotatum_arm_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/subscription.hpp"

using namespace std::chrono_literals;

namespace rotatum_arm
{
hardware_interface::CallbackReturn RotatumArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  subscription_ = node1->create_subscription<std_msgs::msg::Float32MultiArray>(
        "topic", 1000, std::bind(&RotatumArmHardware::topic_callback, this, std::placeholders::_1));

  publisher_ = node1->create_publisher<std_msgs::msg::Float32MultiArray>("topic2", 1000);
  //timer_ = node1->create_wall_timer(
  //    1000ms, std::bind(&MinimalPublisher::timer_callback, this));

  executor.add_node(node1);

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_actual_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RotatumArmHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RotatumArmHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RotatumArmHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RotatumArmHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RotatumArmHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RotatumArmHardware"), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RotatumArmHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
    joint_actual_positions_[i] = 0;
  }

  executor.spin_once();

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = joint_actual_positions_[i];
    hw_commands_[i] = joint_actual_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RotatumArmHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RotatumArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RotatumArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RotatumArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RotatumArmHardware"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RotatumArmHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  executor.spin_once();

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = joint_actual_positions_[i];
    hw_commands_[i] = joint_actual_positions_[i];
  }



  RCLCPP_INFO(rclcpp::get_logger("RotatumArmHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RotatumArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RotatumArmHardware"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RotatumArmHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("RotatumArmHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RotatumArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  executor.spin_once();

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //RCLCPP_INFO(rclcpp::get_logger("RotatumArmHardware"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    //hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;

    hw_states_[i] = joint_actual_positions_[i];

    //RCLCPP_INFO(
    //  rclcpp::get_logger("RotatumArmHardware"), "Got state %.5f for joint %d!",
    //  hw_states_[i], i);
  }

	RCLCPP_INFO(
        rclcpp::get_logger("RotatumArmHardware"), "Got state   %.5f for joint %d!",
        hw_states_[5], 6);

  //RCLCPP_INFO(rclcpp::get_logger("RotatumArmHardware"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RotatumArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //RCLCPP_INFO(rclcpp::get_logger("RotatumArmHardware"), "Writing...");

  auto message = std_msgs::msg::Float32MultiArray();

  message.data.resize(info_.joints.size(), 0.0);

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    message.data[i] = hw_commands_[i];
    //RCLCPP_INFO(
    //  rclcpp::get_logger("RotatumArmHardware"), "Got command %.5f for joint %d!",
    //  hw_commands_[i], i);
  }
  
   RCLCPP_INFO(
      rclcpp::get_logger("RotatumArmHardware"), "Got command %.5f for joint %d!",
      hw_commands_[5], 6);
  
  
  publisher_->publish(message);
  //RCLCPP_INFO(
   // rclcpp::get_logger("RotatumArmHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

void RotatumArmHardware::topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
   // RCLCPP_INFO(
   //   rclcpp::get_logger("RotatumArmHardware"), "I heared : %.5f ", msg->data[5]);

    joint_actual_positions_[0] = msg->data[0];
    joint_actual_positions_[1] = msg->data[1];
    joint_actual_positions_[2] = msg->data[2];
    joint_actual_positions_[3] = msg->data[3];
    joint_actual_positions_[4] = msg->data[4];
    joint_actual_positions_[5] = msg->data[5];
}



}  // namespace rotatum_arm

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rotatum_arm::RotatumArmHardware, hardware_interface::SystemInterface)
