// Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

#ifndef ALOHAMINI_BASE_CONTROL__SYSTEM_HARDWARE_HPP_
#define ALOHAMINI_BASE_CONTROL__SYSTEM_HARDWARE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "alohamini_base_control/feetech_protocol.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace alohamini_base_control
{

// ros2_control SystemInterface for the AlohaMini three-omniwheel base.
//
// Talks Feetech protocol 0 directly over a serial port (no lerobot host).
// Exposes per-wheel velocity command + velocity/position state interfaces in
// rad/s; all whole-body kinematics live in OmniBaseController, not here.
class AlohaMiniBaseHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AlohaMiniBaseHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Write velocity operating mode + enable torque on every wheel.
  bool setupMotors();
  // Command zero velocity on every wheel (used on deactivate / error).
  void stopBase();

  // Per-wheel config and live state, indexed in joint declaration order.
  std::vector<std::uint8_t> motor_ids_;
  std::vector<double> hw_commands_vel_;   // rad/s, written by controller
  std::vector<double> hw_states_vel_;     // rad/s, read back from motors
  std::vector<double> hw_states_pos_;     // rad, integrated from velocity

  std::string serial_port_;
  int baud_rate_ = 1000000;
  int max_wheel_raw_ = 3000;  // matches lekiwi.py `max_raw`
  bool configure_motors_ = true;

  FeetechBus bus_;
};

}  // namespace alohamini_base_control

#endif  // ALOHAMINI_BASE_CONTROL__SYSTEM_HARDWARE_HPP_
