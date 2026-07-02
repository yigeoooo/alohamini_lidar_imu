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

#include "alohamini_base_control/system_hardware.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alohamini_base_control
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
rclcpp::Logger logger() { return rclcpp::get_logger("AlohaMiniBaseHardware"); }
}  // namespace

hardware_interface::CallbackReturn AlohaMiniBaseHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Bus-level params from <hardware><param>.
  auto get_param = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return it != info_.hardware_parameters.end() ? it->second : def;
  };

  serial_port_ = get_param("serial_port", "/dev/am_arm_follower_left");
  baud_rate_ = std::stoi(get_param("baud_rate", "1000000"));
  max_wheel_raw_ = std::stoi(get_param("max_wheel_raw", "3000"));
  configure_motors_ = get_param("configure_motors", "true") == "true";
  use_sync_read_ = get_param("use_sync_read", "false") == "true";

  const std::size_t n = info_.joints.size();
  motor_ids_.assign(n, 0);
  hw_commands_vel_.assign(n, 0.0);
  hw_states_vel_.assign(n, 0.0);
  hw_states_pos_.assign(n, 0.0);

  for (std::size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];

    auto id_it = joint.parameters.find("motor_id");
    if (id_it == joint.parameters.end()) {
      RCLCPP_FATAL(logger(), "Joint '%s' is missing required param 'motor_id'.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    motor_ids_[i] = static_cast<std::uint8_t>(std::stoi(id_it->second));

    // Require exactly one velocity command interface.
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger(), "Joint '%s' must expose exactly one '%s' command interface.",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Require velocity (and optionally position) state interfaces.
    bool has_velocity_state = false;
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_state = true;
      }
    }
    if (!has_velocity_state) {
      RCLCPP_FATAL(
        logger(), "Joint '%s' must expose a '%s' state interface.",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    logger(), "AlohaMiniBaseHardware init: port=%s baud=%d wheels=%zu",
    serial_port_.c_str(), baud_rate_, n);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlohaMiniBaseHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bus_.open(serial_port_, baud_rate_)) {
    RCLCPP_FATAL(logger(), "Failed to open serial port: %s", bus_.lastError().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger(), "Serial port %s opened at %d baud.", serial_port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlohaMiniBaseHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bus_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool AlohaMiniBaseHardware::setupMotors()
{
  bool ok = true;
  for (std::uint8_t id : motor_ids_) {
    // IMPORTANT: on Feetech STS motors, Operating_Mode lives in the EEPROM area and
    // can only be changed while torque is disabled and the EEPROM lock is open
    // (Lock=0). If we write Operating_Mode with torque enabled / lock closed, the
    // change is silently rejected and the motor stays in position mode — then
    // Goal_Velocity (addr 46) has no effect and the wheels never spin.
    // This mirrors lekiwi.py configure(): disable_torque() (Torque_Enable=0, Lock=0)
    // BEFORE writing Operating_Mode.
    ok &= bus_.writeRegister(id, reg::kTorqueEnable, 0, 1);
    ok &= bus_.writeRegister(id, reg::kLock, 0, 1);

    if (configure_motors_) {
      // Minimise the return-delay and speed up accel/decel (matches configure_motors()).
      ok &= bus_.writeRegister(id, reg::kReturnDelayTime, 0, 1);
      ok &= bus_.writeRegister(id, reg::kMaximumAcceleration, 254, 1);
      ok &= bus_.writeRegister(id, reg::kAcceleration, 254, 1);
    }

    // Now switch to velocity mode (torque is off / lock open, so this takes effect).
    ok &= bus_.writeRegister(id, reg::kOperatingMode, operating_mode::kVelocity, 1);

    // Re-enable torque so Goal_Velocity actually drives the wheel.
    ok &= bus_.writeRegister(id, reg::kTorqueEnable, 1, 1);
  }
  return ok;
}

void AlohaMiniBaseHardware::stopBase()
{
  std::vector<std::pair<std::uint8_t, std::int16_t>> zero;
  zero.reserve(motor_ids_.size());
  for (std::uint8_t id : motor_ids_) {
    zero.emplace_back(id, static_cast<std::int16_t>(0));
  }
  bus_.syncWriteGoalVelocity(zero);
}

hardware_interface::CallbackReturn AlohaMiniBaseHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bus_.isOpen()) {
    RCLCPP_FATAL(logger(), "Serial port not open on activate.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!setupMotors()) {
    RCLCPP_WARN(logger(), "Some motor setup writes failed; continuing.");
  }
  std::fill(hw_commands_vel_.begin(), hw_commands_vel_.end(), 0.0);
  stopBase();
  RCLCPP_INFO(logger(), "AlohaMiniBaseHardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlohaMiniBaseHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopBase();
  RCLCPP_INFO(logger(), "AlohaMiniBaseHardware deactivated; base stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlohaMiniBaseHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopBase();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AlohaMiniBaseHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AlohaMiniBaseHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type AlohaMiniBaseHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::vector<std::int16_t> raw;
  const bool ok = use_sync_read_
    ? bus_.syncReadPresentVelocity(motor_ids_, raw)
    : bus_.readPresentVelocityIndividually(motor_ids_, raw);
  if (!ok) {
    // Keep last good values rather than spiking; surface the issue but stay alive.
    RCLCPP_DEBUG(logger(), "velocity read failed: %s", bus_.lastError().c_str());
    return hardware_interface::return_type::OK;
  }

  const double dt = period.seconds();
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    // raw tick -> deg/s -> rad/s.
    const double degps = rawToDegps(raw[i]);
    hw_states_vel_[i] = degps * kDegToRad;
    if (dt > 0.0 && dt < 1.0) {
      hw_states_pos_[i] += hw_states_vel_[i] * dt;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlohaMiniBaseHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // rad/s -> deg/s -> raw tick. Proportional scaling so no single wheel exceeds
  // max_wheel_raw_, mirroring lekiwi.py `_body_to_wheel_raw`.
  std::vector<double> degps(motor_ids_.size());
  double max_raw_computed = 0.0;
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    degps[i] = hw_commands_vel_[i] * kRadToDeg;
    max_raw_computed = std::max(max_raw_computed, std::abs(degps[i]) * kStepsPerDeg);
  }
  double scale = 1.0;
  if (max_wheel_raw_ > 0 && max_raw_computed > static_cast<double>(max_wheel_raw_)) {
    scale = static_cast<double>(max_wheel_raw_) / max_raw_computed;
  }

  std::vector<std::pair<std::uint8_t, std::int16_t>> id_raw;
  id_raw.reserve(motor_ids_.size());
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    id_raw.emplace_back(motor_ids_[i], degpsToRaw(degps[i] * scale));
  }
  bus_.syncWriteGoalVelocity(id_raw);
  return hardware_interface::return_type::OK;
}

}  // namespace alohamini_base_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  alohamini_base_control::AlohaMiniBaseHardware, hardware_interface::SystemInterface)
