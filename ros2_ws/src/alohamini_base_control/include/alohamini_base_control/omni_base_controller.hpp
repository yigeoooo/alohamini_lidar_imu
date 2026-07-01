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

#ifndef ALOHAMINI_BASE_CONTROL__OMNI_BASE_CONTROLLER_HPP_
#define ALOHAMINI_BASE_CONTROL__OMNI_BASE_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace alohamini_base_control
{

// Whole-body controller for the three-omniwheel base.
//
// Subscribes /cmd_vel (Twist), maps body velocity -> three wheel velocities
// (rad/s) via the omni kinematics from lekiwi.py, and writes them to the
// hardware's per-wheel velocity command interfaces. Reads wheel velocity state
// back, runs the inverse kinematics to recover body velocity, integrates pose,
// and publishes /odom (+ optional odom->base_link TF).
class OmniBaseController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct BodyVelocity
  {
    double x = 0.0;    // m/s
    double y = 0.0;    // m/s
    double yaw = 0.0;  // rad/s
  };

  // Build the 3x3 kinematic matrix M and its inverse from wheel angles + base_radius.
  void buildKinematics();
  // Forward: body velocity -> wheel angular velocities (rad/s), with proportional
  // down-scaling so no wheel raw tick exceeds max_wheel_raw_ (matches lekiwi.py).
  std::array<double, 3> bodyToWheel(const BodyVelocity & v) const;
  // Inverse: wheel angular velocities (rad/s) -> body velocity.
  BodyVelocity wheelToBody(const std::array<double, 3> & wheel_radps) const;

  // Parameters.
  std::vector<std::string> wheel_names_;
  std::vector<double> wheel_angles_deg_{240.0, 0.0, 120.0};
  double angle_offset_deg_ = -90.0;
  double wheel_radius_ = 0.05;
  double base_radius_ = 0.125;
  int max_wheel_raw_ = 3000;
  double cmd_timeout_ = 0.5;
  std::string cmd_vel_topic_ = "/cmd_vel";
  std::string odom_topic_ = "/odom";
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";
  bool publish_tf_ = true;
  bool use_stamped_vel_ = false;
  double max_linear_speed_ = 0.0;   // 0 -> unlimited
  double max_lateral_speed_ = 0.0;
  double max_angular_speed_ = 0.0;
  std::array<double, 2> pose_cov_{0.10, 0.25};   // xy, yaw
  std::array<double, 2> twist_cov_{0.20, 0.35};  // xy, yaw

  // Kinematics matrix rows m_[i] = {cos(a), sin(a), base_radius}; inverse m_inv_.
  std::array<std::array<double, 3>, 3> m_{};
  std::array<std::array<double, 3>, 3> m_inv_{};

  // Latest command (written from subscription callback, read in update()).
  realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> rt_cmd_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_stamped_;
  rclcpp::Time last_cmd_time_;

  // Odometry state.
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> rt_tf_pub_;
};

}  // namespace alohamini_base_control

#endif  // ALOHAMINI_BASE_CONTROL__OMNI_BASE_CONTROLLER_HPP_
