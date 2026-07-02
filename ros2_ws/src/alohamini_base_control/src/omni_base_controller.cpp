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

#include "alohamini_base_control/omni_base_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "alohamini_base_control/feetech_protocol.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace alohamini_base_control
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double clampSym(double v, double limit)
{
  if (limit <= 0.0) {
    return v;
  }
  return std::max(-limit, std::min(limit, v));
}
}  // namespace

controller_interface::CallbackReturn OmniBaseController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("wheel_names", wheel_names_);
    auto_declare<std::vector<double>>("wheel_angles_deg", wheel_angles_deg_);
    auto_declare<double>("angle_offset_deg", angle_offset_deg_);
    auto_declare<double>("wheel_radius", wheel_radius_);
    auto_declare<double>("base_radius", base_radius_);
    auto_declare<int>("max_wheel_raw", max_wheel_raw_);
    auto_declare<double>("cmd_timeout", cmd_timeout_);
    auto_declare<std::string>("cmd_vel_topic", cmd_vel_topic_);
    auto_declare<std::string>("odom_topic", odom_topic_);
    auto_declare<std::string>("odom_frame", odom_frame_);
    auto_declare<std::string>("base_frame", base_frame_);
    auto_declare<bool>("publish_tf", publish_tf_);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
    auto_declare<double>("max_linear_speed", max_linear_speed_);
    auto_declare<double>("max_lateral_speed", max_lateral_speed_);
    auto_declare<double>("max_angular_speed", max_angular_speed_);
    auto_declare<double>("pose_covariance_xy", pose_cov_[0]);
    auto_declare<double>("pose_covariance_yaw", pose_cov_[1]);
    auto_declare<double>("twist_covariance_xy", twist_cov_[0]);
    auto_declare<double>("twist_covariance_yaw", twist_cov_[1]);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception during on_init: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void OmniBaseController::buildKinematics()
{
  // m[i] = {cos(a_i), sin(a_i), base_radius}, a_i = wheel_angle_i + offset.
  // Mirrors lekiwi.py: angles = radians([240, 0, 120] - 90).
  for (std::size_t i = 0; i < 3; ++i) {
    const double a = (wheel_angles_deg_[i] + angle_offset_deg_) * kDegToRad;
    m_[i] = {std::cos(a), std::sin(a), base_radius_};
  }

  // Closed-form 3x3 inverse.
  const auto & m = m_;
  const double det =
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  const double inv_det = (std::abs(det) > 1e-12) ? 1.0 / det : 0.0;

  m_inv_[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
  m_inv_[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
  m_inv_[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
  m_inv_[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
  m_inv_[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
  m_inv_[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
  m_inv_[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
  m_inv_[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
  m_inv_[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
}

std::array<double, 3> OmniBaseController::bodyToWheel(const BodyVelocity & v) const
{
  // velocity_vector = [-x, -y, yaw]   (note x,y negation, matches lekiwi.py).
  const std::array<double, 3> vel{-v.x, -v.y, v.yaw};

  std::array<double, 3> wheel_radps{};
  std::array<double, 3> wheel_degps{};
  double max_raw_computed = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    const double linear = m_[i][0] * vel[0] + m_[i][1] * vel[1] + m_[i][2] * vel[2];
    wheel_radps[i] = linear / wheel_radius_;
    wheel_degps[i] = wheel_radps[i] * kRadToDeg;
    max_raw_computed = std::max(max_raw_computed, std::abs(wheel_degps[i]) * kStepsPerDeg);
  }

  // Proportional down-scaling so the largest wheel raw tick stays within max_wheel_raw_.
  if (max_wheel_raw_ > 0 && max_raw_computed > static_cast<double>(max_wheel_raw_)) {
    const double scale = static_cast<double>(max_wheel_raw_) / max_raw_computed;
    for (auto & w : wheel_radps) {
      w *= scale;
    }
  }
  return wheel_radps;
}

OmniBaseController::BodyVelocity OmniBaseController::wheelToBody(
  const std::array<double, 3> & wheel_radps) const
{
  std::array<double, 3> wheel_linear{};
  for (std::size_t i = 0; i < 3; ++i) {
    wheel_linear[i] = wheel_radps[i] * wheel_radius_;
  }
  BodyVelocity v;
  v.x = m_inv_[0][0] * wheel_linear[0] + m_inv_[0][1] * wheel_linear[1] +
    m_inv_[0][2] * wheel_linear[2];
  v.y = m_inv_[1][0] * wheel_linear[0] + m_inv_[1][1] * wheel_linear[1] +
    m_inv_[1][2] * wheel_linear[2];
  v.yaw = m_inv_[2][0] * wheel_linear[0] + m_inv_[2][1] * wheel_linear[1] +
    m_inv_[2][2] * wheel_linear[2];
  return v;
}

controller_interface::InterfaceConfiguration
OmniBaseController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : wheel_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::InterfaceConfiguration
OmniBaseController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : wheel_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::CallbackReturn OmniBaseController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  wheel_names_ = node->get_parameter("wheel_names").as_string_array();
  wheel_angles_deg_ = node->get_parameter("wheel_angles_deg").as_double_array();
  angle_offset_deg_ = node->get_parameter("angle_offset_deg").as_double();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  base_radius_ = node->get_parameter("base_radius").as_double();
  max_wheel_raw_ = static_cast<int>(node->get_parameter("max_wheel_raw").as_int());
  cmd_timeout_ = node->get_parameter("cmd_timeout").as_double();
  cmd_vel_topic_ = node->get_parameter("cmd_vel_topic").as_string();
  odom_topic_ = node->get_parameter("odom_topic").as_string();
  odom_frame_ = node->get_parameter("odom_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();
  publish_tf_ = node->get_parameter("publish_tf").as_bool();
  use_stamped_vel_ = node->get_parameter("use_stamped_vel").as_bool();
  max_linear_speed_ = node->get_parameter("max_linear_speed").as_double();
  max_lateral_speed_ = node->get_parameter("max_lateral_speed").as_double();
  max_angular_speed_ = node->get_parameter("max_angular_speed").as_double();
  pose_cov_[0] = node->get_parameter("pose_covariance_xy").as_double();
  pose_cov_[1] = node->get_parameter("pose_covariance_yaw").as_double();
  twist_cov_[0] = node->get_parameter("twist_covariance_xy").as_double();
  twist_cov_[1] = node->get_parameter("twist_covariance_yaw").as_double();

  if (wheel_names_.size() != 3 || wheel_angles_deg_.size() != 3) {
    RCLCPP_FATAL(
      node->get_logger(), "wheel_names and wheel_angles_deg must each have 3 entries.");
    return controller_interface::CallbackReturn::ERROR;
  }

  buildKinematics();

  // /cmd_vel subscription (plain Twist by default; stamped optional).
  if (use_stamped_vel_) {
    cmd_sub_stamped_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
        rt_cmd_.set(msg);
      });
  } else {
    cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
        auto stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
        stamped->twist = *msg;
        stamped->header.stamp = get_node()->now();
        rt_cmd_.set(stamped);
      });
  }

  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS());
  rt_odom_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);

  if (publish_tf_) {
    tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
    rt_tf_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub_);
  }

  RCLCPP_INFO(
    node->get_logger(),
    "OmniBaseController configured: wheels=[%s,%s,%s] r=%.3f R=%.3f, sub %s, pub %s",
    wheel_names_[0].c_str(), wheel_names_[1].c_str(), wheel_names_[2].c_str(),
    wheel_radius_, base_radius_, cmd_vel_topic_.c_str(), odom_topic_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  odom_x_ = odom_y_ = odom_yaw_ = 0.0;
  last_cmd_time_ = get_node()->now();
  rt_cmd_.set(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Command zero velocity on the way out.
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OmniBaseController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // ---- 1. Resolve the active command (with watchdog). ----
  std::shared_ptr<geometry_msgs::msg::TwistStamped> cmd_msg;
  rt_cmd_.get(cmd_msg);

  BodyVelocity cmd;
  bool have_cmd = false;
  if (cmd_msg) {
    const double age = (time - rclcpp::Time(cmd_msg->header.stamp)).seconds();
    if (age <= cmd_timeout_ && age >= -1.0) {
      cmd.x = clampSym(cmd_msg->twist.linear.x, max_linear_speed_);
      cmd.y = clampSym(cmd_msg->twist.linear.y, max_lateral_speed_);
      cmd.yaw = clampSym(cmd_msg->twist.angular.z, max_angular_speed_);
      have_cmd = true;
    }
  }
  if (!have_cmd) {
    cmd = BodyVelocity{};  // stale or no command -> stop (watchdog).
  }

  // NOTE: neither the command nor the odom velocity is rotated here. On real hardware
  // the wheel-level kinematics already produce correct physical motion for /cmd_vel,
  // and odom is published in `base_frame` (base_footprint), which a static URDF joint
  // rotates onto the SolidWorks base_link. So the whole chain stays self-consistent
  // and REP-103 compliant (x=forward, y=left) without any velocity fudging.

  // ---- 2. Forward kinematics -> wheel velocity commands. ----
  const std::array<double, 3> wheel_cmd = bodyToWheel(cmd);
  for (std::size_t i = 0; i < command_interfaces_.size() && i < 3; ++i) {
    command_interfaces_[i].set_value(wheel_cmd[i]);
  }

  // ---- 3. Read wheel velocity state -> inverse kinematics -> integrate odom. ----
  std::array<double, 3> wheel_state{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < state_interfaces_.size() && i < 3; ++i) {
    wheel_state[i] = state_interfaces_[i].get_value();
  }
  BodyVelocity vel = wheelToBody(wheel_state);
  // lekiwi's forward kinematics negate x,y (velocity_vector = [-x,-y,theta]) but the
  // inverse does not, so the body velocity recovered from wheel feedback comes out
  // 180 deg flipped relative to the physical motion. Negate x,y here so /odom reports
  // motion in the same direction the base actually moves (+x = forward in
  // base_footprint), keeping the odom arrow, trajectory and mesh all consistent.
  vel.x = -vel.x;
  vel.y = -vel.y;

  // Throttled diagnostic (once per second): what this controller reads on its claimed
  // state interfaces and what it computes for odom. DEBUG so it is silent by default;
  // enable with `ros2 service call /omni_base_controller/set_logger_levels ...` or a
  // launch --log-level to debug "wheels move but /odom stays 0" type issues.
  RCLCPP_DEBUG_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "n_state_if=%zu wheel_state=[%.3f %.3f %.3f] rad/s -> vel x=%.3f y=%.3f yaw=%.3f | "
    "odom=[%.3f %.3f %.3f]",
    state_interfaces_.size(), wheel_state[0], wheel_state[1], wheel_state[2],
    vel.x, vel.y, vel.yaw, odom_x_, odom_y_, odom_yaw_);

  const double dt = period.seconds();
  if (dt > 0.0 && dt < 1.0) {
    const double cos_yaw = std::cos(odom_yaw_);
    const double sin_yaw = std::sin(odom_yaw_);
    odom_x_ += (vel.x * cos_yaw - vel.y * sin_yaw) * dt;
    odom_y_ += (vel.x * sin_yaw + vel.y * cos_yaw) * dt;
    odom_yaw_ = std::atan2(
      std::sin(odom_yaw_ + vel.yaw * dt), std::cos(odom_yaw_ + vel.yaw * dt));
  }

  // ---- 4. Publish odom + TF. ----
  const double half = odom_yaw_ * 0.5;
  const double qz = std::sin(half);
  const double qw = std::cos(half);

  if (rt_odom_pub_ && rt_odom_pub_->trylock()) {
    auto & odom = rt_odom_pub_->msg_;
    odom.header.stamp = time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = vel.x;
    odom.twist.twist.linear.y = vel.y;
    odom.twist.twist.angular.z = vel.yaw;
    odom.pose.covariance[0] = pose_cov_[0];
    odom.pose.covariance[7] = pose_cov_[0];
    odom.pose.covariance[35] = pose_cov_[1];
    odom.twist.covariance[0] = twist_cov_[0];
    odom.twist.covariance[7] = twist_cov_[0];
    odom.twist.covariance[35] = twist_cov_[1];
    rt_odom_pub_->unlockAndPublish();
  }

  if (publish_tf_ && rt_tf_pub_ && rt_tf_pub_->trylock()) {
    auto & tf_msg = rt_tf_pub_->msg_;
    tf_msg.transforms.resize(1);
    auto & t = tf_msg.transforms[0];
    t.header.stamp = time;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = odom_x_;
    t.transform.translation.y = odom_y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;
    rt_tf_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace alohamini_base_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  alohamini_base_control::OmniBaseController, controller_interface::ControllerInterface)
