// Copyright 2026 Kevin Eppacher
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace lerobot_teleop
{

class MockTeleop : public rclcpp::Node
{
public:
  MockTeleop()
  : Node("mock_teleop")
  {
    declareParameters();
    loadParameters();

    leader_joint_state_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>(
      leader_joint_states_topic_,
      rclcpp::SensorDataQoS());

    follower_joint_state_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>(
      follower_joint_states_topic_,
      rclcpp::SensorDataQoS());

    follower_command_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
      follower_command_topic_,
      10,
      std::bind(
        &MockTeleop::followerCommandCallback,
        this,
        std::placeholders::_1));

    guiding_mode_service_ =
      create_service<std_srvs::srv::SetBool>(
      guiding_mode_service_name_,
      std::bind(
        &MockTeleop::guidingModeCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto period =
      std::chrono::duration<double>(
      1.0 / publish_rate_);

    publish_timer_ =
      create_wall_timer(
      std::chrono::duration_cast<
        std::chrono::nanoseconds>(period),
      std::bind(
        &MockTeleop::publishJointStates,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Mock teleop started");

    RCLCPP_INFO(
      get_logger(),
      "Leader JointState: %s",
      leader_joint_states_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Follower JointState: %s",
      follower_joint_states_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Follower command: %s",
      follower_command_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Guiding mode service: %s",
      guiding_mode_service_name_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter(
      "publish_rate",
      50.0);

    declare_parameter(
      "leader_joint_states_topic",
      "/leader/joint_states");

    declare_parameter(
      "follower_joint_states_topic",
      "/follower/joint_states");

    declare_parameter(
      "follower_command_topic",
      "/follower/position_controller/commands");

    declare_parameter(
      "guiding_mode_service",
      "/leader/leader_so101/guiding_mode");

    declare_parameter(
      "leader_positions",
      std::vector<double>{
        0.5,
        -0.3,
        0.2,
        0.1,
        -0.4,
        0.5});

    declare_parameter(
      "follower_positions",
      std::vector<double>{
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0});
  }

  void loadParameters()
  {
    publish_rate_ =
      get_parameter(
      "publish_rate").as_double();

    leader_joint_states_topic_ =
      get_parameter(
      "leader_joint_states_topic").as_string();

    follower_joint_states_topic_ =
      get_parameter(
      "follower_joint_states_topic").as_string();

    follower_command_topic_ =
      get_parameter(
      "follower_command_topic").as_string();

    guiding_mode_service_name_ =
      get_parameter(
      "guiding_mode_service").as_string();

    leader_positions_ =
      get_parameter(
      "leader_positions").as_double_array();

    follower_positions_ =
      get_parameter(
      "follower_positions").as_double_array();

    if (publish_rate_ <= 0.0) {
      throw std::runtime_error(
              "Publish rate must be greater than zero");
    }

    if (
      leader_positions_.size() != joint_names_.size() ||
      follower_positions_.size() != joint_names_.size())
    {
      throw std::runtime_error(
              "Leader and follower positions must match joint count");
    }
  }

  void publishJointStates()
  {
    publishLeaderJointState();
    publishFollowerJointState();
  }

  void publishLeaderJointState()
  {
    sensor_msgs::msg::JointState message;

    message.header.stamp = now();
    message.name = joint_names_;
    message.position = leader_positions_;

    leader_joint_state_publisher_->publish(
      message);
  }

  void publishFollowerJointState()
  {
    sensor_msgs::msg::JointState message;

    message.header.stamp = now();
    message.name = joint_names_;
    message.position = follower_positions_;

    follower_joint_state_publisher_->publish(
      message);
  }

  void followerCommandCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != joint_names_.size()) {
      RCLCPP_ERROR(
        get_logger(),
        "Received follower command with unexpected size: %zu",
        msg->data.size());

      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Follower command:");

    for (std::size_t i = 0; i < msg->data.size(); ++i) {
      RCLCPP_INFO(
        get_logger(),
        "  %-15s %+.4f rad",
        joint_names_[i].c_str(),
        msg->data[i]);
    }

    RCLCPP_INFO(
      get_logger(),
      "--------------------------------");
  }

  void guidingModeCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    guiding_mode_enabled_ =
      request->data;

    response->success = true;

    response->message =
      guiding_mode_enabled_ ?
      "Mock guiding mode enabled" :
      "Mock guiding mode disabled";

    RCLCPP_INFO(
      get_logger(),
      "Guiding mode %s",
      guiding_mode_enabled_ ?
      "enabled" :
      "disabled");
  }

  const std::vector<std::string> joint_names_{
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
  };

  double publish_rate_{50.0};

  std::string leader_joint_states_topic_;
  std::string follower_joint_states_topic_;
  std::string follower_command_topic_;
  std::string guiding_mode_service_name_;

  std::vector<double> leader_positions_;
  std::vector<double> follower_positions_;

  bool guiding_mode_enabled_{false};

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
    leader_joint_state_publisher_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
    follower_joint_state_publisher_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    follower_command_subscription_;

  rclcpp::Service<
    std_srvs::srv::SetBool>::SharedPtr
    guiding_mode_service_;

  rclcpp::TimerBase::SharedPtr
    publish_timer_;
};

}  // namespace lerobot_teleop

int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);

  rclcpp::spin(
    std::make_shared<
      lerobot_teleop::MockTeleop>());

  rclcpp::shutdown();

  return 0;
}
