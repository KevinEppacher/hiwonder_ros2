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

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace lerobot_teleop
{

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("teleop_node")
  {
    joint_state_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
      "/leader/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(
        &TeleopNode::jointStateCallback,
        this,
        std::placeholders::_1));

    command_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      "/follower/position_controller/commands",
      10);

    RCLCPP_INFO(
      get_logger(),
      "Teleop node started: /leader/joint_states -> "
      "/follower/position_controller/commands");
  }

private:
  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.size() != msg->position.size()) {
      RCLCPP_WARN(
        get_logger(),
        "JointState name and position arrays have different sizes");
      return;
    }

    std::unordered_map<std::string, double> positions;
    positions.reserve(msg->name.size());

    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      positions[msg->name[i]] = msg->position[i];
    }

    std_msgs::msg::Float64MultiArray command;
    command.data.reserve(kJointNames.size());

    for (const auto & joint_name : kJointNames) {
      const auto it = positions.find(joint_name);

      if (it == positions.end()) {
        RCLCPP_WARN(
          get_logger(),
          "Joint '%s' missing from leader JointState",
          joint_name);
        return;
      }

      command.data.push_back(it->second);
    }

    command_publisher_->publish(command);
  }

  static constexpr std::array<const char *, 6> kJointNames = {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
  };

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    command_publisher_;
};

}  // namespace lerobot_teleop

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<lerobot_teleop::TeleopNode>());

  rclcpp::shutdown();

  return 0;
}
