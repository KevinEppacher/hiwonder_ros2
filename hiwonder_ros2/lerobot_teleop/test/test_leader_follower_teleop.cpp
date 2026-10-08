// Copyright 2026 Kevin Eppacher
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "lerobot_teleop/leader_follower_teleop.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

namespace lerobot_teleop
{

class LeaderFollowerTeleopTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    mock_node_ =
      std::make_shared<rclcpp::Node>(
      "teleop_test_mock");

    rclcpp::NodeOptions options;

    options.parameter_overrides({
      rclcpp::Parameter(
        "update_rate",
        100.0),
      rclcpp::Parameter(
        "initial_motion.duration",
        0.2),
      rclcpp::Parameter(
        "low_pass_filter.enabled",
        true),
      rclcpp::Parameter(
        "low_pass_filter.cutoff_frequency",
        5.0),
      rclcpp::Parameter(
        "safety.leader_timeout",
        0.5)
    });

    teleop_ =
      std::make_shared<LeaderFollowerTeleop>(
      options);

    leader_publisher_ =
      mock_node_->create_publisher<
      sensor_msgs::msg::JointState>(
      "/leader/joint_states",
      rclcpp::SensorDataQoS());

    follower_publisher_ =
      mock_node_->create_publisher<
      sensor_msgs::msg::JointState>(
      "/follower/joint_states",
      rclcpp::SensorDataQoS());

    command_subscription_ =
      mock_node_->create_subscription<
      std_msgs::msg::Float64MultiArray>(
      "/follower/position_controller/commands",
      10,
      [this](
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        received_commands_.push_back(
          msg->data);
      });

    guiding_service_ =
      mock_node_->create_service<
      std_srvs::srv::SetBool>(
      "/leader/leader_so101/guiding_mode",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
      {
        guiding_enabled_ =
          request->data;

        ++guiding_service_calls_;

        response->success = true;
        response->message =
          "Mock guiding mode changed";
      });

    executor_.add_node(
      mock_node_);

    executor_.add_node(
      teleop_->get_node_base_interface());

    spinFor(100ms);
  }

  void TearDown() override
  {
    executor_.remove_node(
      teleop_->get_node_base_interface());

    executor_.remove_node(
      mock_node_);

    teleop_.reset();
    mock_node_.reset();

    received_commands_.clear();
  }

  sensor_msgs::msg::JointState createJointState(
    const std::vector<double> & positions)
  {
    sensor_msgs::msg::JointState message;

    message.header.stamp =
      mock_node_->now();

    message.name = joint_names_;
    message.position = positions;

    return message;
  }

  void publishLeader(
    const std::vector<double> & positions)
  {
    leader_publisher_->publish(
      createJointState(positions));
  }

  void publishFollower(
    const std::vector<double> & positions)
  {
    follower_publisher_->publish(
      createJointState(positions));
  }

  void publishJointStates(
    const std::vector<double> & leader,
    const std::vector<double> & follower)
  {
    publishLeader(leader);
    publishFollower(follower);
  }

  void spinFor(
    std::chrono::milliseconds duration)
  {
    const auto end =
      std::chrono::steady_clock::now() +
      duration;

    while (
      std::chrono::steady_clock::now() < end)
    {
      executor_.spin_some();

      std::this_thread::sleep_for(
        2ms);
    }
  }

  void publishFor(
    const std::vector<double> & leader,
    const std::vector<double> & follower,
    std::chrono::milliseconds duration)
  {
    const auto end =
      std::chrono::steady_clock::now() +
      duration;

    while (
      std::chrono::steady_clock::now() < end)
    {
      publishJointStates(
        leader,
        follower);

      spinFor(10ms);
    }
  }

  void activateTeleop()
  {
    const auto configured_state =
      teleop_->configure();

    ASSERT_EQ(
      configured_state.id(),
      lifecycle_msgs::msg::State::
      PRIMARY_STATE_INACTIVE);

    const auto active_state =
      teleop_->activate();

    ASSERT_EQ(
      active_state.id(),
      lifecycle_msgs::msg::State::
      PRIMARY_STATE_ACTIVE);
  }

  void expectCommandWithinBounds(
    const std::vector<double> & command,
    const std::vector<double> & start,
    const std::vector<double> & target)
  {
    ASSERT_EQ(
      command.size(),
      start.size());

    ASSERT_EQ(
      command.size(),
      target.size());

    for (std::size_t i = 0;
      i < command.size();
      ++i)
    {
      const double minimum =
        std::min(
        start[i],
        target[i]);

      const double maximum =
        std::max(
        start[i],
        target[i]);

      EXPECT_GE(
        command[i],
        minimum - kTolerance);

      EXPECT_LE(
        command[i],
        maximum + kTolerance);
    }
  }

  void expectNear(
    const std::vector<double> & actual,
    const std::vector<double> & expected,
    double tolerance)
  {
    ASSERT_EQ(
      actual.size(),
      expected.size());

    for (std::size_t i = 0;
      i < actual.size();
      ++i)
    {
      EXPECT_NEAR(
        actual[i],
        expected[i],
        tolerance);
    }
  }

  const std::vector<std::string> joint_names_{
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
  };

  static constexpr double kTolerance =
    1e-6;

  rclcpp::executors::SingleThreadedExecutor
    executor_;

  std::shared_ptr<rclcpp::Node>
    mock_node_;

  std::shared_ptr<LeaderFollowerTeleop>
    teleop_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
    leader_publisher_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
    follower_publisher_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    command_subscription_;

  rclcpp::Service<
    std_srvs::srv::SetBool>::SharedPtr
    guiding_service_;

  std::vector<std::vector<double>>
    received_commands_;

  bool guiding_enabled_{false};

  std::size_t guiding_service_calls_{0};
};

TEST_F(
  LeaderFollowerTeleopTest,
  AlignmentTrajectoryStaysWithinBounds)
{
  const std::vector<double> leader{
    0.5,
    -0.3,
    0.2,
    0.1,
    -0.4,
    0.5
  };

  const std::vector<double> follower{
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  };

  activateTeleop();

  publishFor(
    leader,
    follower,
    500ms);

  ASSERT_FALSE(
    received_commands_.empty());

  for (const auto & command :
    received_commands_)
  {
    expectCommandWithinBounds(
      command,
      follower,
      leader);
  }

  expectNear(
    received_commands_.back(),
    leader,
    0.02);

  EXPECT_TRUE(
    guiding_enabled_);

  EXPECT_GE(
    guiding_service_calls_,
    1U);
}

TEST_F(
  LeaderFollowerTeleopTest,
  TracksLeaderTrajectoryAfterAlignment)
{
  const std::vector<double> follower_initial{
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  };

  const std::vector<double> leader_initial{
    0.2,
    -0.1,
    0.1,
    0.0,
    -0.1,
    0.2
  };

  const std::vector<double> leader_target{
    0.6,
    -0.4,
    0.3,
    0.2,
    -0.5,
    0.7
  };

  activateTeleop();

  publishFor(
    leader_initial,
    follower_initial,
    500ms);

  ASSERT_TRUE(
    guiding_enabled_);

  ASSERT_FALSE(
    received_commands_.empty());

  expectNear(
    received_commands_.back(),
    leader_initial,
    0.02);

  received_commands_.clear();

  publishFor(
    leader_target,
    leader_initial,
    500ms);

  ASSERT_FALSE(
    received_commands_.empty());

  for (const auto & command :
    received_commands_)
  {
    expectCommandWithinBounds(
      command,
      leader_initial,
      leader_target);
  }

  expectNear(
    received_commands_.back(),
    leader_target,
    0.02);
}

TEST_F(
  LeaderFollowerTeleopTest,
  TrackingTrajectoryIsMonotonic)
{
  const std::vector<double> follower_initial{
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  };

  const std::vector<double> leader_initial{
    0.1,
    -0.1,
    0.1,
    -0.1,
    0.1,
    -0.1
  };

  const std::vector<double> leader_target{
    0.6,
    -0.6,
    0.5,
    -0.4,
    0.3,
    -0.2
  };

  activateTeleop();

  publishFor(
    leader_initial,
    follower_initial,
    500ms);

  ASSERT_TRUE(
    guiding_enabled_);

  received_commands_.clear();

  publishFor(
    leader_target,
    leader_initial,
    500ms);

  ASSERT_GT(
    received_commands_.size(),
    2U);

  for (std::size_t sample = 1;
    sample < received_commands_.size();
    ++sample)
  {
    const auto & previous =
      received_commands_[sample - 1];

    const auto & current =
      received_commands_[sample];

    ASSERT_EQ(
      previous.size(),
      leader_target.size());

    ASSERT_EQ(
      current.size(),
      leader_target.size());

    for (std::size_t joint = 0;
      joint < leader_target.size();
      ++joint)
    {
      if (
        leader_target[joint] >
        leader_initial[joint])
      {
        EXPECT_GE(
          current[joint] + kTolerance,
          previous[joint]);
      } else if (
        leader_target[joint] <
        leader_initial[joint])
      {
        EXPECT_LE(
          current[joint] - kTolerance,
          previous[joint]);
      }
    }
  }

  expectNear(
    received_commands_.back(),
    leader_target,
    0.02);
}

TEST_F(
  LeaderFollowerTeleopTest,
  MapsShuffledJointStatesToControllerOrder)
{
  const std::vector<std::string> shuffled_joint_names{
    "elbow_flex",
    "gripper",
    "shoulder_lift",
    "shoulder_pan",
    "wrist_flex",
    "wrist_roll"
  };

  const std::vector<double> shuffled_leader_positions{
    0.3,   // elbow_flex
    0.6,   // gripper
    -0.2,  // shoulder_lift
    0.5,   // shoulder_pan
    -0.4,  // wrist_flex
    0.1    // wrist_roll
  };

  const std::vector<double> shuffled_follower_positions{
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  };

  const std::vector<double> expected_command{
    0.5,   // shoulder_pan
    -0.2,  // shoulder_lift
    0.3,   // elbow_flex
    -0.4,  // wrist_flex
    0.1,   // wrist_roll
    0.6    // gripper
  };

  activateTeleop();

  const auto end =
    std::chrono::steady_clock::now() +
    500ms;

  while (
    std::chrono::steady_clock::now() < end)
  {
    sensor_msgs::msg::JointState leader_message;
    leader_message.header.stamp =
      mock_node_->now();
    leader_message.name =
      shuffled_joint_names;
    leader_message.position =
      shuffled_leader_positions;

    sensor_msgs::msg::JointState follower_message;
    follower_message.header.stamp =
      mock_node_->now();
    follower_message.name =
      shuffled_joint_names;
    follower_message.position =
      shuffled_follower_positions;

    leader_publisher_->publish(
      leader_message);

    follower_publisher_->publish(
      follower_message);

    spinFor(10ms);
  }

  ASSERT_FALSE(
    received_commands_.empty());

  ASSERT_TRUE(
    guiding_enabled_);

  expectNear(
    received_commands_.back(),
    expected_command,
    0.02);
}

}  // namespace lerobot_teleop
