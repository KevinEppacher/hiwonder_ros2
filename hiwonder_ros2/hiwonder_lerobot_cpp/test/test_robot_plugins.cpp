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

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <type_traits>

#include <pluginlib/class_loader.hpp>

#include "hiwonder_lerobot_cpp/follower.hpp"
#include "hiwonder_lerobot_cpp/leader.hpp"
#include "lerobot_cpp/robot.hpp"

// Verify that Follower satisfies the generic Robot interface.
TEST(RobotInterfaceTest, FollowerImplementsRobotInterface)
{
  static_assert(
    std::is_base_of_v<lerobot::Robot, hiwonder::Follower>);

  static_assert(
    std::is_default_constructible_v<hiwonder::Follower>);
}

// Verify that Leader satisfies the generic Robot interface.
TEST(RobotInterfaceTest, LeaderImplementsRobotInterface)
{
  static_assert(
    std::is_base_of_v<lerobot::Robot, hiwonder::Leader>);

  static_assert(
    std::is_default_constructible_v<hiwonder::Leader>);
}

// Ensure Robot instances can be safely destroyed through the base class.
TEST(RobotInterfaceTest, RobotHasVirtualDestructor)
{
  static_assert(
    std::has_virtual_destructor_v<lerobot::Robot>);
}

// Verify that pluginlib can discover and instantiate the Follower plugin.
TEST(RobotPluginTest, LoadsFollowerPlugin)
{
  pluginlib::ClassLoader<lerobot::Robot> loader(
    "lerobot_cpp",
    "lerobot::Robot");

  auto robot =
    loader.createUniqueInstance(
      "hiwonder_lerobot_cpp/Follower");

  ASSERT_NE(robot, nullptr);
  EXPECT_EQ(robot->motorCount(), 6U);
  EXPECT_FALSE(robot->isConnected());
}

// Verify that pluginlib can discover and instantiate the Leader plugin.
TEST(RobotPluginTest, LoadsLeaderPlugin)
{
  pluginlib::ClassLoader<lerobot::Robot> loader(
    "lerobot_cpp",
    "lerobot::Robot");

  auto robot =
    loader.createUniqueInstance(
      "hiwonder_lerobot_cpp/Leader");

  ASSERT_NE(robot, nullptr);
  EXPECT_EQ(robot->motorCount(), 6U);
  EXPECT_FALSE(robot->isConnected());
}
