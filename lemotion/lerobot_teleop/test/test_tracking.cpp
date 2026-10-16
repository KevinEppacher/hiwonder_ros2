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

#include "leader_follower_teleop_test_fixture.hpp"
namespace lerobot_teleop
{

using namespace std::chrono_literals;

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
      }

      if (
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

}  // namespace lerobot_teleop
