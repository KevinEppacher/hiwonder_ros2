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

}  // namespace lerobot_teleop
