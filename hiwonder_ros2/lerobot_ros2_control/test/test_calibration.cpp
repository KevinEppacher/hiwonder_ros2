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

#include "lerobot_ros2_control/lerobot_system_hardware.hpp"

TEST(CalibrationTest, RawToPositionHandlesInvertedEndpoints)
{
  lerobot_ros2_control::JointCalibration joint;
  joint.lower_position = 3092;
  joint.upper_position = 388;
  joint.lower_limit = -1.91986;
  joint.upper_limit = 1.91986;

  EXPECT_NEAR(lerobot_ros2_control::rawToPosition(joint, 3092), -1.91986, 1e-6);
  EXPECT_NEAR(lerobot_ros2_control::rawToPosition(joint, 388), 1.91986, 1e-6);
  EXPECT_NEAR(lerobot_ros2_control::rawToPosition(joint, 1740), 0.0, 1e-3);
}

TEST(CalibrationTest, PositionToRawHandlesInvertedEndpoints)
{
  lerobot_ros2_control::JointCalibration joint;
  joint.lower_position = 3092;
  joint.upper_position = 388;
  joint.lower_limit = -1.91986;
  joint.upper_limit = 1.91986;

  EXPECT_EQ(lerobot_ros2_control::positionToRaw(joint, -1.91986), 3092U);
  EXPECT_EQ(lerobot_ros2_control::positionToRaw(joint, 1.91986), 388U);
  EXPECT_NEAR(lerobot_ros2_control::positionToRaw(joint, 0.0), 1740.0, 1.0);
}
