#pragma once

#include <functional>
#include <string>

#include "lerobot_teleop/robot.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace lerobot_teleop
{

struct LeaderParameters
{
  std::string joint_states_topic;
  std::string guiding_mode_service;
  double timeout;
};

class Leader : public Robot
{
public:
  using GuidingCallback =
    std::function<void(bool)>;

  Leader(
    rclcpp_lifecycle::LifecycleNode & node,
    const LeaderParameters & parameters);

  [[nodiscard]] bool timedOut() const;

  bool setGuidingMode(
    bool enabled,
    GuidingCallback callback);

private:
  double timeout_;

  rclcpp::Logger logger_;

  rclcpp::Client<
    std_srvs::srv::SetBool>::SharedPtr guiding_mode_client_;
};

}  // namespace lerobot_teleop
