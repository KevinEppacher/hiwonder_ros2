#pragma once

#include <functional>

#include "lerobot_teleop/robot.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace lerobot_teleop
{

class Leader : public Robot
{
public:
  using GuidingCallback =
    std::function<void(bool)>;

  explicit Leader(
    rclcpp_lifecycle::LifecycleNode & node);

  [[nodiscard]] bool timedOut() const;

  bool setGuidingMode(
    bool enabled,
    GuidingCallback callback);

private:
  double timeout_;

  rclcpp::Client<
    std_srvs::srv::SetBool>::SharedPtr guiding_mode_client_;
};

}  // namespace lerobot_teleop
