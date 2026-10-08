#pragma once

#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lerobot_teleop
{

class LowPassFilter
{
public:
  LowPassFilter(
    rclcpp_lifecycle::LifecycleNode & node,
    double update_rate);

  void reset(
    const std::vector<double> & values);

  [[nodiscard]] std::vector<double> filter(
    const std::vector<double> & input);

private:
  bool enabled_;
  bool initialized_{false};

  double alpha_;

  std::vector<double> state_;
};

}  // namespace lerobot_teleop
