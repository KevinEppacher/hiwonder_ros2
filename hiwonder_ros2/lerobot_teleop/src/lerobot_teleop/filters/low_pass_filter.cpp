#include "lerobot_teleop/filters/low_pass_filter.hpp"

#include <numbers>
#include <stdexcept>
#include <vector>

namespace lerobot_teleop
{

LowPassFilter::LowPassFilter(
  rclcpp_lifecycle::LifecycleNode & node,
  double update_rate)
: enabled_(
    node.declare_parameter<bool>(
      "low_pass_filter.enabled",
      true))
{
  const double cutoff_frequency =
    node.declare_parameter<double>(
    "low_pass_filter.cutoff_frequency",
    1.0);

  if (update_rate <= 0.0) {
    throw std::invalid_argument(
            "Update rate must be greater than zero");
  }

  if (cutoff_frequency <= 0.0) {
    throw std::invalid_argument(
            "Cutoff frequency must be greater than zero");
  }

  const double dt =
    1.0 / update_rate;

  const double rc =
    1.0 /
    (2.0 * std::numbers::pi * cutoff_frequency);

  alpha_ =
    dt / (rc + dt);
}

void LowPassFilter::reset(
  const std::vector<double> & values)
{
  state_ = values;
  initialized_ = true;
}

std::vector<double> LowPassFilter::filter(
  const std::vector<double> & input)
{
  if (!enabled_) {
    return input;
  }

  if (
    !initialized_ ||
    state_.size() != input.size())
  {
    reset(input);
  }

  for (std::size_t i = 0; i < input.size(); ++i) {
    state_[i] +=
      alpha_ *
      (input[i] - state_[i]);
  }

  return state_;
}

}  // namespace lerobot_teleop
