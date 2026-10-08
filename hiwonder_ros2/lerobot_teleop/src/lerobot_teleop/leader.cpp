#include "lerobot_teleop/leader.hpp"

#include <memory>
#include <string>
#include <utility>

namespace lerobot_teleop
{

Leader::Leader(
  rclcpp_lifecycle::LifecycleNode & node)
: Robot(
    node,
    node.declare_parameter<std::string>(
      "leader.joint_state_topic",
      "/leader/joint_states")),
  timeout_(
    node.declare_parameter<double>(
      "leader.timeout",
      0.2))
{
  const auto guiding_mode_service =
    node_.declare_parameter<std::string>(
    "leader.guiding_mode_service",
    "/leader/leader_so101/guiding_mode");

  guiding_mode_client_ =
    node_.create_client<std_srvs::srv::SetBool>(
    guiding_mode_service);
}

bool Leader::timedOut() const
{
  return stateAge() > timeout_;
}

bool Leader::setGuidingMode(
  bool enabled,
  GuidingCallback callback)
{
  if (!guiding_mode_client_->service_is_ready()) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "Guiding mode service is not available");

    return false;
  }

  auto request =
    std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = enabled;

  guiding_mode_client_->async_send_request(
    request,
    [callback = std::move(callback)](
      rclcpp::Client<
        std_srvs::srv::SetBool>::SharedFuture future)
    {
      callback(future.get()->success);
    });

  return true;
}

}  // namespace lerobot_teleop
