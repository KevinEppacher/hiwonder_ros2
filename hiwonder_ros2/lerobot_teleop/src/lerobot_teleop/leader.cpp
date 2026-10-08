#include "lerobot_teleop/leader.hpp"

#include <memory>
#include <utility>

namespace lerobot_teleop
{

Leader::Leader(
  rclcpp_lifecycle::LifecycleNode & node,
  const LeaderParameters & parameters)
: Robot(
    node,
    parameters.joint_states_topic),
  timeout_(parameters.timeout),
  logger_(node.get_logger())
{
  guiding_mode_client_ =
    node.create_client<std_srvs::srv::SetBool>(
    parameters.guiding_mode_service);
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
      logger_,
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
