#include "cortexai_bringup/cortexai_manager.hpp"

namespace cortexai_bringup
{

CortexAIManager::CortexAIManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("cortexai_manager", options)
{
  enable_health_log_ = this->declare_parameter<bool>("enable_health_log", true);

  // 주기적으로 시스템/토픽 상태 점검 로직을 넣기 위한 타이머(지금은 로그만)
  timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&CortexAIManager::on_timer, this));

  RCLCPP_INFO(get_logger(), "[cortexai_manager] started (health_log=%s)",
              enable_health_log_ ? "true" : "false");
}

void CortexAIManager::on_timer()
{
  if (!enable_health_log_) return;

  // 향후 확장 포인트:
  // - required topics hz 체크 (left/right/depth/odom)
  // - TF guard (base_link <-> camera_link 존재 여부)
  // - Visual SLAM tracking 상태 감시
  // - nvblox map 업데이트 감시
  // - PX4 bridge 상태 감시
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                       "[cortexai_manager] alive. (extend here: health/tf/topic guards)");
}

}  // namespace cortexai_bringup

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cortexai_bringup::CortexAIManager)
