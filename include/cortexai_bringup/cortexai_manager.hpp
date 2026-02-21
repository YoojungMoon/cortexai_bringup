#pragma once
#include <rclcpp/rclcpp.hpp>

namespace cortexai_bringup
{

class CortexAIManager : public rclcpp::Node
{
public:
  explicit CortexAIManager(const rclcpp::NodeOptions & options);

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  bool enable_health_log_{true};
};

}  // namespace cortexai_bringup
