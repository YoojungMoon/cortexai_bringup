#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TFSanity : public rclcpp::Node
{
public:
  TFSanity() : Node("enyoai_tf_sanity"), buffer_(this->get_clock()), listener_(buffer_)
  {
    this->declare_parameter<std::string>("tf_parent", "odom");
    this->declare_parameter<std::string>("tf_child", "camera0_link");
    this->declare_parameter<double>("check_hz", 2.0);

    parent_ = this->get_parameter("tf_parent").as_string();
    child_ = this->get_parameter("tf_child").as_string();
    check_hz_ = this->get_parameter("check_hz").as_double();

    const double period = 1.0 / std::max(0.1, check_hz_);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period), [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "TF sanity: %s -> %s", parent_.c_str(), child_.c_str());
  }

private:
  void tick()
  {
    try {
      if (!buffer_.canTransform(parent_, child_, tf2::TimePointZero, tf2::durationFromSec(0.1))) {
        RCLCPP_WARN(get_logger(), "Missing TF %s -> %s", parent_.c_str(), child_.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "TF error %s -> %s: %s", parent_.c_str(), child_.c_str(), e.what());
    }
  }

  std::string parent_;
  std::string child_;
  double check_hz_{2.0};
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFSanity>());
  rclcpp::shutdown();
  return 0;
}
