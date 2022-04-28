#include <chrono>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "pingpong/timer_compo.hpp"

pingpong::TimerNode::TimerNode(rclcpp::NodeOptions opts)
: Node("timer", opts) {
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(100us,
  [this] {
    struct timespec time_gettime;
    auto steady_clock = rclcpp::Clock(RCL_STEADY_TIME);

    auto time_chrono = std::chrono::steady_clock::now();
    clock_gettime(CLOCK_MONOTONIC_RAW, &time_gettime);
    auto time_ros = steady_clock.now();
    auto time_chrono_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      time_chrono.time_since_epoch());
    RCLCPP_INFO(this->get_logger(), "crn: %ld  cgt: %ld  ros: %ld",
      time_chrono_ns.count(),
      (1000000000 * time_gettime.tv_sec + time_gettime.tv_nsec),
      time_ros.nanoseconds());
  });
}

RCLCPP_COMPONENTS_REGISTER_NODE(pingpong::TimerNode)
