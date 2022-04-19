#include <chrono>
#include <functional>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "pingpong/ping_compo.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"

#define S2NS 1000000000

pingpong::PingNode::PingNode(rclcpp::NodeOptions opts) : Node("ping", opts) {
  using namespace std::chrono_literals;
  pub_ = this->create_publisher<pingpong::msg::Ping>("ping", 10);
  timer_ = this->create_wall_timer(1s,
  [this] {
    // if(i_++ < count_) {
      clock_gettime(CLOCK_MONOTONIC, &time0_);
      msg_buf_.t0_sec = (long)time0_.tv_sec;
      msg_buf_.t0_nsec = (long)time0_.tv_nsec;
      pub_->publish(msg_buf_);
    // } else {
    //   timer_->cancel();
    // }
  });
  sub_ = this->create_subscription<pingpong::msg::Pong>("pong", 10,
  [this](pingpong::msg::Pong::SharedPtr msg) {
    clock_gettime(CLOCK_MONOTONIC, &time3_);
    RCLCPP_INFO(this->get_logger(),
      "t0: %ld", msg->t0_sec*S2NS + msg->t0_nsec);
    RCLCPP_INFO(this->get_logger(),
      "t1: %ld", msg->t1_sec*S2NS + msg->t1_nsec);
    RCLCPP_INFO(this->get_logger(),
      "t2: %ld", msg->t2_sec*S2NS + msg->t2_nsec);
    RCLCPP_INFO(this->get_logger(),
      "t3: %ld", (long)(time3_.tv_sec*S2NS + time3_.tv_nsec));
  });
}

RCLCPP_COMPONENTS_REGISTER_NODE(pingpong::PingNode)
