#include <chrono>
#include <functional>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "pingpong/ping_compo.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"

#define S2NS 1000000000

pingpong::PingPubNode::PingPubNode(rclcpp::NodeOptions opts)
: Node("ping_pub", opts) {
  using namespace std::chrono_literals;
  pub_ = this->create_publisher<pingpong::msg::Ping>("ping", 10);
  timer_ = this->create_wall_timer(1s,
  [this] {
    clock_gettime(CLOCK_MONOTONIC, &time0_);
    msg_.t0_sec = (long)time0_.tv_sec;
    msg_.t0_nsec = (long)time0_.tv_nsec;
    pub_->publish(msg_);
  });
}

pingpong::PingSubNode::PingSubNode(rclcpp::NodeOptions opts)
: Node("ping_sub", opts) {
  using std::placeholders::_1;
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

RCLCPP_COMPONENTS_REGISTER_NODE(pingpong::PingPubNode)
RCLCPP_COMPONENTS_REGISTER_NODE(pingpong::PingSubNode)
