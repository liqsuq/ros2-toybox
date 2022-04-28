#include <functional>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "pingpong/pong_compo.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"

#define S2NS 1000000000

pingpong::PongNode::PongNode(rclcpp::NodeOptions opts) : Node("pong", opts) {
  pub_ = this->create_publisher<pingpong::msg::Pong>("pong", 10);
  sub_ = this->create_subscription<pingpong::msg::Ping>("ping", 10,
  [this](pingpong::msg::Ping::SharedPtr msg) {
    clock_gettime(CLOCK_MONOTONIC, &time1_);
    msg_.t1_sec = (long)time1_.tv_sec;
    msg_.t1_nsec = (long)time1_.tv_nsec;

    msg_.t0_sec = msg->t0_sec;
    msg_.t0_nsec = msg->t0_nsec;
    #if 0
    RCLCPP_INFO(this->get_logger(),"t0: %ld",msg->t0_sec*S2NS + msg->t0_nsec);
    #endif

    clock_gettime(CLOCK_MONOTONIC, &time2_);
    msg_.t2_sec = (long)time2_.tv_sec;
    msg_.t2_nsec = (long)time2_.tv_nsec;

    pub_->publish(msg_);
  });
}

RCLCPP_COMPONENTS_REGISTER_NODE(pingpong::PongNode)
