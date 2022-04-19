#include <functional>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "pingpong/pong_compo.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"

#define S2NS 1000000000

pingpong::PongNode::PongNode(rclcpp::NodeOptions opts) : Node("pong", opts) {
  using std::placeholders::_1;
  pub_ = this->create_publisher<pingpong::msg::Pong>("pong", 10);
  sub_ = this->create_subscription<pingpong::msg::Ping>(
    "ping", 10, std::bind(&pingpong::PongNode::callback, this, _1));
}

void pingpong::PongNode::callback(const pingpong::msg::Ping::SharedPtr msg) {
  clock_gettime(CLOCK_MONOTONIC, &time1_);
  msg_.t1_sec = (long)time1_.tv_sec;
  msg_.t1_nsec = (long)time1_.tv_nsec;

  msg_.t0_sec = msg->t0_sec;
  msg_.t0_nsec = msg->t0_nsec;
  RCLCPP_INFO(this->get_logger(), "t0: %ld", msg->t0_sec*S2NS + msg->t0_nsec);

  clock_gettime(CLOCK_MONOTONIC, &time2_);
  msg_.t2_sec = (long)time2_.tv_sec;
  msg_.t2_nsec = (long)time2_.tv_nsec;

  pub_->publish(msg_);
}

RCLCPP_COMPONENTS_REGISTER_NODE(pingpong::PongNode)

//int main(int argc, char **argv) {
//  rclcpp::init(argc, argv);
//  rclcpp::executors::SingleThreadedExecutor executor;
//  auto pong_node = std::make_shared<PongNode>();
//  executor.add_node(pong_node);
//  executor.spin();
//  rclcpp::shutdown();
//  return 0;
//}
