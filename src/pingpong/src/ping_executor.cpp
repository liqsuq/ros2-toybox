#include <chrono>
#include <functional>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"

#define S2NS 1000000000

class PingPubNode : public rclcpp::Node {
public:
  PingPubNode() : Node("ping_pub") {
    using namespace std::chrono_literals;
    pub_ = this->create_publisher<pingpong::msg::Ping>("ping", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&PingPubNode::callback, this));
  }
private:
  void callback() {
    clock_gettime(CLOCK_MONOTONIC, &time0_);
    msg_.t0_sec = (long)time0_.tv_sec;
    msg_.t0_nsec = (long)time0_.tv_nsec;
    pub_->publish(msg_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<pingpong::msg::Ping>::SharedPtr pub_;
  pingpong::msg::Ping msg_;
  struct timespec time0_;
};

class PingSubNode : public rclcpp::Node {
public:
  PingSubNode() : Node("ping_sub") {
    using std::placeholders::_1;
    sub_ = this->create_subscription<pingpong::msg::Pong>(
      "pong", 10, std::bind(&PingSubNode::callback, this, _1));
  }
private:
  void callback(const pingpong::msg::Pong::SharedPtr msg) {
    clock_gettime(CLOCK_MONOTONIC, &time3_);
    RCLCPP_INFO(this->get_logger(), "t0: %ld", msg->t0_sec*S2NS + msg->t0_nsec);
    RCLCPP_INFO(this->get_logger(), "t1: %ld", msg->t1_sec*S2NS + msg->t1_nsec);
    RCLCPP_INFO(this->get_logger(), "t2: %ld", msg->t2_sec*S2NS + msg->t2_nsec);
    RCLCPP_INFO(this->get_logger(), "t3: %ld",
      (long)(time3_.tv_sec*S2NS + time3_.tv_nsec));
  }
  rclcpp::Subscription<pingpong::msg::Pong>::SharedPtr sub_;
  struct timespec time3_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto ping_pub_node = std::make_shared<PingPubNode>();
  executor.add_node(ping_pub_node);
  auto ping_sub_node = std::make_shared<PingSubNode>();
  executor.add_node(ping_sub_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
