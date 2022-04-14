#include <functional>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "pingpong_interfaces/msg/ping.hpp"
#include "pingpong_interfaces/msg/pong.hpp"

class PongNode : public rclcpp::Node {
public:
  PongNode() : Node("pong") {
    using std::placeholders::_1;
    pub_ = this->create_publisher<pingpong_interfaces::msg::Pong>("pong", 10);
    sub_ = this->create_subscription<pingpong_interfaces::msg::Ping>(
      "ping", 10, std::bind(&PongNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const pingpong_interfaces::msg::Ping::SharedPtr msg) {
    clock_gettime(CLOCK_MONOTONIC, &time1_);
    msg_.t1_sec = (long)time1_.tv_sec;
    msg_.t1_nsec = (long)time1_.tv_nsec;

    msg_.t0_sec = msg->t0_sec;
    msg_.t0_nsec = msg->t0_nsec;

    clock_gettime(CLOCK_MONOTONIC, &time2_);
    msg_.t2_sec = (long)time2_.tv_sec;
    msg_.t2_nsec = (long)time2_.tv_nsec;

    pub_->publish(msg_);
  }
  rclcpp::Publisher<pingpong_interfaces::msg::Pong>::SharedPtr pub_;
  rclcpp::Subscription<pingpong_interfaces::msg::Ping>::SharedPtr sub_;
  pingpong_interfaces::msg::Pong msg_;
  struct timespec time1_ = {0, 0};
  struct timespec time2_ = {0, 0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PongNode>());
  rclcpp::shutdown();
  return 0;
}
