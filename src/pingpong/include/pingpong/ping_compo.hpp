#ifndef PINGPONG__PING_COMPO_HPP_
#define PINGPONG__PING_COMPO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"
#include "pingpong/visibility.h"

#define PINGPONG_COUNT 10

namespace pingpong {

class PingNode : public rclcpp::Node {
public:
  PINGPONG_PUBLIC PingNode(rclcpp::NodeOptions options);

private:
  int count_ = PINGPONG_COUNT;
  int i_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<pingpong::msg::Ping>::SharedPtr pub_;
  rclcpp::Subscription<pingpong::msg::Pong>::SharedPtr sub_;
  pingpong::msg::Ping msg_buf_;
  struct timespec time0_;
  struct timespec time3_;
};

class PingPubNode : public rclcpp::Node {
public:
  PINGPONG_PUBLIC PingPubNode(rclcpp::NodeOptions options);

private:
  int count_ = PINGPONG_COUNT;
  int i_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<pingpong::msg::Ping>::SharedPtr pub_;
  pingpong::msg::Ping msg_;
  struct timespec time0_;
};

class PingSubNode : public rclcpp::Node {
public:
  PINGPONG_PUBLIC PingSubNode(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<pingpong::msg::Pong>::SharedPtr sub_;
  struct timespec time3_;
};

} // namespace pingpong

#endif  // PINGPONG__PING_COMPO_HPP_
