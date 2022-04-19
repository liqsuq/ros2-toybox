#ifndef PINGPONG__PING_COMPO_HPP_
#define PINGPONG__PING_COMPO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"
#include "pingpong/visibility.h"

namespace pingpong {

class PingPubNode : public rclcpp::Node {
public:
  PINGPONG_PUBLIC PingPubNode(rclcpp::NodeOptions options);

private:
  void callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<pingpong::msg::Ping>::SharedPtr pub_;
  pingpong::msg::Ping msg_;
  struct timespec time0_;
};

class PingSubNode : public rclcpp::Node {
public:
  PINGPONG_PUBLIC PingSubNode(rclcpp::NodeOptions options);

private:
  void callback(const pingpong::msg::Pong::SharedPtr msg);
  rclcpp::Subscription<pingpong::msg::Pong>::SharedPtr sub_;
  struct timespec time3_;
};

} // namespace pingpong

#endif  // PINGPONG__PING_COMPO_HPP_
