#ifndef PINGPONG__PONG_COMPO_HPP_
#define PINGPONG__PONG_COMPO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"
#include "pingpong/visibility.h"

namespace pingpong {

class PongNode : public rclcpp::Node {
public:
  PINGPONG_PUBLIC PongNode(rclcpp::NodeOptions options);

private:
  void callback(const pingpong::msg::Ping::SharedPtr msg);
  rclcpp::Publisher<pingpong::msg::Pong>::SharedPtr pub_;
  rclcpp::Subscription<pingpong::msg::Ping>::SharedPtr sub_;
  pingpong::msg::Pong msg_;
  struct timespec time1_;
  struct timespec time2_;
};

} // namespace pingpong

#endif  // PINGPONG__PONG_COMPO_HPP_
