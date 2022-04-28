#ifndef PINGPONG__PING_COMPO_HPP_
#define PINGPONG__PING_COMPO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"
#include "pingpong/visibility.h"

#define PINGPONG_COUNT 100

namespace pingpong {

class PingNode : public rclcpp::Node {
public:
  PINGPONG_PUBLIC PingNode(rclcpp::NodeOptions options);

private:
  int max_cnt_ = PINGPONG_COUNT;
  int send_cnt_ = 0;
  int recv_cnt_ = 0;
  std::array<long, PINGPONG_COUNT> t0;
  std::array<long, PINGPONG_COUNT> t1;
  std::array<long, PINGPONG_COUNT> t2;
  std::array<long, PINGPONG_COUNT> t3;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<pingpong::msg::Ping>::SharedPtr pub_;
  rclcpp::Subscription<pingpong::msg::Pong>::SharedPtr sub_;
  pingpong::msg::Ping msg_buf_;
  struct timespec time0_;
  struct timespec time3_;
};

} // namespace pingpong

#endif  // PINGPONG__PING_COMPO_HPP_
