#ifndef PINGPONG__TIMER_COMPO_HPP_
#define PINGPONG__TIMER_COMPO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pingpong/visibility.h"

namespace pingpong {

class TimerNode : public rclcpp::Node {
public:
  PINGPONG_PUBLIC TimerNode(rclcpp::NodeOptions options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace pingpong

#endif  // PINGPONG__TIMER_COMPO_HPP_
