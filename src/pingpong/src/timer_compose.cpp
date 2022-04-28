#include "rclcpp/rclcpp.hpp"
#include "pingpong/timer_compo.hpp"

int main (int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto opts = rclcpp::NodeOptions();
  auto timer_node = std::make_shared<pingpong::TimerNode>(opts);
  executor.add_node(timer_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
