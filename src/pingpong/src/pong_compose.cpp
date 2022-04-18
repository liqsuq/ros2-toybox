#include "rclcpp/rclcpp.hpp"
#include "pingpong/pong_compo.hpp"

int main (int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto opts = rclcpp::NodeOptions();
  auto pong_node = std::make_shared<pingpong::PongNode>(opts);
  executor.add_node(pong_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
