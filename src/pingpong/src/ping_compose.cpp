#include "rclcpp/rclcpp.hpp"
#include "pingpong/ping_compo.hpp"

int main (int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto opts = rclcpp::NodeOptions();
  auto ping_node = std::make_shared<pingpong::PingNode>(opts);
  executor.add_node(ping_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
