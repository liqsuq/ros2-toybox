#include "rclcpp/rclcpp.hpp"
#include "pingpong/ping_compo.hpp"

int main (int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto opts = rclcpp::NodeOptions();
  auto ping_pub_node = std::make_shared<pingpong::PingPubNode>(opts);
  executor.add_node(ping_pub_node);
  auto ping_sub_node = std::make_shared<pingpong::PingSubNode>(opts);
  executor.add_node(ping_sub_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
