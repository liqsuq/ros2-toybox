#include "rclcpp/rclcpp.hpp"

class NotopicNode: public rclcpp::Node {
public:
    NotopicNode() : Node("Node") {
    ;
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<NotopicNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
