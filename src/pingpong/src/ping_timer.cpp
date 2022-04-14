#include <chrono>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "pingpong_interfaces/msg/ping.hpp"
#include "pingpong_interfaces/msg/pong.hpp"

rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Publisher<pingpong_interfaces::msg::Ping>::SharedPtr
  publisher = nullptr;
pingpong_interfaces::msg::Ping message;

struct timespec time0 = {0, 0};
struct timespec time3 = {0, 0};

void callback(const pingpong_interfaces::msg::Pong::SharedPtr msg) {
  clock_gettime(CLOCK_MONOTONIC, &time3);
  RCLCPP_INFO(node->get_logger(), "t0: %ld",
    1000000000 * msg->t0_sec + msg->t0_nsec);
  RCLCPP_INFO(node->get_logger(), "t1: %ld",
    1000000000 * msg->t1_sec + msg->t1_nsec);
  RCLCPP_INFO(node->get_logger(), "t2: %ld",
    1000000000 * msg->t2_sec + msg->t2_nsec);
  RCLCPP_INFO(node->get_logger(), "t3: %ld",
    (long)(1000000000 * time3.tv_sec + time3.tv_nsec));
}

void publish(void) {
  clock_gettime(CLOCK_MONOTONIC, &time0);
  message.t0_sec = (long)time0.tv_sec;
  message.t0_nsec = (long)time0.tv_nsec;
  publisher->publish(message);
}

int main(int argc, char **argv) {
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("ping_node");

  // ping publisher
  publisher = node->
    create_publisher<pingpong_interfaces::msg::Ping>("ping", 10);

  // pong subscription
  auto subscription = node->
    create_subscription<pingpong_interfaces::msg::Pong>("pong", 10, callback);

  auto timer = node->create_wall_timer(1s, publish);
  
  rclcpp::spin(node);  
  rclcpp::shutdown();
  return 0;
}
