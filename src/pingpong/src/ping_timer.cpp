#include <chrono>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"

#define S2NS 1000000000

rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Publisher<pingpong::msg::Ping>::SharedPtr publisher = nullptr;
pingpong::msg::Ping message;

struct timespec time0;
struct timespec time3;

void callback(const pingpong::msg::Pong::SharedPtr msg) {
  clock_gettime(CLOCK_MONOTONIC, &time3);
  RCLCPP_INFO(node->get_logger(), "t0: %ld", msg->t0_sec*S2NS + msg->t0_nsec);
  RCLCPP_INFO(node->get_logger(), "t1: %ld", msg->t1_sec*S2NS + msg->t1_nsec);
  RCLCPP_INFO(node->get_logger(), "t2: %ld", msg->t2_sec*S2NS + msg->t2_nsec);
  RCLCPP_INFO(node->get_logger(), "t3: %ld",
    (long)(time3.tv_sec*S2NS + time3.tv_nsec));
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
    create_publisher<pingpong::msg::Ping>("ping", 10);

  // pong subscription
  auto subscription = node->
    create_subscription<pingpong::msg::Pong>("pong", 10, callback);

  auto timer = node->create_wall_timer(1s, publish);
  
  rclcpp::spin(node);  
  rclcpp::shutdown();
  return 0;
}
