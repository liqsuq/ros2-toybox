#include "rclcpp/rclcpp.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"

#define S2NS 1000000000

rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Publisher<pingpong::msg::Pong>::SharedPtr publisher = nullptr;
pingpong::msg::Pong message;

struct timespec time1;
struct timespec time2;

void callback(const pingpong::msg::Ping::SharedPtr msg) {
  clock_gettime(CLOCK_MONOTONIC, &time1);
  message.t1_sec = (long)time1.tv_sec;
  message.t1_nsec = (long)time1.tv_nsec;

  message.t0_sec = msg->t0_sec;
  message.t0_nsec = msg->t0_nsec;
  RCLCPP_INFO(node->get_logger(), "t0: %ld", msg->t0_sec*S2NS + msg->t0_nsec);

  clock_gettime(CLOCK_MONOTONIC, &time2);
  message.t2_sec = (long)time2.tv_sec;
  message.t2_nsec = (long)time2.tv_nsec;
 
  publisher->publish(message);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("pong_node");

  // pong publisher
  publisher = node->
    create_publisher<pingpong::msg::Pong>("pong", 10);

  // ping subscriber
  auto subscription = node->
    create_subscription<pingpong::msg::Ping>("ping", 10, callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
