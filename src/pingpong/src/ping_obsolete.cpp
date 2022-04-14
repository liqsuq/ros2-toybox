#include <chrono>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "pingpong_interfaces/msg/ping.hpp"
#include "pingpong_interfaces/msg/pong.hpp"

rclcpp::Node::SharedPtr node = nullptr;
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

int main(int argc, char **argv) {
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("ping_node");

  // ping publisher
  auto publisher = node->
    create_publisher<pingpong_interfaces::msg::Ping>("ping", 10);
  pingpong_interfaces::msg::Ping message;
  rclcpp::WallRate rate(1s);

  // pong subscription
  auto subscription = node->
    create_subscription<pingpong_interfaces::msg::Pong>("pong", 10, callback);

  // time measurement test //
  //struct timespec reqtime = {0, 0};
  //rclcpp::Clock steady_clock_ros(RCL_STEADY_TIME);
  //
  //auto t0 = std::chrono::steady_clock::now();
  //clock_gettime(CLOCK_MONOTONIC, &reqtime);
  //auto now = steady_clock_ros.now();
  //
  //auto nanosec =
  //  std::chrono::duration_cast<nanoseconds>(t0.time_since_epoch());
  //RCLCPP_INFO(node->get_logger(), "t0:      %ld", nanosec.count());
  //RCLCPP_INFO(node->get_logger(), "reqtime: %ld", 1000000000 * reqtime.tv_sec + reqtime.tv_nsec); 
  //RCLCPP_INFO(node->get_logger(), "now:     %ld", now.nanoseconds()); 

  struct timespec time0 = {0, 0};
  while (rclcpp::ok()) {
    clock_gettime(CLOCK_MONOTONIC, &time0);
    message.t0_sec = (long)time0.tv_sec;
    message.t0_nsec = (long)time0.tv_nsec;
    publisher->publish(message);
    rclcpp::spin_some(node);
    rate.sleep();
  }
	  
  rclcpp::shutdown();
  return 0;
}