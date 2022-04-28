#include <iostream>
#include <chrono>
#include <functional>
#include <csignal>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "pingpong/ping_compo.hpp"
#include "pingpong/msg/ping.hpp"
#include "pingpong/msg/pong.hpp"

#define S2NS 1000000000

pingpong::PingNode::PingNode(rclcpp::NodeOptions opts) : Node("ping", opts) {
  using namespace std::chrono_literals;
  pub_ = this->create_publisher<pingpong::msg::Ping>("ping", 10);
  timer_ = this->create_wall_timer(10ms,
  [this] {
    if(send_cnt_++ < max_cnt_) {
      clock_gettime(CLOCK_MONOTONIC, &time0_);
      msg_buf_.t0_sec = (long)time0_.tv_sec;
      msg_buf_.t0_nsec = (long)time0_.tv_nsec;
      pub_->publish(msg_buf_);
    } else {
      timer_->cancel();
    }
  });
  sub_ = this->create_subscription<pingpong::msg::Pong>("pong", 10,
  [this](pingpong::msg::Pong::SharedPtr msg) {
    clock_gettime(CLOCK_MONOTONIC, &time3_);
    #if 0
    RCLCPP_INFO(this->get_logger(),
      "t0: %ld", msg->t0_sec*S2NS + msg->t0_nsec);
    RCLCPP_INFO(this->get_logger(),
      "t1: %ld", msg->t1_sec*S2NS + msg->t1_nsec);
    RCLCPP_INFO(this->get_logger(),
      "t2: %ld", msg->t2_sec*S2NS + msg->t2_nsec);
    RCLCPP_INFO(this->get_logger(),
      "t3: %ld", (long)(time3_.tv_sec*S2NS + time3_.tv_nsec));
    #endif
    t0[recv_cnt_] = msg->t0_sec*S2NS + msg->t0_nsec;
    t1[recv_cnt_] = msg->t1_sec*S2NS + msg->t1_nsec;
    t2[recv_cnt_] = msg->t2_sec*S2NS + msg->t2_nsec;
    t3[recv_cnt_] = (long)(time3_.tv_sec*S2NS + time3_.tv_nsec);
    if(++recv_cnt_ >= max_cnt_) {
      std::array<long, PINGPONG_COUNT> diff_t3t0;
      std::array<long, PINGPONG_COUNT> diff_t2t1;
      for(int i=0; i < PINGPONG_COUNT; i++) {
        diff_t2t1[i] = (t2[i] - t1[i]) / 1000;
        diff_t3t0[i] = (t3[i] - t0[i]) / 1000;
      }

      #if 0
      for(int i=0; i < PINGPONG_COUNT; i++) {
        std::cout << "[" << i << "]"
                  << "  t2-t1: " << diff_t2t1[i]
                  << "  t3-t0: " << diff_t3t0[i] << std::endl;
      }
      #endif

      long min = 1000000000000, max = 0, avr = 0;
      for(int i=0; i < PINGPONG_COUNT; i++) {
      avr += diff_t2t1[i];
        if(min > diff_t2t1[i]) {
          min = diff_t2t1[i];
        }
        if(max < diff_t2t1[i]) {
          max = diff_t2t1[i];
        }
      }
      avr /= PINGPONG_COUNT;
      std::cout << "Node  latency (us): min " << min
                << " avr " << avr << " max " << max << std::endl;

      min = 1000000000000, max = 0, avr = 0;
      for(int i=0; i < PINGPONG_COUNT; i++) {
        avr += (diff_t3t0[i] - diff_t2t1[i]) / 2;
        if(min > (diff_t3t0[i] - diff_t2t1[i]) / 2) {
          min = (diff_t3t0[i] - diff_t2t1[i]) / 2;
        }
        if(max < (diff_t3t0[i] - diff_t2t1[i]) / 2) {
          max = (diff_t3t0[i] - diff_t2t1[i]) / 2;
        }
      }
      avr /= PINGPONG_COUNT;
      std::cout << "Topic latency (us): min " << min
                << " avr " << avr << " max " << max << std::endl;

      min = 1000000000000, max = 0, avr = 0;
      for(int i=0; i < PINGPONG_COUNT; i++) {
        avr += diff_t3t0[i];
        if(min > diff_t3t0[i]) {
          min = diff_t3t0[i];
        }
        if(max < diff_t3t0[i]) {
          max = diff_t3t0[i];
        }
      }
      avr /= PINGPONG_COUNT;
      std::cout << "Total latency (us): min " << min
                << " avr " << avr << " max " << max << std::endl;

      raise(SIGINT);
    }
  });
}

RCLCPP_COMPONENTS_REGISTER_NODE(pingpong::PingNode)
