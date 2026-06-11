// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_benchmark_msgs/msg/sample.hpp"
#include "rmw_iceoryx2_benchmark_nodes/options.hpp"
#include "rmw_iceoryx2_benchmark_nodes/stats.hpp"

using namespace std::chrono_literals;

namespace {

int64_t system_time_nanos() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

constexpr auto IDLE_TIMEOUT = 2s;

} // namespace

class BenchmarkSubscriber : public rclcpp::Node {
public:
  BenchmarkSubscriber(const benchmark::Options &options,
                      const rclcpp::NodeOptions &node_options)
      : Node("benchmark_subscriber", node_options), m_options(options),
        m_recorder(options.warmup) {
    auto on_msg =
        [this](rmw_iceoryx2_benchmark_msgs::msg::Sample::UniquePtr msg,
               const rclcpp::MessageInfo &info) {
          m_recorder.record(system_time_nanos() -
                            info.get_rmw_message_info().source_timestamp);
          m_last_receive = std::chrono::steady_clock::now();

          if (msg->sequence + 1 >= m_options.count) {
            finish();
          }
        };
    m_subscription =
        create_subscription<rmw_iceoryx2_benchmark_msgs::msg::Sample>(
            "benchmark", 10, on_msg);

    m_idle_timer = create_wall_timer(500ms, [this]() {
      if (m_recorder.received() == 0) {
        return; // run has not started yet
      }
      if (std::chrono::steady_clock::now() - m_last_receive > IDLE_TIMEOUT) {
        std::cerr << "no samples for 2s - reporting incomplete run"
                  << std::endl;
        finish();
      }
    });
  }

private:
  void finish() {
    std::cout << m_recorder.report("ros2 subscriber", m_options.count)
              << std::endl;
    rclcpp::shutdown();
  }

  benchmark::Options m_options;
  benchmark::LatencyRecorder m_recorder;
  std::chrono::steady_clock::time_point m_last_receive;
  rclcpp::TimerBase::SharedPtr m_idle_timer;
  rclcpp::Subscription<rmw_iceoryx2_benchmark_msgs::msg::Sample>::SharedPtr
      m_subscription;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);
  node_options.enable_logger_service(false);
  node_options.enable_rosout(false);

  std::cout << "expecting " << options.count << " samples" << std::endl;
  rclcpp::spin(std::make_shared<BenchmarkSubscriber>(options, node_options));
  rclcpp::shutdown();
  return 0;
}
