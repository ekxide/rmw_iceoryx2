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

class BenchmarkPublisher : public rclcpp::Node {
public:
  BenchmarkPublisher(const benchmark::Options &options,
                     const rclcpp::NodeOptions &node_options)
      : Node("benchmark_publisher", node_options), m_options(options) {
    m_publisher = create_publisher<rmw_iceoryx2_benchmark_msgs::msg::Sample>(
        "benchmark", 10);

    const auto period = std::chrono::duration<double>(1.0 / m_options.rate);
    m_timer = create_wall_timer(period, [this]() {
      auto loan = m_publisher->borrow_loaned_message();
      loan.get().sequence = m_sequence;
      m_publisher->publish(std::move(loan));
      ++m_sequence;

      if (m_sequence == m_options.count) {
        m_timer->cancel();
        rclcpp::shutdown();
      }
    });
  }

private:
  benchmark::Options m_options;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<rmw_iceoryx2_benchmark_msgs::msg::Sample>::SharedPtr
      m_publisher;
  uint64_t m_sequence{0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);
  node_options.enable_logger_service(false);
  node_options.enable_rosout(false);

  std::cout << "publishing " << options.count << " samples at " << options.rate
            << " Hz" << std::endl;
  rclcpp::spin(std::make_shared<BenchmarkPublisher>(options, node_options));
  rclcpp::shutdown();
  return 0;
}
