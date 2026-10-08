// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_benchmark_msgs/msg/sample.hpp"
#include "rmw_iceoryx2_benchmark_nodes/options.hpp"
#include "rmw_iceoryx2_benchmark_nodes/stats.hpp"

using Sample = rmw_iceoryx2_benchmark_msgs::msg::Sample;

int main(int argc, char *argv[]) {
  benchmark::exit_on_help(argc, argv,
                          "[--subscriptions <n>] [--count <n>] [--warmup <n>]");
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);
  const auto subscriptions =
      benchmark::parse_flag(argc, argv, "--subscriptions", 10);

  auto node = std::make_shared<rclcpp::Node>("idle_spin",
                                             benchmark::quiet_node_options());
  std::vector<rclcpp::Subscription<Sample>::SharedPtr> idle_subscriptions;
  for (uint64_t i = 0; i < subscriptions; ++i) {
    idle_subscriptions.push_back(node->create_subscription<Sample>(
        "idle_spin_" + std::to_string(i), 10, [](Sample::UniquePtr) {}));
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  benchmark::LatencyRecorder recorder(options.warmup);
  for (uint64_t i = 0; i < options.count; ++i) {
    const auto start = benchmark::steady_time_nanos();
    executor.spin_some(std::chrono::nanoseconds(0));
    recorder.record(benchmark::steady_time_nanos() - start);
  }

  std::cout << recorder.report("idle spin · " + std::to_string(subscriptions) +
                                   " subscriptions",
                               options.count)
            << std::endl;
  rclcpp::shutdown();
  return 0;
}
