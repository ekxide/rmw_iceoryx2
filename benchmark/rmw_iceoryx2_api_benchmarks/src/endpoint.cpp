// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_benchmark_msgs/msg/sample.hpp"
#include "rmw_iceoryx2_benchmark_nodes/options.hpp"
#include "rmw_iceoryx2_benchmark_nodes/stats.hpp"

using Sample = rmw_iceoryx2_benchmark_msgs::msg::Sample;

int main(int argc, char *argv[]) {
  benchmark::exit_on_help(
      argc, argv, "[--endpoints <n>[,<n>...]] [--count <n>] [--warmup <n>]");
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);
  const auto levels = benchmark::parse_levels(argc, argv, "--endpoints", "50");

  auto node = std::make_shared<rclcpp::Node>("endpoint_benchmark",
                                             benchmark::quiet_node_options());
  benchmark::LatencyRecorder create_publisher(options.warmup);
  benchmark::LatencyRecorder destroy_publisher(options.warmup);
  benchmark::LatencyRecorder create_subscription(options.warmup);
  benchmark::LatencyRecorder destroy_subscription(options.warmup);
  for (uint64_t i = 0; i < options.count; ++i) {
    const auto topic = "endpoint_" + std::to_string(i);
    auto start = benchmark::steady_time_nanos();
    auto publisher = node->create_publisher<Sample>(topic, 10);
    auto created = benchmark::steady_time_nanos();
    publisher.reset();
    auto destroyed = benchmark::steady_time_nanos();
    create_publisher.record(created - start);
    destroy_publisher.record(destroyed - created);

    start = benchmark::steady_time_nanos();
    auto subscription =
        node->create_subscription<Sample>(topic, 10, [](Sample::UniquePtr) {});
    created = benchmark::steady_time_nanos();
    subscription.reset();
    destroyed = benchmark::steady_time_nanos();
    create_subscription.record(created - start);
    destroy_subscription.record(destroyed - created);
  }
  std::cout << create_publisher.report("create a publisher", options.count)
            << destroy_publisher.report("destroy a publisher", options.count)
            << create_subscription.report("create a subscription",
                                          options.count)
            << destroy_subscription.report("destroy a subscription",
                                           options.count);

  benchmark::report_footprint("publisher", levels, [&node](size_t i) {
    return node->create_publisher<Sample>("publisher_" + std::to_string(i), 10);
  });
  benchmark::report_footprint("subscription", levels, [&node](size_t i) {
    return node->create_subscription<Sample>(
        "subscription_" + std::to_string(i), 10, [](Sample::UniquePtr) {});
  });

  rclcpp::shutdown();
  return 0;
}
