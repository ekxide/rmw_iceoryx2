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
#include "rmw_iceoryx2_benchmark_nodes/options.hpp"
#include "rmw_iceoryx2_benchmark_nodes/stats.hpp"

int main(int argc, char *argv[]) {
  benchmark::exit_on_help(
      argc, argv, "[--nodes <n>[,<n>...]] [--count <n>] [--warmup <n>]");
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);
  const auto levels = benchmark::parse_levels(argc, argv, "--nodes", "10");

  benchmark::LatencyRecorder create(options.warmup);
  benchmark::LatencyRecorder destroy(options.warmup);
  for (uint64_t i = 0; i < options.count; ++i) {
    const auto start = benchmark::steady_time_nanos();
    auto node = std::make_shared<rclcpp::Node>("benchmark_node");
    const auto created = benchmark::steady_time_nanos();
    node.reset();
    const auto destroyed = benchmark::steady_time_nanos();
    create.record(created - start);
    destroy.record(destroyed - created);
  }
  std::cout << create.report("create a node", options.count)
            << destroy.report("destroy a node", options.count);

  benchmark::report_footprint("node", levels, [](size_t i) {
    return std::make_shared<rclcpp::Node>("benchmark_node_" +
                                          std::to_string(i));
  });

  rclcpp::shutdown();
  return 0;
}
