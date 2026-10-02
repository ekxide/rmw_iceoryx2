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

#include "common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_benchmark_msgs/msg/sample.hpp"
#include "rmw_iceoryx2_benchmark_nodes/options.hpp"
#include "rmw_iceoryx2_benchmark_nodes/stats.hpp"

using namespace std::chrono_literals;

namespace {

using Sample = rmw_iceoryx2_benchmark_msgs::msg::Sample;

constexpr auto GRAPH_CHANGE_TIMEOUT = 1s;
constexpr uint64_t MAX_MISSED_IN_A_ROW = 3;

} // namespace

int main(int argc, char *argv[]) {
  benchmark::exit_on_help(argc, argv, "[--count <n>] [--warmup <n>]");
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);

  auto other_context = std::make_shared<rclcpp::Context>();
  other_context->init(argc, argv);

  auto observer = std::make_shared<rclcpp::Node>(
      "graph_observer", benchmark::quiet_node_options());
  auto actor_options = benchmark::quiet_node_options();
  actor_options.context(other_context);
  auto actor = std::make_shared<rclcpp::Node>("graph_actor", actor_options);

  auto graph_event = observer->get_graph_event();
  uint64_t missed_in_a_row = 0;
  auto wait_for_change = [&](benchmark::LatencyRecorder &recorder,
                             int64_t start) {
    observer->wait_for_graph_change(graph_event, GRAPH_CHANGE_TIMEOUT);
    if (!graph_event->check_and_clear()) {
      ++missed_in_a_row;
      return;
    }
    missed_in_a_row = 0;
    recorder.record(benchmark::steady_time_nanos() - start);
  };

  benchmark::LatencyRecorder created(options.warmup);
  benchmark::LatencyRecorder destroyed(options.warmup);
  uint64_t creations = 0;
  uint64_t destructions = 0;
  while (creations < options.count && missed_in_a_row < MAX_MISSED_IN_A_ROW) {
    ++creations;
    (void)graph_event->check_and_clear();
    auto start = benchmark::steady_time_nanos();
    auto publisher = actor->create_publisher<Sample>("graph_change", 10);
    wait_for_change(created, start);
    if (missed_in_a_row >= MAX_MISSED_IN_A_ROW) {
      break;
    }

    ++destructions;
    (void)graph_event->check_and_clear();
    start = benchmark::steady_time_nanos();
    publisher.reset();
    wait_for_change(destroyed, start);
  }

  if (missed_in_a_row >= MAX_MISSED_IN_A_ROW) {
    std::cout << "\nno graph change seen within 1 s for " << missed_in_a_row
              << " changes in a row, stopping\n";
  }
  std::cout << created.report("graph change seen after another context "
                              "starts creating a publisher",
                              creations)
            << destroyed.report("graph change seen after another context "
                                "starts destroying a publisher",
                                destructions)
            << std::endl;

  other_context->shutdown("benchmark done");
  rclcpp::shutdown();
  return 0;
}
