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
#include <thread>
#include <vector>

#include "common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_benchmark_msgs/msg/sample.hpp"
#include "rmw_iceoryx2_benchmark_nodes/options.hpp"
#include "rmw_iceoryx2_benchmark_nodes/stats.hpp"

using namespace std::chrono_literals;

namespace {

using Sample = rmw_iceoryx2_benchmark_msgs::msg::Sample;

constexpr auto DISCOVERY_TIMEOUT = 10s;

template <typename Query>
void time_query(benchmark::LatencyRecorder &recorder, Query &&query) {
  const auto start = benchmark::steady_time_nanos();
  query();
  recorder.record(benchmark::steady_time_nanos() - start);
}

} // namespace

int main(int argc, char *argv[]) {
  benchmark::exit_on_help(argc, argv,
                          "[--topics <n>] [--count <n>] [--warmup <n>]");
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);
  const auto topics = benchmark::parse_flag(argc, argv, "--topics", 100);

  auto other_context = std::make_shared<rclcpp::Context>();
  other_context->init(argc, argv);
  auto actor_options = benchmark::quiet_node_options();
  actor_options.context(other_context);
  auto actor = std::make_shared<rclcpp::Node>("graph_actor", actor_options);
  std::vector<rclcpp::Publisher<Sample>::SharedPtr> publishers;
  for (uint64_t i = 0; i < topics; ++i) {
    publishers.push_back(actor->create_publisher<Sample>(
        "graph_query_" + std::to_string(i), 10));
  }

  auto observer = std::make_shared<rclcpp::Node>(
      "graph_observer", benchmark::quiet_node_options());
  const std::string topic = "/graph_query_0";
  const auto discovery_deadline =
      std::chrono::steady_clock::now() + DISCOVERY_TIMEOUT;
  while (observer->get_topic_names_and_types().size() < topics &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }

  benchmark::LatencyRecorder topic_names(options.warmup);
  benchmark::LatencyRecorder node_names(options.warmup);
  benchmark::LatencyRecorder count_publishers(options.warmup);
  benchmark::LatencyRecorder publishers_info(options.warmup);
  for (uint64_t i = 0; i < options.count; ++i) {
    time_query(topic_names,
               [&] { (void)observer->get_topic_names_and_types(); });
    time_query(node_names, [&] { (void)observer->get_node_names(); });
    time_query(count_publishers,
               [&] { (void)observer->count_publishers(topic); });
    time_query(publishers_info,
               [&] { (void)observer->get_publishers_info_by_topic(topic); });
  }

  const auto suffix = " · " + std::to_string(topics) + " topics";
  std::cout << topic_names.report("get_topic_names_and_types" + suffix,
                                  options.count)
            << node_names.report("get_node_names" + suffix, options.count)
            << count_publishers.report("count_publishers" + suffix,
                                       options.count)
            << publishers_info.report("get_publishers_info_by_topic" + suffix,
                                      options.count)
            << std::endl;

  other_context->shutdown("benchmark done");
  rclcpp::shutdown();
  return 0;
}
