// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_benchmark_nodes/options.hpp"
#include "rmw_iceoryx2_benchmark_nodes/stats.hpp"

namespace {

constexpr size_t FILE_DESCRIPTOR_SAMPLE = 100;

bool wait_until_ready(rclcpp::WaitSet &wait_set) {
  return wait_set.wait().kind() == rclcpp::WaitResultKind::Ready;
}

} // namespace

int main(int argc, char *argv[]) {
  benchmark::exit_on_help(argc, argv, "[--count <n>] [--warmup <n>]");
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);

  benchmark::LatencyRecorder create(options.warmup);
  benchmark::LatencyRecorder trigger(options.warmup);
  benchmark::LatencyRecorder destroy(options.warmup);
  for (uint64_t i = 0; i < options.count; ++i) {
    const auto start = benchmark::steady_time_nanos();
    auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
    const auto created = benchmark::steady_time_nanos();
    guard_condition->trigger();
    const auto triggered = benchmark::steady_time_nanos();
    guard_condition.reset();
    const auto destroyed = benchmark::steady_time_nanos();
    create.record(created - start);
    trigger.record(triggered - created);
    destroy.record(destroyed - triggered);
  }

  auto wake = std::make_shared<rclcpp::GuardCondition>();
  auto acknowledge = std::make_shared<rclcpp::GuardCondition>();
  rclcpp::WaitSet wake_wait_set;
  wake_wait_set.add_guard_condition(wake);
  rclcpp::WaitSet acknowledge_wait_set;
  acknowledge_wait_set.add_guard_condition(acknowledge);

  std::atomic<int64_t> triggered_at{0};
  benchmark::LatencyRecorder wake_up(options.warmup);
  std::thread waiter([&] {
    for (uint64_t i = 0; i < options.count; ++i) {
      if (!wait_until_ready(wake_wait_set)) {
        return;
      }
      wake_up.record(benchmark::steady_time_nanos() - triggered_at.load());
      acknowledge->trigger();
    }
  });
  for (uint64_t i = 0; i < options.count; ++i) {
    triggered_at.store(benchmark::steady_time_nanos());
    wake->trigger();
    if (!wait_until_ready(acknowledge_wait_set)) {
      break;
    }
  }
  waiter.join();

  std::cout << create.report("create a guard condition", options.count)
            << trigger.report("trigger a guard condition", options.count)
            << destroy.report("destroy a guard condition", options.count)
            << wake_up.report("wake up a thread waiting on a guard condition",
                              options.count);

  if (benchmark::can_count_file_descriptors()) {
    const auto before = benchmark::open_file_descriptors();
    std::vector<std::shared_ptr<rclcpp::GuardCondition>> guard_conditions;
    for (size_t i = 0; i < FILE_DESCRIPTOR_SAMPLE; ++i) {
      guard_conditions.push_back(std::make_shared<rclcpp::GuardCondition>());
    }
    std::cout << "\nfile descriptors per guard condition: "
              << static_cast<double>(benchmark::open_file_descriptors() -
                                     before) /
                     FILE_DESCRIPTOR_SAMPLE
              << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
