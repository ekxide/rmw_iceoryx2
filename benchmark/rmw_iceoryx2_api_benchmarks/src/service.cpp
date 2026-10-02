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
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_benchmark_msgs/srv/echo.hpp"
#include "rmw_iceoryx2_benchmark_nodes/options.hpp"
#include "rmw_iceoryx2_benchmark_nodes/stats.hpp"

using namespace std::chrono_literals;

namespace {

using Echo = rmw_iceoryx2_benchmark_msgs::srv::Echo;

constexpr auto RESPONSE_TIMEOUT = 1s;
constexpr auto DISCOVERY_TIMEOUT = 10s;

void echo(const std::shared_ptr<Echo::Request> request,
          std::shared_ptr<Echo::Response> response) {
  response->sequence = request->sequence;
}

void measure_requests(const benchmark::Options &options) {
  auto client_node = std::make_shared<rclcpp::Node>(
      "echo_client", benchmark::quiet_node_options());
  auto client = client_node->create_client<Echo>("echo");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);

  benchmark::LatencyRecorder round_trip(options.warmup);
  uint64_t answered = 0;
  if (client->wait_for_service(DISCOVERY_TIMEOUT)) {
    for (uint64_t i = 0; i < options.count; ++i) {
      auto request = std::make_shared<Echo::Request>();
      request->sequence = i;
      const auto start = benchmark::steady_time_nanos();
      auto response = client->async_send_request(request);
      if (client_executor.spin_until_future_complete(response,
                                                     RESPONSE_TIMEOUT) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        client->remove_pending_request(response);
        continue;
      }
      round_trip.record(benchmark::steady_time_nanos() - start);
      ++answered;
    }
  } else {
    std::cout << "\nthe service was not discovered within 10 s\n";
  }
  std::cout << round_trip.report("service round trip", options.count);
  if (answered < options.count) {
    std::cout << "\n"
              << options.count - answered
              << " requests got no response within 1 s\n";
  }
}

void measure_round_trip(int argc, char *argv[],
                        const benchmark::Options &options) {
  auto server_context = std::make_shared<rclcpp::Context>();
  server_context->init(argc, argv);
  auto server_options = benchmark::quiet_node_options();
  server_options.context(server_context);
  auto server_node =
      std::make_shared<rclcpp::Node>("echo_server", server_options);
  auto server = server_node->create_service<Echo>("echo", echo);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = server_context;
  rclcpp::executors::SingleThreadedExecutor server_executor(executor_options);
  server_executor.add_node(server_node);
  std::thread server_thread([&server_executor] { server_executor.spin(); });
  try {
    measure_requests(options);
  } catch (const std::exception &error) {
    std::cout << "\nservice round trip failed: " << error.what() << std::endl;
  }
  server_executor.cancel();
  server_thread.join();
  server_context->shutdown("benchmark done");
}

} // namespace

int main(int argc, char *argv[]) {
  benchmark::exit_on_help(
      argc, argv, "[--services <n>[,<n>...]] [--count <n>] [--warmup <n>]");
  rclcpp::init(argc, argv);
  const auto options = benchmark::parse(argc, argv);
  const auto levels = benchmark::parse_levels(argc, argv, "--services", "10");

  auto node = std::make_shared<rclcpp::Node>("service_benchmark",
                                             benchmark::quiet_node_options());
  benchmark::LatencyRecorder create(options.warmup);
  benchmark::LatencyRecorder destroy(options.warmup);
  try {
    for (uint64_t i = 0; i < options.count; ++i) {
      const auto start = benchmark::steady_time_nanos();
      auto service =
          node->create_service<Echo>("echo_" + std::to_string(i), echo);
      const auto created = benchmark::steady_time_nanos();
      service.reset();
      const auto destroyed = benchmark::steady_time_nanos();
      create.record(created - start);
      destroy.record(destroyed - created);
    }
  } catch (const std::exception &error) {
    std::cout << "creating a service failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 0;
  }
  std::cout << create.report("create a service", options.count)
            << destroy.report("destroy a service", options.count);

  measure_round_trip(argc, argv, options);

  benchmark::report_footprint("service", levels, [&node](size_t i) {
    return node->create_service<Echo>("footprint_" + std::to_string(i), echo);
  });

  rclcpp::shutdown();
  return 0;
}
