// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_ICEORYX2_API_BENCHMARKS_COMMON_HPP
#define RMW_ICEORYX2_API_BENCHMARKS_COMMON_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace benchmark {

inline void exit_on_help(int argc, char *argv[], const std::string &flags) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "usage: " << argv[0] << ' ' << flags << '\n';
      std::exit(0);
    }
  }
}

inline uint64_t parse_flag(int argc, char *argv[], const std::string &flag,
                           uint64_t fallback) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == flag) {
      return std::stoull(argv[i + 1]);
    }
  }
  return fallback;
}

inline std::vector<uint64_t> parse_levels(int argc, char *argv[],
                                          const std::string &flag,
                                          const std::string &fallback) {
  std::string value = fallback;
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == flag) {
      value = argv[i + 1];
    }
  }
  std::vector<uint64_t> levels;
  std::stringstream stream(value);
  std::string level;
  while (std::getline(stream, level, ',')) {
    levels.push_back(std::stoull(level));
  }
  std::sort(levels.begin(), levels.end());
  return levels;
}

inline rclcpp::NodeOptions quiet_node_options() {
  rclcpp::NodeOptions node_options;
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);
  node_options.enable_logger_service(false);
  node_options.enable_rosout(false);
  return node_options;
}

inline int64_t steady_time_nanos() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

inline bool can_count_file_descriptors() {
  return std::filesystem::exists("/proc/self/fd");
}

inline size_t open_file_descriptors() {
  return static_cast<size_t>(
      std::distance(std::filesystem::directory_iterator("/proc/self/fd"),
                    std::filesystem::directory_iterator{}));
}

inline uint64_t private_memory_kilobytes() {
  std::ifstream smaps("/proc/self/smaps_rollup");
  uint64_t total = 0;
  std::string key;
  while (smaps >> key) {
    if (key == "Private_Clean:" || key == "Private_Dirty:") {
      uint64_t kilobytes = 0;
      smaps >> kilobytes;
      total += kilobytes;
    }
    smaps.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return total;
}

inline bool can_measure_footprint() {
  return can_count_file_descriptors() &&
         std::filesystem::exists("/proc/self/smaps_rollup");
}

template <typename Create>
void report_footprint(const std::string &noun,
                      const std::vector<uint64_t> &levels, Create create) {
  if (!can_measure_footprint()) {
    return;
  }
  const auto memory_before = private_memory_kilobytes();
  const auto file_descriptors_before = open_file_descriptors();
  std::vector<std::shared_ptr<void>> alive;
  for (const auto level : levels) {
    while (alive.size() < level) {
      try {
        alive.push_back(create(alive.size()));
      } catch (const std::exception &error) {
        std::cout << '\n'
                  << noun << ' ' << alive.size() + 1
                  << " failed: " << error.what() << '\n';
        break;
      }
    }
    if (alive.empty()) {
      return;
    }
    const auto created = static_cast<double>(alive.size());
    std::cout << '\n'
              << noun << "s alive: " << alive.size() << '/' << level
              << "\nprivate memory per " << noun << ": "
              << static_cast<double>(private_memory_kilobytes() -
                                     memory_before) /
                     1024.0 / created
              << " MB\nfile descriptors per " << noun << ": "
              << static_cast<double>(open_file_descriptors() -
                                     file_descriptors_before) /
                     created
              << std::endl;
    if (alive.size() < level) {
      return;
    }
  }
}

} // namespace benchmark

#endif // RMW_ICEORYX2_API_BENCHMARKS_COMMON_HPP
