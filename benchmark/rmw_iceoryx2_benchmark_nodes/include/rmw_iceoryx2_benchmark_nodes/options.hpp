// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_ICEORYX2_BENCHMARK_NODES_OPTIONS_HPP
#define RMW_ICEORYX2_BENCHMARK_NODES_OPTIONS_HPP

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

namespace benchmark {

struct Options {
  double rate{1000.0};   // publish rate in Hz (publisher only)
  uint64_t count{10000}; // total samples in the run
  uint64_t warmup{100};  // received samples excluded from the statistics
};

// Parses the benchmark flags, ignoring anything else (e.g. --ros-args).
inline Options parse(int argc, char *argv[]) {
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = i + 1 < argc;

    if (arg == "--rate" && has_value) {
      options.rate = std::stod(argv[++i]);
    } else if (arg == "--count" && has_value) {
      options.count = std::stoull(argv[++i]);
    } else if (arg == "--warmup" && has_value) {
      options.warmup = std::stoull(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: " << argv[0]
                << " [--rate <hz>] [--count <n>] [--warmup <n>]\n";
      std::exit(0);
    }
  }

  return options;
}

} // namespace benchmark

#endif // RMW_ICEORYX2_BENCHMARK_NODES_OPTIONS_HPP
