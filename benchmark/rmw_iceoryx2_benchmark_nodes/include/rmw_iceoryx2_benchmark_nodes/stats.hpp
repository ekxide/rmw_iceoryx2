// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_ICEORYX2_BENCHMARK_NODES_STATS_HPP
#define RMW_ICEORYX2_BENCHMARK_NODES_STATS_HPP

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace benchmark {

// Formats a duration given in nanoseconds, picking a readable unit.
inline std::string format_latency(int64_t nanoseconds) {
  char buffer[32];
  if (nanoseconds < 1'000) {
    std::snprintf(buffer, sizeof(buffer), "%lld ns",
                  static_cast<long long>(nanoseconds));
  } else if (nanoseconds < 1'000'000) {
    std::snprintf(buffer, sizeof(buffer), "%.1f µs", nanoseconds / 1'000.0);
  } else {
    std::snprintf(buffer, sizeof(buffer), "%.2f ms", nanoseconds / 1'000'000.0);
  }
  return buffer;
}

// Collects one-way latencies and renders a summary with the percentile
// distribution drawn as proportional bars, e.g.:
//
//   REPORT · ros2 subscriber
//   samples 10000/10000 · lost 0 · warmup 100
//
//     min    4.1 µs ▏
//     p50    8.9 µs ███▎
//     p99   21.0 µs ████████▏
//     max   98.2 µs ██████████████████████████████████████
//
//     mean   9.4 µs
class LatencyRecorder {
public:
  explicit LatencyRecorder(uint64_t warmup) : m_warmup(warmup) {}

  // Records one received sample; the first `warmup` samples are counted but
  // excluded from the latency statistics.
  void record(int64_t latency_nanoseconds) {
    ++m_received;
    if (m_received <= m_warmup) {
      return;
    }
    m_latencies.push_back(latency_nanoseconds);
  }

  uint64_t received() const { return m_received; }

  std::string report(const std::string &label, uint64_t expected) const {
    const char *colour = "\033[1;35m"; // bold magenta
    const char *reset = "\033[0m";

    std::ostringstream out;
    out << '\n' << colour << "REPORT" << reset << " · " << label << '\n';
    out << "samples " << m_received << "/" << expected << " · lost "
        << (expected - m_received) << " · warmup " << m_warmup << '\n';

    if (m_latencies.empty()) {
      return out.str();
    }

    auto sorted = m_latencies;
    std::sort(sorted.begin(), sorted.end());
    auto percentile = [&sorted](double fraction) {
      const auto index = static_cast<size_t>(
          fraction * static_cast<double>(sorted.size() - 1));
      return sorted[index];
    };
    const auto max = sorted.back();
    auto row = [&out, max](const std::string &name, int64_t value,
                           bool with_bar) {
      out << "  " << name << std::string(5 - name.size(), ' ')
          << pad_left(format_latency(value), 8);
      if (with_bar) {
        out << ' ' << bar(value, max);
      }
      out << '\n';
    };

    out << '\n';
    row("min", sorted.front(), true);
    row("p50", percentile(0.50), true);
    row("p90", percentile(0.90), true);
    row("p99", percentile(0.99), true);
    row("max", max, true);
    out << '\n';
    const auto sum = std::accumulate(sorted.begin(), sorted.end(), int64_t{0});
    row("mean", sum / static_cast<int64_t>(sorted.size()), false);
    return out.str();
  }

private:
  // Right-aligns by display columns; the µ in `format_latency` output is two
  // bytes but one column, which std::setw would miscount.
  static std::string pad_left(const std::string &text, size_t width) {
    size_t display_width = 0;
    for (const char c : text) {
      display_width += (static_cast<unsigned char>(c) & 0xC0) != 0x80;
    }
    if (display_width >= width) {
      return text;
    }
    return std::string(width - display_width, ' ') + text;
  }

  // A bar proportional to value/max, using eighth-blocks for the remainder.
  static std::string bar(int64_t value, int64_t max) {
    constexpr int FULL_WIDTH = 38;
    static const char *EIGHTHS[] = {"", "▏", "▎", "▍", "▌", "▋", "▊", "▉"};
    if (max <= 0) {
      return "";
    }
    const double cells =
        static_cast<double>(value) / static_cast<double>(max) * FULL_WIDTH;
    std::string out;
    for (int i = 0; i < static_cast<int>(cells); ++i) {
      out += "█";
    }
    out += EIGHTHS[static_cast<int>((cells - static_cast<int>(cells)) * 8)];
    if (out.empty() && value > 0) {
      out = "▏"; // a nonzero value always gets a visible bar
    }
    return out;
  }

  uint64_t m_warmup{0};
  uint64_t m_received{0};
  std::vector<int64_t> m_latencies;
};

} // namespace benchmark

#endif // RMW_ICEORYX2_BENCHMARK_NODES_STATS_HPP
