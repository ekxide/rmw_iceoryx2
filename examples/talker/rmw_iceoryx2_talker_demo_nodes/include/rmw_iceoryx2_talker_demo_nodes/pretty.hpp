// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_ICEORYX2_TALKER_DEMO_NODES_PRETTY_HPP
#define RMW_ICEORYX2_TALKER_DEMO_NODES_PRETTY_HPP

#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace pretty {

// Direction of a message relative to the node printing it, which selects the
// ANSI colour used to highlight the frame header.
enum class Direction { Sent, Received };

// A single labelled value within a message frame.
using Field = std::pair<std::string, std::string>;

// Renders a message as a left-bordered, colour-headed frame. An optional `meta`
// string is appended to the header line (the listener uses it for the sequence
// number; the talker leaves it empty). A leading newline lets the frame start
// on a fresh line below the logger prefix so its borders align.
//
//   ╭─ RECV · seq 42
//   │  bool_value     false
//   │  int32_value    42
//   ╰────────────────────────────
inline std::string frame(Direction direction, const std::string &meta,
                         const std::vector<Field> &fields) {
  const bool sent = direction == Direction::Sent;
  const char *label = sent ? "SENT" : "RECV";
  const char *colour = sent ? "\033[1;32m" : "\033[1;36m"; // bold green / cyan
  const char *reset = "\033[0m";

  std::ostringstream out;
  out << '\n' << colour << "╭─ " << label << reset;
  if (!meta.empty()) {
    out << " · " << meta;
  }
  out << '\n';
  for (const auto &[name, value] : fields) {
    out << "│  " << std::left << std::setw(14) << name << value << '\n';
  }
  out << "╰────────────────────────────";
  return out.str();
}

// Formats a floating point value with two decimal places.
inline std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", value);
  return buffer;
}

} // namespace pretty

#endif // RMW_ICEORYX2_TALKER_DEMO_NODES_PRETTY_HPP
