// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/qos/port_name.hpp"

#include "rmw/time.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace rmw::iox2::port_name
{

using ::iox2::bb::NULLOPT;
using ::iox2::bb::Optional;

namespace
{

constexpr std::string_view VERSION = "V1";
constexpr std::string_view FIELDS = "DEST";
constexpr char FLAGS = 'V';
constexpr char DEPTH = 'D';
constexpr char DEADLINE = 'E';
constexpr char LIFESPAN = 'S';
constexpr char LEASE = 'T';
constexpr int BASE = 36;

constexpr uint64_t KEEP_ALL = 1U;
constexpr uint64_t BEST_EFFORT = 2U;
constexpr uint64_t TRANSIENT_LOCAL = 4U;
constexpr uint64_t MANUAL_BY_TOPIC = 8U;
constexpr uint64_t AVOID_ROS_NAMESPACE_CONVENTIONS = 16U;
constexpr uint64_t ALL_FLAGS =
    KEEP_ALL | BEST_EFFORT | TRANSIENT_LOCAL | MANUAL_BY_TOPIC | AVOID_ROS_NAMESPACE_CONVENTIONS;

constexpr uint64_t MAX_NANOSECONDS = std::numeric_limits<rmw_duration_t>::max();
constexpr rmw_time_t UNSET = RMW_DURATION_UNSPECIFIED;

auto append(std::string& name, uint64_t value) -> void {
    std::array<char, std::numeric_limits<uint64_t>::digits> digits{};
    const auto result = std::to_chars(digits.data(), digits.data() + digits.size(), value, BASE);
    name.append(digits.data(), result.ptr);
}

auto parse(std::string_view text) -> Optional<uint64_t> {
    uint64_t value{};
    const auto* begin = text.data();
    const auto* end = begin + text.size();
    const auto result = std::from_chars(begin, end, value, BASE);
    if (result.ec != std::errc{} || result.ptr != end) {
        return NULLOPT;
    }
    return value;
}

auto to_duration(uint64_t nanoseconds) -> Qos::Duration {
    const auto time = rmw_time_from_nsec(static_cast<rmw_duration_t>(std::min(nanoseconds, MAX_NANOSECONDS)));
    return Qos::Duration{time.sec, time.nsec};
}

} // namespace

auto encode(const Qos& qos) -> ::iox2::bb::Expected<::iox2::PortName, ::iox2::bb::SemanticStringError> {
    uint64_t flags{};
    if (qos.history() == Qos::History::KEEP_ALL) {
        flags |= KEEP_ALL;
    }
    if (qos.reliability() == Qos::Reliability::BEST_EFFORT) {
        flags |= BEST_EFFORT;
    }
    if (qos.durability() == Qos::Durability::TRANSIENT_LOCAL) {
        flags |= TRANSIENT_LOCAL;
    }
    if (qos.liveliness() == Qos::Liveliness::MANUAL_BY_TOPIC) {
        flags |= MANUAL_BY_TOPIC;
    }
    if (qos.avoid_ros_namespace_conventions()) {
        flags |= AVOID_ROS_NAMESPACE_CONVENTIONS;
    }

    std::string name{VERSION};
    append(name, flags);
    name += DEPTH;
    append(name, qos.depth());
    const std::array durations{std::pair{DEADLINE, qos.deadline()},
                               std::pair{LIFESPAN, qos.lifespan()},
                               std::pair{LEASE, qos.liveliness_lease_duration()}};
    for (const auto& [field, duration] : durations) {
        const rmw_time_t time{duration.sec, duration.nsec};
        if (!rmw_time_equal(time, UNSET)) {
            name += field;
            append(name, static_cast<uint64_t>(rmw_time_total_nsec(time)));
        }
    }
    return ::iox2::PortName::create(name.c_str());
}

auto decode(::iox2::PortNameView name) -> Optional<Qos> {
    const auto owned = name.to_string();
    std::string_view text{owned.unchecked_access().c_str()};
    if (text.substr(0, VERSION.size()) != VERSION) {
        return NULLOPT;
    }
    text.remove_prefix(VERSION.size());

    uint64_t flags{};
    Optional<uint64_t> depth;
    Qos::Duration deadline{};
    Qos::Duration lifespan{};
    Qos::Duration lease{};
    char field = FLAGS;
    while (true) {
        const auto end = text.find_first_of(FIELDS);
        auto value = parse(text.substr(0, end));
        if (!value.has_value()) {
            return NULLOPT;
        }
        switch (field) {
        case FLAGS:
            flags = value.value();
            break;
        case DEPTH:
            depth = value;
            break;
        case DEADLINE:
            deadline = to_duration(value.value());
            break;
        case LIFESPAN:
            lifespan = to_duration(value.value());
            break;
        case LEASE:
            lease = to_duration(value.value());
            break;
        default:
            return NULLOPT;
        }
        if (end == std::string_view::npos) {
            break;
        }
        field = text[end];
        text.remove_prefix(end + 1);
    }
    if (!depth.has_value() || flags > ALL_FLAGS) {
        return NULLOPT;
    }

    return Qos::Builder{}
        .set_history((flags & KEEP_ALL) == KEEP_ALL ? Qos::History::KEEP_ALL : Qos::History::KEEP_LAST, depth.value())
        .set_reliability((flags & BEST_EFFORT) == BEST_EFFORT ? Qos::Reliability::BEST_EFFORT
                                                              : Qos::Reliability::RELIABLE)
        .set_durability((flags & TRANSIENT_LOCAL) == TRANSIENT_LOCAL ? Qos::Durability::TRANSIENT_LOCAL
                                                                     : Qos::Durability::VOLATILE)
        .set_deadline(deadline)
        .set_lifespan(lifespan)
        .set_liveliness((flags & MANUAL_BY_TOPIC) == MANUAL_BY_TOPIC ? Qos::Liveliness::MANUAL_BY_TOPIC
                                                                     : Qos::Liveliness::AUTOMATIC,
                        lease)
        .set_avoid_ros_namespace_conventions((flags & AVOID_ROS_NAMESPACE_CONVENTIONS)
                                             == AVOID_ROS_NAMESPACE_CONVENTIONS)
        .build();
}

} // namespace rmw::iox2::port_name
