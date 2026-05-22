// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/qos/codec.hpp"

#include <charconv>
#include <cstdio>
#include <cstring>
#include <system_error>

namespace rmw::iox2::codec
{

using ::iox2::bb::NULLOPT;

namespace
{

auto read_uint64(const char* first, const char* last, uint64_t& out, const char*& next) -> bool {
    auto result = std::from_chars(first, last, out);
    if (result.ec != std::errc{}) {
        return false;
    }
    next = result.ptr;
    return true;
}

auto read_duration(const char* str, Qos::Duration& out) -> bool {
    const char* end = str + std::strlen(str);
    const char* next = nullptr;
    if (!read_uint64(str, end, out.sec, next) || next == end || *next != ':') {
        return false;
    }
    const char* nsec_start = next + 1;
    if (!read_uint64(nsec_start, end, out.nsec, next) || next != end) {
        return false;
    }
    return true;
}

/// If `str` starts with `prefix` followed by ':', returns the pointer past
/// the colon. Returns `nullptr` otherwise.
auto strip_prefix(const char* str, const char* prefix) -> const char* {
    size_t index = 0;
    for (; prefix[index] != '\0'; ++index) {
        if (str[index] != prefix[index]) {
            return nullptr;
        }
    }
    if (str[index] != ':') {
        return nullptr;
    }
    return str + index + 1;
}

} // namespace

void History::format(const Qos& qos, char* buf, size_t len) {
    // Qos guarantees KEEP_LAST.
    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf, len, "%s:%llu", VALUE_KEEP_LAST, static_cast<unsigned long long>(qos.depth()));
}

auto History::parse(const char* str) -> ::iox2::bb::Optional<uint64_t> {
    const char* rest = strip_prefix(str, VALUE_KEEP_LAST);
    if (rest == nullptr) {
        return NULLOPT;
    }

    const char* end = rest + std::strlen(rest);
    uint64_t depth = 0;
    auto result = std::from_chars(rest, end, depth);
    if (result.ec != std::errc{} || result.ptr != end) {
        return NULLOPT;
    }
    return depth;
}

void Reliability::format(const Qos& qos, char* buf, size_t len) {
    const char* str = qos.reliability() == Qos::Reliability::RELIABLE ? VALUE_RELIABLE : VALUE_BEST_EFFORT;

    // NOLINTNEXTLINE(cert-err33-c) source is a fixed short string constant, destination is 256 bytes
    std::snprintf(buf, len, "%s", str);
}

auto Reliability::parse(const char* str) -> ::iox2::bb::Optional<Qos::Reliability> {
    if (std::strcmp(str, VALUE_RELIABLE) == 0) {
        return Qos::Reliability::RELIABLE;
    }

    if (std::strcmp(str, VALUE_BEST_EFFORT) == 0) {
        return Qos::Reliability::BEST_EFFORT;
    }

    return NULLOPT;
}

void Durability::format(const Qos& qos, char* buf, size_t len) {
    const char* str = qos.durability() == Qos::Durability::TRANSIENT_LOCAL ? VALUE_TRANSIENT_LOCAL : VALUE_VOLATILE;

    // NOLINTNEXTLINE(cert-err33-c) source is a fixed short string constant, destination is 256 bytes
    std::snprintf(buf, len, "%s", str);
}

auto Durability::parse(const char* str) -> ::iox2::bb::Optional<Qos::Durability> {
    if (std::strcmp(str, VALUE_VOLATILE) == 0) {
        return Qos::Durability::VOLATILE;
    }
    if (std::strcmp(str, VALUE_TRANSIENT_LOCAL) == 0) {
        return Qos::Durability::TRANSIENT_LOCAL;
    }

    return NULLOPT;
}

void Deadline::format(const Qos& qos, char* buf, size_t len) {
    auto duration = qos.deadline();
    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf,
                  len,
                  "%s:%llu:%llu",
                  VALUE_DURATION,
                  static_cast<unsigned long long>(duration.sec),
                  static_cast<unsigned long long>(duration.nsec));
}

auto Deadline::parse(const char* str) -> ::iox2::bb::Optional<Qos::Duration> {
    const char* rest = strip_prefix(str, VALUE_DURATION);
    if (rest == nullptr) {
        return NULLOPT;
    }
    Qos::Duration duration{};
    if (!read_duration(rest, duration)) {
        return NULLOPT;
    }
    return duration;
}

void Lifespan::format(const Qos& qos, char* buf, size_t len) {
    auto duration = qos.lifespan();
    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf,
                  len,
                  "%s:%llu:%llu",
                  VALUE_DURATION,
                  static_cast<unsigned long long>(duration.sec),
                  static_cast<unsigned long long>(duration.nsec));
}

auto Lifespan::parse(const char* str) -> ::iox2::bb::Optional<Qos::Duration> {
    const char* rest = strip_prefix(str, VALUE_DURATION);
    if (rest == nullptr) {
        return NULLOPT;
    }
    Qos::Duration duration{};
    if (!read_duration(rest, duration)) {
        return NULLOPT;
    }
    return duration;
}

void Liveliness::format(const Qos& qos, char* buf, size_t len) {
    const char* kind = qos.liveliness() == Qos::Liveliness::MANUAL_BY_TOPIC ? VALUE_MANUAL_BY_TOPIC : VALUE_AUTOMATIC;
    auto lease = qos.liveliness_lease_duration();

    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf,
                  len,
                  "%s:%llu:%llu",
                  kind,
                  static_cast<unsigned long long>(lease.sec),
                  static_cast<unsigned long long>(lease.nsec));
}

auto Liveliness::parse(const char* str) -> ::iox2::bb::Optional<Liveliness::Value> {
    Qos::Liveliness kind = Qos::Liveliness::AUTOMATIC;

    const char* rest = strip_prefix(str, VALUE_AUTOMATIC);
    if (rest == nullptr) {
        rest = strip_prefix(str, VALUE_MANUAL_BY_TOPIC);
        if (rest == nullptr) {
            return NULLOPT;
        }
        kind = Qos::Liveliness::MANUAL_BY_TOPIC;
    }
    Qos::Duration lease{};
    if (!read_duration(rest, lease)) {
        return NULLOPT;
    }

    return Liveliness::Value{kind, lease};
}

} // namespace rmw::iox2::codec
