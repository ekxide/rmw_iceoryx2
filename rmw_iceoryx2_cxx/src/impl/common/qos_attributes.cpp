// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/common/qos_attributes.hpp"

#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"

#include <charconv>
#include <cstdio>
#include <cstring>
#include <system_error>
#include <utility>

namespace rmw::iox2
{

namespace
{

using ::iox2::Attribute;
using ::iox2::AttributeSetView;
using ::iox2::AttributeSpecifier;
using ::iox2::AttributeVerifier;
using ::iox2::bb::err;
using ::iox2::bb::Expected;
using ::iox2::bb::NULLOPT;
using ::iox2::bb::Optional;

// Sentinel that means "match whatever other endpoints have" from rmw/types.h.
constexpr rmw_time_t BEST_AVAILABLE_DURATION = RMW_QOS_DEADLINE_BEST_AVAILABLE;

constexpr uint64_t DEFAULT_DEPTH = 10;

auto map_time(rmw_time_t time) -> ResolvedQos::Duration {
    // BEST_AVAILABLE sentinel collapses to the canonical default (0:0).
    if (time.sec == BEST_AVAILABLE_DURATION.sec && time.nsec == BEST_AVAILABLE_DURATION.nsec) {
        return {0U, 0U};
    }
    return {time.sec, time.nsec};
}

// ----------------------------------------------------------------------------
// Format / parse helpers
// ----------------------------------------------------------------------------

void write_duration(ResolvedQos::Duration duration, char* buf, size_t len) {
    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf,
                  len,
                  "%llu:%llu",
                  static_cast<unsigned long long>(duration.sec),
                  static_cast<unsigned long long>(duration.nsec));
}

auto read_uint64(const char* first, const char* last, uint64_t& out, const char*& next) -> bool {
    auto result = std::from_chars(first, last, out);
    if (result.ec != std::errc{}) {
        return false;
    }
    next = result.ptr;
    return true;
}

auto read_duration(const char* str, ResolvedQos::Duration& out) -> bool {
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

auto is_default_duration(ResolvedQos::Duration duration) -> bool {
    return duration.sec == 0U && duration.nsec == 0U;
}

// ----------------------------------------------------------------------------
// Populate AttributeSpecifier or AttributeVerifier
// ----------------------------------------------------------------------------

template <typename Target>
auto define_or_require(Target& target, const Attribute::Key& key, const char* value) -> bool;

template <>
auto define_or_require<AttributeSpecifier>(AttributeSpecifier& target,
                                           const Attribute::Key& key,
                                           const char* value) -> bool {
    auto val_obj = Attribute::Value::from_utf8_null_terminated_unchecked(value);
    if (!val_obj.has_value()) {
        return false;
    }
    auto res = target.define(key, val_obj.value());
    return res.has_value();
}

template <>
auto define_or_require<AttributeVerifier>(AttributeVerifier& target,
                                          const Attribute::Key& key,
                                          const char* value) -> bool {
    auto val_obj = Attribute::Value::from_utf8_null_terminated_unchecked(value);
    if (!val_obj.has_value()) {
        return false;
    }
    auto res = target.require(key, val_obj.value());
    return res.has_value();
}

template <typename Target>
auto set_qos_attributes(Target& target, const ResolvedQos& qos) -> bool {
    char buf[256];
    for (const auto& policy : POLICIES) {
        auto key = Attribute::Key::from_utf8_null_terminated_unchecked(policy.key);
        if (!key.has_value()) {
            return false;
        }
        policy.format(qos, buf, sizeof(buf));
        if (!define_or_require(target, key.value(), buf)) {
            return false;
        }
    }
    return true;
}

} // namespace

// ----------------------------------------------------------------------------
// Policy codecs
// ----------------------------------------------------------------------------

void History::format(const ResolvedQos& qos, char* buf, size_t len) {
    // ResolvedQos guarantees KEEP_LAST.
    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf, len, "%s:%llu", KEEP_LAST, static_cast<unsigned long long>(qos.depth()));
}

auto History::parse(const char* str) -> ::iox2::bb::Optional<uint64_t> {
    const char* rest = strip_prefix(str, KEEP_LAST);
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

void Reliability::format(const ResolvedQos& qos, char* buf, size_t len) {
    const char* str = qos.reliability() == ResolvedQos::Reliability::RELIABLE ? RELIABLE : BEST_EFFORT;

    // NOLINTNEXTLINE(cert-err33-c) source is a fixed short string constant, destination is 256 bytes
    std::snprintf(buf, len, "%s", str);
}

auto Reliability::parse(const char* str) -> ::iox2::bb::Optional<ResolvedQos::Reliability> {
    if (std::strcmp(str, RELIABLE) == 0) {
        return ResolvedQos::Reliability::RELIABLE;
    }

    if (std::strcmp(str, BEST_EFFORT) == 0) {
        return ResolvedQos::Reliability::BEST_EFFORT;
    }

    return NULLOPT;
}

void Durability::format(const ResolvedQos& qos, char* buf, size_t len) {
    const char* str = qos.durability() == ResolvedQos::Durability::TRANSIENT_LOCAL ? TRANSIENT_LOCAL : VOLATILE;

    // NOLINTNEXTLINE(cert-err33-c) source is a fixed short string constant, destination is 256 bytes
    std::snprintf(buf, len, "%s", str);
}

auto Durability::parse(const char* str) -> ::iox2::bb::Optional<ResolvedQos::Durability> {
    if (std::strcmp(str, VOLATILE) == 0) {
        return ResolvedQos::Durability::VOLATILE;
    }
    if (std::strcmp(str, TRANSIENT_LOCAL) == 0) {
        return ResolvedQos::Durability::TRANSIENT_LOCAL;
    }

    return NULLOPT;
}

void Deadline::format(const ResolvedQos& qos, char* buf, size_t len) {
    write_duration(qos.deadline(), buf, len);
}

auto Deadline::parse(const char* str) -> ::iox2::bb::Optional<ResolvedQos::Duration> {
    ResolvedQos::Duration duration{};
    if (!read_duration(str, duration)) {
        return NULLOPT;
    }

    return duration;
}

void Lifespan::format(const ResolvedQos& qos, char* buf, size_t len) {
    write_duration(qos.lifespan(), buf, len);
}

auto Lifespan::parse(const char* str) -> ::iox2::bb::Optional<ResolvedQos::Duration> {
    ResolvedQos::Duration duration{};
    if (!read_duration(str, duration)) {
        return NULLOPT;
    }

    return duration;
}

void Liveliness::format(const ResolvedQos& qos, char* buf, size_t len) {
    const char* kind = qos.liveliness() == ResolvedQos::Liveliness::MANUAL_BY_TOPIC ? MANUAL_BY_TOPIC : AUTOMATIC;
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
    ResolvedQos::Liveliness kind = ResolvedQos::Liveliness::AUTOMATIC;

    const char* rest = strip_prefix(str, AUTOMATIC);
    if (rest == nullptr) {
        rest = strip_prefix(str, MANUAL_BY_TOPIC);
        if (rest == nullptr) {
            return NULLOPT;
        }
        kind = ResolvedQos::Liveliness::MANUAL_BY_TOPIC;
    }
    ResolvedQos::Duration lease{};
    if (!read_duration(rest, lease)) {
        return NULLOPT;
    }

    return Liveliness::Value{kind, lease};
}

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

auto TryConvert<ResolvedQos>::from(const rmw_qos_profile_t& profile,
                                   ProfileKind kind) -> Expected<ResolvedQos, QosError> {
    (void)kind; // pub/sub and service defaults are identical in v1

    if (profile.history == RMW_QOS_POLICY_HISTORY_UNKNOWN || profile.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN
        || profile.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN
        || profile.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN) {
        RMW_IOX2_CHAIN_ERROR_MSG("QoS contains an UNKNOWN policy value");
        return err(QosError::UNKNOWN_POLICY);
    }

    if (profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        RMW_IOX2_CHAIN_ERROR_MSG("KEEP_ALL not supported; use KEEP_LAST with sufficient depth");
        return err(QosError::UNSUPPORTED_HISTORY_POLICY);
    }

    ResolvedQos::Builder builder;

    builder.set_history(ResolvedQos::History::KEEP_LAST,
                        profile.depth == RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT ? DEFAULT_DEPTH : profile.depth);

    switch (profile.reliability) {
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
        builder.set_reliability(ResolvedQos::Reliability::BEST_EFFORT);
        break;
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE:
    default:
        builder.set_reliability(ResolvedQos::Reliability::RELIABLE);
        break;
    }

    switch (profile.durability) {
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
        builder.set_durability(ResolvedQos::Durability::TRANSIENT_LOCAL);
        break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE:
    default:
        builder.set_durability(ResolvedQos::Durability::VOLATILE);
        break;
    }

    builder.set_deadline(map_time(profile.deadline));
    builder.set_lifespan(map_time(profile.lifespan));

    auto lease = map_time(profile.liveliness_lease_duration);
    switch (profile.liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
        builder.set_liveliness(ResolvedQos::Liveliness::MANUAL_BY_TOPIC, lease);
        break;
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
    case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE:
    default:
        builder.set_liveliness(ResolvedQos::Liveliness::AUTOMATIC, lease);
        break;
    }

    builder.set_avoid_ros_namespace_conventions(profile.avoid_ros_namespace_conventions);

    return std::move(builder).build();
}

auto TryConvert<ResolvedQos>::from(AttributeSetView attrs, ProfileKind kind) -> Expected<ResolvedQos, QosError> {
    (void)kind;

    ResolvedQos::Builder builder;
    char buf[256];

    auto fail = [](const char* key) -> Expected<ResolvedQos, QosError> {
        RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING("failed to decode attribute '%s'", key);
        return err(QosError::ATTRIBUTE_DECODING_FAILURE);
    };

    if (!read_attribute_value(attrs, History::KEY, buf, sizeof(buf))) {
        return fail(History::KEY);
    }
    auto depth = History::parse(buf);
    if (!depth.has_value()) {
        return fail(History::KEY);
    }
    builder.set_history(ResolvedQos::History::KEEP_LAST, depth.value());

    if (!read_attribute_value(attrs, Reliability::KEY, buf, sizeof(buf))) {
        return fail(Reliability::KEY);
    }
    auto reliability = Reliability::parse(buf);
    if (!reliability.has_value()) {
        return fail(Reliability::KEY);
    }
    builder.set_reliability(reliability.value());

    if (!read_attribute_value(attrs, Durability::KEY, buf, sizeof(buf))) {
        return fail(Durability::KEY);
    }
    auto durability = Durability::parse(buf);
    if (!durability.has_value()) {
        return fail(Durability::KEY);
    }
    builder.set_durability(durability.value());

    if (!read_attribute_value(attrs, Deadline::KEY, buf, sizeof(buf))) {
        return fail(Deadline::KEY);
    }
    auto deadline = Deadline::parse(buf);
    if (!deadline.has_value()) {
        return fail(Deadline::KEY);
    }
    builder.set_deadline(deadline.value());

    if (!read_attribute_value(attrs, Lifespan::KEY, buf, sizeof(buf))) {
        return fail(Lifespan::KEY);
    }
    auto lifespan = Lifespan::parse(buf);
    if (!lifespan.has_value()) {
        return fail(Lifespan::KEY);
    }
    builder.set_lifespan(lifespan.value());

    if (!read_attribute_value(attrs, Liveliness::KEY, buf, sizeof(buf))) {
        return fail(Liveliness::KEY);
    }
    auto liveliness = Liveliness::parse(buf);
    if (!liveliness.has_value()) {
        return fail(Liveliness::KEY);
    }
    builder.set_liveliness(liveliness.value().kind, liveliness.value().lease);

    return std::move(builder).build();
}

auto TryConvert<AttributeSpecifier>::from(const ResolvedQos& qos) -> Expected<AttributeSpecifier, QosError> {
    AttributeSpecifier specifier;
    if (!set_qos_attributes(specifier, qos)) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to define one or more QoS attributes on AttributeSpecifier");
        return err(QosError::ATTRIBUTE_DEFINITION_FAILURE);
    }
    return specifier;
}

auto TryConvert<AttributeVerifier>::from(const ResolvedQos& qos) -> Expected<AttributeVerifier, QosError> {
    AttributeVerifier verifier;
    if (!set_qos_attributes(verifier, qos)) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to require one or more QoS attributes on AttributeVerifier");
        return err(QosError::ATTRIBUTE_DEFINITION_FAILURE);
    }
    return verifier;
}

// ----------------------------------------------------------------------------
// Diagnostics
// ----------------------------------------------------------------------------

void for_each_policy(const std::function<void(const PolicyCodec&)>& func) {
    for (const auto& policy : POLICIES) {
        func(policy);
    }
}

auto read_attribute_value(::iox2::AttributeSetView attrs, const char* key, char* out, size_t out_size) -> bool {
    auto key_obj = Attribute::Key::from_utf8_null_terminated_unchecked(key);
    if (!key_obj.has_value()) {
        return false;
    }
    auto val = attrs.key_value(key_obj.value(), 0);
    if (!val.has_value()) {
        return false;
    }
    // NOLINTNEXTLINE(cert-err33-c) source and destination both bounded by IOX2_ATTRIBUTE_VALUE_LENGTH
    std::snprintf(out, out_size, "%s", val.value().unchecked_access().c_str());
    return true;
}

void log_unsupported_policies(const ResolvedQos& qos, const char* topic) noexcept {
    if (!is_default_duration(qos.deadline())) {
        RMW_IOX2_LOG_WARN("QoS policy 'deadline' (=%llu:%llu) on topic '%s' is not honored by the iceoryx2 transport",
                          static_cast<unsigned long long>(qos.deadline().sec),
                          static_cast<unsigned long long>(qos.deadline().nsec),
                          topic);
    }
    if (!is_default_duration(qos.lifespan())) {
        RMW_IOX2_LOG_WARN("QoS policy 'lifespan' (=%llu:%llu) on topic '%s' is not honored by the iceoryx2 transport",
                          static_cast<unsigned long long>(qos.lifespan().sec),
                          static_cast<unsigned long long>(qos.lifespan().nsec),
                          topic);
    }
    if (!is_default_duration(qos.liveliness_lease_duration())) {
        RMW_IOX2_LOG_WARN("QoS policy 'liveliness_lease_duration' (=%llu:%llu) on topic '%s' is not honored by the "
                          "iceoryx2 transport",
                          static_cast<unsigned long long>(qos.liveliness_lease_duration().sec),
                          static_cast<unsigned long long>(qos.liveliness_lease_duration().nsec),
                          topic);
    }

    if (qos.liveliness() == ResolvedQos::Liveliness::MANUAL_BY_TOPIC) {
        RMW_IOX2_LOG_WARN(
            "QoS policy 'liveliness' (=manual_by_topic) on topic '%s' is not honored by the iceoryx2 transport", topic);
    }
}

void chain_attribute_mismatch_error(const ResolvedQos& requested,
                                    ::iox2::AttributeSetView attrs,
                                    const char* topic) noexcept {
    char message[rmw::iox2::MAX_ERROR_MSG_LENGTH];
    int written = std::snprintf(message, sizeof(message), "QoS mismatch on '%s':", topic);
    size_t offset = (written > 0) ? static_cast<size_t>(written) : 0;
    if (offset >= sizeof(message)) {
        offset = sizeof(message) - 1;
    }

    size_t count = 0;
    char requested_val[256];
    char existing_val[256];

    for_each_policy([&](const PolicyCodec& policy) {
        if (offset >= sizeof(message)) {
            return;
        }
        policy.format(requested, requested_val, sizeof(requested_val));
        if (!read_attribute_value(attrs, policy.key, existing_val, sizeof(existing_val))) {
            return;
        }
        if (std::strcmp(requested_val, existing_val) == 0) {
            return;
        }
        // NOLINTNEXTLINE(cert-err33-c) buffer sized at MAX_ERROR_MSG_LENGTH; trailing diffs truncate if exhausted
        int written = std::snprintf(message + offset,
                                    sizeof(message) - offset,
                                    "%s %s [existing=%s, requested=%s]",
                                    count == 0 ? "" : ";",
                                    policy.key,
                                    existing_val,
                                    requested_val);
        if (written <= 0) {
            return;
        }
        offset += static_cast<size_t>(written);
        if (offset >= sizeof(message)) {
            offset = sizeof(message) - 1;
        }
        ++count;
    });

    if (count == 0) {
        // Shouldn't happen after OpenIncompatibleAttributes.
        RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING("QoS mismatch on '%s' but no per-key diff was produced", topic);
        return;
    }

    // NOLINTNEXTLINE(cert-err33-c) buffer sized at MAX_ERROR_MSG_LENGTH; trailing hint truncates if exhausted
    std::snprintf(message + offset,
                  sizeof(message) - offset,
                  ". Set RMW_IOX2_QOS_MATCH=adopt to auto-match the existing service.");

    RMW_IOX2_CHAIN_ERROR_MSG(message);
}

} // namespace rmw::iox2
