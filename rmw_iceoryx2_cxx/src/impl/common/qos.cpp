// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/common/qos.hpp"

#include "iox2/config.hpp"
#include "iox2/messaging_pattern.hpp"
#include "iox2/service.hpp"
#include "iox2/service_name.hpp"
#include "iox2/service_type.hpp"
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
// Attribute lookup helpers
// ----------------------------------------------------------------------------

/// Fetch the value of `key` from the attribute set into `out` (null-terminated).
/// Returns false if the key is absent.
auto get_attribute_value(AttributeSetView attrs, const char* key, char* out, size_t out_size) -> bool {
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

// ----------------------------------------------------------------------------
// History
// ----------------------------------------------------------------------------

struct History
{
    static constexpr char KEY[] = "rmw.qos.local.history";
    static constexpr char KEEP_LAST[] = "keep_last";

    static void format(const ResolvedQos& qos, char* buf, size_t len) {
        // ResolvedQos guarantees KEEP_LAST.
        // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
        std::snprintf(buf, len, "%s:%llu", KEEP_LAST, static_cast<unsigned long long>(qos.depth()));
    }

    static auto parse(const char* str, ResolvedQos::Builder& builder) -> bool {
        const char* rest = strip_prefix(str, KEEP_LAST);
        if (rest == nullptr) {
            return false;
        }
        const char* end = rest + std::strlen(rest);
        uint64_t depth = 0;
        auto result = std::from_chars(rest, end, depth);
        if (result.ec != std::errc{} || result.ptr != end) {
            return false;
        }
        builder.set_history(ResolvedQos::History::KEEP_LAST, depth);
        return true;
    }
};

// ----------------------------------------------------------------------------
// Reliability
// ----------------------------------------------------------------------------

struct Reliability
{
    static constexpr char KEY[] = "rmw.qos.local.reliability";
    static constexpr char RELIABLE[] = "reliable";
    static constexpr char BEST_EFFORT[] = "best_effort";

    static void format(const ResolvedQos& qos, char* buf, size_t len) {
        const char* str = qos.reliability() == ResolvedQos::Reliability::RELIABLE ? RELIABLE : BEST_EFFORT;
        // NOLINTNEXTLINE(cert-err33-c) source is a fixed short string constant, destination is 256 bytes
        std::snprintf(buf, len, "%s", str);
    }

    static auto parse(const char* str, ResolvedQos::Builder& builder) -> bool {
        if (std::strcmp(str, RELIABLE) == 0) {
            builder.set_reliability(ResolvedQos::Reliability::RELIABLE);
            return true;
        }
        if (std::strcmp(str, BEST_EFFORT) == 0) {
            builder.set_reliability(ResolvedQos::Reliability::BEST_EFFORT);
            return true;
        }
        return false;
    }
};

// ----------------------------------------------------------------------------
// Durability
// ----------------------------------------------------------------------------

struct Durability
{
    static constexpr char KEY[] = "rmw.qos.local.durability";
    static constexpr char VOLATILE[] = "volatile";
    static constexpr char TRANSIENT_LOCAL[] = "transient_local";

    static void format(const ResolvedQos& qos, char* buf, size_t len) {
        const char* str = qos.durability() == ResolvedQos::Durability::TRANSIENT_LOCAL ? TRANSIENT_LOCAL : VOLATILE;
        // NOLINTNEXTLINE(cert-err33-c) source is a fixed short string constant, destination is 256 bytes
        std::snprintf(buf, len, "%s", str);
    }

    static auto parse(const char* str, ResolvedQos::Builder& builder) -> bool {
        if (std::strcmp(str, VOLATILE) == 0) {
            builder.set_durability(ResolvedQos::Durability::VOLATILE);
            return true;
        }
        if (std::strcmp(str, TRANSIENT_LOCAL) == 0) {
            builder.set_durability(ResolvedQos::Durability::TRANSIENT_LOCAL);
            return true;
        }
        return false;
    }
};

// ----------------------------------------------------------------------------
// Deadline
// ----------------------------------------------------------------------------

struct Deadline
{
    static constexpr char KEY[] = "rmw.qos.local.deadline";

    static auto format(const ResolvedQos& qos, char* buf, size_t len) -> void {
        write_duration(qos.deadline(), buf, len);
    }

    static auto parse(const char* str, ResolvedQos::Builder& builder) -> bool {
        ResolvedQos::Duration duration{};
        if (!read_duration(str, duration)) {
            return false;
        }
        builder.set_deadline(duration);
        return true;
    }
};

// ----------------------------------------------------------------------------
// Lifespan
// ----------------------------------------------------------------------------

struct Lifespan
{
    static constexpr char KEY[] = "rmw.qos.local.lifespan";

    static void format(const ResolvedQos& qos, char* buf, size_t len) {
        write_duration(qos.lifespan(), buf, len);
    }

    static auto parse(const char* str, ResolvedQos::Builder& builder) -> bool {
        ResolvedQos::Duration duration{};
        if (!read_duration(str, duration)) {
            return false;
        }
        builder.set_lifespan(duration);
        return true;
    }
};

// ----------------------------------------------------------------------------
// Liveliness
// ----------------------------------------------------------------------------

struct Liveliness
{
    static constexpr char KEY[] = "rmw.qos.local.liveliness";
    static constexpr char AUTOMATIC[] = "automatic";
    static constexpr char MANUAL_BY_TOPIC[] = "manual_by_topic";

    static void format(const ResolvedQos& qos, char* buf, size_t len) {
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

    static auto parse(const char* str, ResolvedQos::Builder& builder) -> bool {
        ResolvedQos::Liveliness kind = ResolvedQos::Liveliness::AUTOMATIC;
        const char* rest = strip_prefix(str, AUTOMATIC);
        if (rest == nullptr) {
            rest = strip_prefix(str, MANUAL_BY_TOPIC);
            if (rest == nullptr) {
                return false;
            }
            kind = ResolvedQos::Liveliness::MANUAL_BY_TOPIC;
        }
        ResolvedQos::Duration lease{};
        if (!read_duration(rest, lease)) {
            return false;
        }
        builder.set_liveliness(kind, lease);
        return true;
    }
};

// ----------------------------------------------------------------------------
// Lookup table
// ----------------------------------------------------------------------------

struct PolicyLookup
{
    const char* key;
    void (*format)(const ResolvedQos&, char*, size_t);
    bool (*parse)(const char*, ResolvedQos::Builder&);
};

constexpr PolicyLookup POLICIES[] = {
    {History::KEY, &History::format, &History::parse},
    {Reliability::KEY, &Reliability::format, &Reliability::parse},
    {Durability::KEY, &Durability::format, &Durability::parse},
    {Deadline::KEY, &Deadline::format, &Deadline::parse},
    {Lifespan::KEY, &Lifespan::format, &Lifespan::parse},
    {Liveliness::KEY, &Liveliness::format, &Liveliness::parse},
};

// ----------------------------------------------------------------------------
// set_qos_attributes — fills an AttributeSpecifier or AttributeVerifier
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

// ----------------------------------------------------------------------------
// diff_attributes helper
// ----------------------------------------------------------------------------

// TODO: Rename
auto record_diff(AttributeDiff& diff, const char* key, const char* requested, const char* existing) -> bool {
    if (std::strcmp(requested, existing) == 0) {
        return false;
    }
    // NOLINTNEXTLINE(cert-err33-c) AttributeDiff fields sized to the iceoryx2 attribute key/value limits
    std::snprintf(diff.key, sizeof(diff.key), "%s", key);
    // NOLINTNEXTLINE(cert-err33-c) AttributeDiff fields sized to the iceoryx2 attribute key/value limits
    std::snprintf(diff.requested, sizeof(diff.requested), "%s", requested);
    // NOLINTNEXTLINE(cert-err33-c) AttributeDiff fields sized to the iceoryx2 attribute key/value limits
    std::snprintf(diff.existing, sizeof(diff.existing), "%s", existing);
    return true;
}

} // namespace

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

auto TryConvert<ResolvedQos>::from(AttributeSetView attrs, ProfileKind kind) -> Expected<ResolvedQos, QosError> {
    (void)kind;

    ResolvedQos::Builder builder;
    char buf[256];

    for (const auto& policy : POLICIES) {
        if (!get_attribute_value(attrs, policy.key, buf, sizeof(buf)) || !policy.parse(buf, builder)) {
            RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING("failed to decode attribute '%s'", policy.key);
            return err(QosError::ATTRIBUTE_DECODING_FAILURE);
        }
    }

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
// Diff Calculation
// ----------------------------------------------------------------------------

auto diff_attributes(const ResolvedQos& required,
                     AttributeSetView actual) -> ::iox2::bb::StaticVector<AttributeDiff, MAX_POLICY_DIFFS> {
    ::iox2::bb::StaticVector<AttributeDiff, MAX_POLICY_DIFFS> result{};
    char requested[256];
    char existing[256];

    for (const auto& policy : POLICIES) {
        policy.format(required, requested, sizeof(requested));
        if (!get_attribute_value(actual, policy.key, existing, sizeof(existing))) {
            continue;
        }
        AttributeDiff diff{};
        if (record_diff(diff, policy.key, requested, existing)) {
            result.try_push_back(diff);
        }
    }

    return result;
}

// ----------------------------------------------------------------------------
// Error Handling
// ----------------------------------------------------------------------------

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
                                    const ::iox2::ServiceName& service_name,
                                    ::iox2::ConfigView config,
                                    const char* topic) noexcept {
    auto details = ::iox2::Service<::iox2::ServiceType::Ipc>::details(
        service_name, config, ::iox2::MessagingPattern::PublishSubscribe);

    if (!details.has_value() || !details.value().has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING("QoS mismatch on '%s' (failed to read existing service attributes)",
                                                    topic);
        return;
    }

    auto attrs = details.value().value().static_details.attributes();
    auto diffs = diff_attributes(requested, attrs);
    if (diffs.empty()) {
        // Shouldn't happen after OpenIncompatibleAttributes.
        RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING("QoS mismatch on '%s' but no per-key diff was produced", topic);
        return;
    }

    char message[rmw::iox2::MAX_ERROR_MSG_LENGTH];
    int written = std::snprintf(message, sizeof(message), "QoS mismatch on '%s':", topic);
    size_t offset = (written > 0) ? static_cast<size_t>(written) : 0;
    if (offset >= sizeof(message)) {
        offset = sizeof(message) - 1;
    }

    for (size_t i = 0; i < diffs.size(); ++i) {
        auto diff = diffs.unchecked_access()[i];
        const char* sep = (i + 1 < diffs.size()) ? ";" : ".";

        // NOLINTNEXTLINE(cert-err33-c) buffer sized at MAX_ERROR_MSG_LENGTH; trailing diffs truncate if exhausted
        written = std::snprintf(message + offset,
                                sizeof(message) - offset,
                                " %s [existing=%s, requested=%s]%s",
                                diff.key,
                                diff.existing,
                                diff.requested,
                                sep);
        if (written <= 0) {
            break;
        }
        offset += static_cast<size_t>(written);
        if (offset >= sizeof(message)) {
            offset = sizeof(message) - 1;
            break;
        }
    }

    // NOLINTNEXTLINE(cert-err33-c) buffer sized at MAX_ERROR_MSG_LENGTH; trailing hint truncates if exhausted
    std::snprintf(message + offset,
                  sizeof(message) - offset,
                  " Set RMW_IOX2_QOS_MATCH=adopt to auto-match the existing service.");

    RMW_IOX2_CHAIN_ERROR_MSG(message);
}

} // namespace rmw::iox2
