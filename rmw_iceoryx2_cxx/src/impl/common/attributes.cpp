// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT


#include "rmw_iceoryx2_cxx/impl/common/attributes.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/matching.hpp"

#include "rcutils/allocator.h"
#include "rosidl_runtime_c/type_hash.h"

#include <charconv>
#include <cstdio>
#include <cstring>
#include <system_error>

namespace rmw::iox2::attributes
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

auto History::encode(const Qos& qos, char* buf, size_t len) -> void {
    // Qos guarantees KEEP_LAST.
    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf, len, "%s:%llu", VALUE_KEEP_LAST, static_cast<unsigned long long>(qos.depth()));
}

auto History::decode(const char* str) -> ::iox2::bb::Optional<uint64_t> {
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

auto Reliability::encode(const Qos& qos, char* buf, size_t len) -> void {
    const char* str = qos.reliability() == Qos::Reliability::RELIABLE ? VALUE_RELIABLE : VALUE_BEST_EFFORT;

    // NOLINTNEXTLINE(cert-err33-c) source is a fixed short string constant, destination is 256 bytes
    std::snprintf(buf, len, "%s", str);
}

auto Reliability::decode(const char* str) -> ::iox2::bb::Optional<Qos::Reliability> {
    if (std::strcmp(str, VALUE_RELIABLE) == 0) {
        return Qos::Reliability::RELIABLE;
    }

    if (std::strcmp(str, VALUE_BEST_EFFORT) == 0) {
        return Qos::Reliability::BEST_EFFORT;
    }

    return NULLOPT;
}

auto Durability::encode(const Qos& qos, char* buf, size_t len) -> void {
    const char* str = qos.durability() == Qos::Durability::TRANSIENT_LOCAL ? VALUE_TRANSIENT_LOCAL : VALUE_VOLATILE;

    // NOLINTNEXTLINE(cert-err33-c) source is a fixed short string constant, destination is 256 bytes
    std::snprintf(buf, len, "%s", str);
}

auto Durability::decode(const char* str) -> ::iox2::bb::Optional<Qos::Durability> {
    if (std::strcmp(str, VALUE_VOLATILE) == 0) {
        return Qos::Durability::VOLATILE;
    }
    if (std::strcmp(str, VALUE_TRANSIENT_LOCAL) == 0) {
        return Qos::Durability::TRANSIENT_LOCAL;
    }

    return NULLOPT;
}

auto Deadline::encode(const Qos& qos, char* buf, size_t len) -> void {
    auto duration = qos.deadline();
    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf,
                  len,
                  "%s:%llu:%llu",
                  VALUE_DURATION,
                  static_cast<unsigned long long>(duration.sec),
                  static_cast<unsigned long long>(duration.nsec));
}

auto Deadline::decode(const char* str) -> ::iox2::bb::Optional<Qos::Duration> {
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

auto Lifespan::encode(const Qos& qos, char* buf, size_t len) -> void {
    auto duration = qos.lifespan();
    // NOLINTNEXTLINE(cert-err33-c) buffer statically sized for max output (caller passes a 256-byte buffer)
    std::snprintf(buf,
                  len,
                  "%s:%llu:%llu",
                  VALUE_DURATION,
                  static_cast<unsigned long long>(duration.sec),
                  static_cast<unsigned long long>(duration.nsec));
}

auto Lifespan::decode(const char* str) -> ::iox2::bb::Optional<Qos::Duration> {
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

auto Liveliness::encode(const Qos& qos, char* buf, size_t len) -> void {
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

auto Liveliness::decode(const char* str) -> ::iox2::bb::Optional<Liveliness::Value> {
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

auto TypeHash::encode(const rosidl_type_hash_t& type_hash, char* buf, size_t len) -> void {
    auto allocator = rcutils_get_default_allocator();
    char* hash_string = nullptr;
    if (rosidl_stringify_type_hash(&type_hash, allocator, &hash_string) != RCUTILS_RET_OK || hash_string == nullptr) {
        if (len > 0) {
            buf[0] = '\0';
        }
        return;
    }
    // NOLINTNEXTLINE(cert-err33-c) source is a bounded RIHS string, destination is 256 bytes
    std::snprintf(buf, len, "%s", hash_string);
    allocator.deallocate(hash_string, allocator.state);
}

auto TypeHash::decode(const char* str) -> ::iox2::bb::Optional<rosidl_type_hash_t> {
    rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
    if (rosidl_parse_type_hash_string(str, &hash) != RCUTILS_RET_OK) {
        return NULLOPT;
    }
    return hash;
}

} // namespace rmw::iox2::attributes

namespace rmw::iox2
{

namespace
{

namespace matching = ::rmw::iox2::matching;
namespace attributes = ::rmw::iox2::attributes;

using ::iox2::Attribute;
using ::iox2::AttributeSetView;
using ::iox2::AttributeSpecifier;
using ::iox2::AttributeVerifier;
using ::iox2::bb::err;
using ::iox2::bb::Expected;
using ::iox2::bb::Optional;

auto to_duration(rmw_time_t time) -> Qos::Duration {
    auto resolved = matching::resolve(time);
    return {resolved.sec, resolved.nsec};
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
auto write_attribute(Target& target, const char* key, const char* value) -> bool {
    auto key_obj = Attribute::Key::from_utf8_null_terminated_unchecked(key);
    if (!key_obj.has_value()) {
        return false;
    }
    return define_or_require(target, key_obj.value(), value);
}

template <typename Target>
auto set_type_hash_attribute(Target& target, const rosidl_type_hash_t& type_hash) -> bool {
    char buf[256];
    attributes::TypeHash::encode(type_hash, buf, sizeof(buf));
    return write_attribute(target, attributes::TypeHash::KEY, buf);
}

template <typename Target>
auto set_qos_attributes(Target& target, const Qos& qos) -> bool {
    char buf[256];
    attributes::History::encode(qos, buf, sizeof(buf));
    if (!write_attribute(target, attributes::History::KEY, buf)) {
        return false;
    }
    attributes::Reliability::encode(qos, buf, sizeof(buf));
    if (!write_attribute(target, attributes::Reliability::KEY, buf)) {
        return false;
    }
    attributes::Durability::encode(qos, buf, sizeof(buf));
    if (!write_attribute(target, attributes::Durability::KEY, buf)) {
        return false;
    }
    attributes::Deadline::encode(qos, buf, sizeof(buf));
    if (!write_attribute(target, attributes::Deadline::KEY, buf)) {
        return false;
    }
    attributes::Lifespan::encode(qos, buf, sizeof(buf));
    if (!write_attribute(target, attributes::Lifespan::KEY, buf)) {
        return false;
    }
    attributes::Liveliness::encode(qos, buf, sizeof(buf));
    if (!write_attribute(target, attributes::Liveliness::KEY, buf)) {
        return false;
    }
    return true;
}

} // namespace

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

auto Convert<rmw_qos_profile_t>::from(const Qos& qos) noexcept -> rmw_qos_profile_t {
    rmw_qos_profile_t out{};
    out.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    out.depth = qos.depth();
    out.reliability = qos.reliability() == Qos::Reliability::RELIABLE ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                                                      : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    out.durability = qos.durability() == Qos::Durability::TRANSIENT_LOCAL ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                                                                          : RMW_QOS_POLICY_DURABILITY_VOLATILE;
    out.deadline.sec = qos.deadline().sec;
    out.deadline.nsec = qos.deadline().nsec;
    out.lifespan.sec = qos.lifespan().sec;
    out.lifespan.nsec = qos.lifespan().nsec;
    out.liveliness = qos.liveliness() == Qos::Liveliness::MANUAL_BY_TOPIC ? RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
                                                                          : RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    out.liveliness_lease_duration.sec = qos.liveliness_lease_duration().sec;
    out.liveliness_lease_duration.nsec = qos.liveliness_lease_duration().nsec;
    out.avoid_ros_namespace_conventions = qos.avoid_ros_namespace_conventions();
    return out;
}

auto TryConvert<Qos>::from(const rmw_qos_profile_t& profile, ProfileKind kind) -> Expected<Qos, QosError> {
    (void)kind;

    const bool has_unknown_policy = matching::is_unknown(profile.history) || matching::is_unknown(profile.reliability)
                                    || matching::is_unknown(profile.durability)
                                    || matching::is_unknown(profile.liveliness);
    if (has_unknown_policy) {
        RMW_IOX2_CHAIN_ERROR_MSG("QoS contains an UNKNOWN policy value");
        return err(QosError::UNKNOWN_POLICY);
    }

    if (profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        RMW_IOX2_CHAIN_ERROR_MSG("KEEP_ALL not supported; use KEEP_LAST with sufficient depth");
        return err(QosError::UNSUPPORTED_HISTORY_POLICY);
    }

    // History only resolves to KEEP_LAST (KEEP_ALL is rejected above).
    return Qos::Builder{}
        .set_history(Qos::History::KEEP_LAST, matching::resolve_depth(profile.depth))
        .set_reliability(matching::resolve(profile.reliability) == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                             ? Qos::Reliability::BEST_EFFORT
                             : Qos::Reliability::RELIABLE)
        .set_durability(matching::resolve(profile.durability) == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                            ? Qos::Durability::TRANSIENT_LOCAL
                            : Qos::Durability::VOLATILE)
        .set_deadline(to_duration(profile.deadline))
        .set_lifespan(to_duration(profile.lifespan))
        .set_liveliness(matching::resolve(profile.liveliness) == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
                            ? Qos::Liveliness::MANUAL_BY_TOPIC
                            : Qos::Liveliness::AUTOMATIC,
                        to_duration(profile.liveliness_lease_duration))
        .set_avoid_ros_namespace_conventions(profile.avoid_ros_namespace_conventions)
        .build();
}

auto TryConvert<Qos>::from(AttributeSetView attribute_set, ProfileKind kind) -> Expected<Qos, QosError> {
    (void)kind;

    Qos::Builder builder;

    auto fail = [](const char* key) -> Expected<Qos, QosError> {
        RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING("failed to decode attribute '%s'", key);
        return err(QosError::ATTRIBUTE_DECODING_FAILURE);
    };

    Optional<uint64_t> depth;
    attributes::visit_attribute_value(attribute_set, attributes::History::KEY, [&](const char* value) {
        depth = attributes::History::decode(value);
    });
    if (!depth.has_value()) {
        return fail(attributes::History::KEY);
    }
    builder.set_history(Qos::History::KEEP_LAST, depth.value());

    Optional<Qos::Reliability> reliability;
    attributes::visit_attribute_value(attribute_set, attributes::Reliability::KEY, [&](const char* value) {
        reliability = attributes::Reliability::decode(value);
    });
    if (!reliability.has_value()) {
        return fail(attributes::Reliability::KEY);
    }
    builder.set_reliability(reliability.value());

    Optional<Qos::Durability> durability;
    attributes::visit_attribute_value(attribute_set, attributes::Durability::KEY, [&](const char* value) {
        durability = attributes::Durability::decode(value);
    });
    if (!durability.has_value()) {
        return fail(attributes::Durability::KEY);
    }
    builder.set_durability(durability.value());

    Optional<Qos::Duration> deadline;
    attributes::visit_attribute_value(attribute_set, attributes::Deadline::KEY, [&](const char* value) {
        deadline = attributes::Deadline::decode(value);
    });
    if (!deadline.has_value()) {
        return fail(attributes::Deadline::KEY);
    }
    builder.set_deadline(deadline.value());

    Optional<Qos::Duration> lifespan;
    attributes::visit_attribute_value(attribute_set, attributes::Lifespan::KEY, [&](const char* value) {
        lifespan = attributes::Lifespan::decode(value);
    });
    if (!lifespan.has_value()) {
        return fail(attributes::Lifespan::KEY);
    }
    builder.set_lifespan(lifespan.value());

    Optional<attributes::Liveliness::Value> liveliness;
    attributes::visit_attribute_value(attribute_set, attributes::Liveliness::KEY, [&](const char* value) {
        liveliness = attributes::Liveliness::decode(value);
    });
    if (!liveliness.has_value()) {
        return fail(attributes::Liveliness::KEY);
    }
    builder.set_liveliness(liveliness.value().kind, liveliness.value().lease);

    return builder.build();
}

auto TryConvert<AttributeSpecifier>::from(const Qos& qos, const Optional<rosidl_type_hash_t>& type_hash)
    -> Expected<AttributeSpecifier, QosError> {
    AttributeSpecifier specifier;
    if (!set_qos_attributes(specifier, qos)) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to define one or more QoS attributes on AttributeSpecifier");
        return err(QosError::ATTRIBUTE_DEFINITION_FAILURE);
    }
    if (type_hash.has_value() && !set_type_hash_attribute(specifier, type_hash.value())) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to define type hash attribute on AttributeSpecifier");
        return err(QosError::ATTRIBUTE_DEFINITION_FAILURE);
    }
    return specifier;
}

auto TryConvert<AttributeVerifier>::from(const Qos& qos, const Optional<rosidl_type_hash_t>& type_hash)
    -> Expected<AttributeVerifier, QosError> {
    AttributeVerifier verifier;
    if (!set_qos_attributes(verifier, qos)) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to require one or more QoS attributes on AttributeVerifier");
        return err(QosError::ATTRIBUTE_DEFINITION_FAILURE);
    }
    if (type_hash.has_value() && !set_type_hash_attribute(verifier, type_hash.value())) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to require type hash attribute on AttributeVerifier");
        return err(QosError::ATTRIBUTE_DEFINITION_FAILURE);
    }
    return verifier;
}

} // namespace rmw::iox2
