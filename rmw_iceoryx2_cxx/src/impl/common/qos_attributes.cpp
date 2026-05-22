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
#include "rmw_iceoryx2_cxx/impl/common/qos_codec.hpp"

#include <cstdio>
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
using ::iox2::bb::Optional;

// Sentinel that means "match whatever other endpoints have" from rmw/types.h.
constexpr rmw_time_t BEST_AVAILABLE_DURATION = RMW_QOS_DEADLINE_BEST_AVAILABLE;

constexpr uint64_t DEFAULT_DEPTH = 10;

auto map_time(rmw_time_t time) -> Qos::Duration {
    // BEST_AVAILABLE sentinel collapses to the canonical default (0:0).
    if (time.sec == BEST_AVAILABLE_DURATION.sec && time.nsec == BEST_AVAILABLE_DURATION.nsec) {
        return {0U, 0U};
    }
    return {time.sec, time.nsec};
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
auto set_qos_attributes(Target& target, const Qos& qos) -> bool {
    char buf[256];
    codec::History::format(qos, buf, sizeof(buf));
    if (!write_attribute(target, codec::History::KEY, buf)) {
        return false;
    }
    codec::Reliability::format(qos, buf, sizeof(buf));
    if (!write_attribute(target, codec::Reliability::KEY, buf)) {
        return false;
    }
    codec::Durability::format(qos, buf, sizeof(buf));
    if (!write_attribute(target, codec::Durability::KEY, buf)) {
        return false;
    }
    codec::Deadline::format(qos, buf, sizeof(buf));
    if (!write_attribute(target, codec::Deadline::KEY, buf)) {
        return false;
    }
    codec::Lifespan::format(qos, buf, sizeof(buf));
    if (!write_attribute(target, codec::Lifespan::KEY, buf)) {
        return false;
    }
    codec::Liveliness::format(qos, buf, sizeof(buf));
    if (!write_attribute(target, codec::Liveliness::KEY, buf)) {
        return false;
    }
    return true;
}

} // namespace

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

auto TryConvert<Qos>::from(const rmw_qos_profile_t& profile, ProfileKind kind) -> Expected<Qos, QosError> {
    (void)kind;

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

    Qos::Builder builder;

    builder.set_history(Qos::History::KEEP_LAST,
                        profile.depth == RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT ? DEFAULT_DEPTH : profile.depth);

    switch (profile.reliability) {
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
        builder.set_reliability(Qos::Reliability::BEST_EFFORT);
        break;
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE:
    default:
        builder.set_reliability(Qos::Reliability::RELIABLE);
        break;
    }

    switch (profile.durability) {
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
        builder.set_durability(Qos::Durability::TRANSIENT_LOCAL);
        break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE:
    default:
        builder.set_durability(Qos::Durability::VOLATILE);
        break;
    }

    builder.set_deadline(map_time(profile.deadline));
    builder.set_lifespan(map_time(profile.lifespan));

    auto lease = map_time(profile.liveliness_lease_duration);
    switch (profile.liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
        builder.set_liveliness(Qos::Liveliness::MANUAL_BY_TOPIC, lease);
        break;
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
    case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE:
    default:
        builder.set_liveliness(Qos::Liveliness::AUTOMATIC, lease);
        break;
    }

    builder.set_avoid_ros_namespace_conventions(profile.avoid_ros_namespace_conventions);

    return std::move(builder).build();
}

auto TryConvert<Qos>::from(AttributeSetView attrs, ProfileKind kind) -> Expected<Qos, QosError> {
    (void)kind;

    Qos::Builder builder;
    char buf[256];

    auto fail = [](const char* key) -> Expected<Qos, QosError> {
        RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING("failed to decode attribute '%s'", key);
        return err(QosError::ATTRIBUTE_DECODING_FAILURE);
    };

    if (!read_attribute_value(attrs, codec::History::KEY, buf, sizeof(buf))) {
        return fail(codec::History::KEY);
    }
    auto depth = codec::History::parse(buf);
    if (!depth.has_value()) {
        return fail(codec::History::KEY);
    }
    builder.set_history(Qos::History::KEEP_LAST, depth.value());

    if (!read_attribute_value(attrs, codec::Reliability::KEY, buf, sizeof(buf))) {
        return fail(codec::Reliability::KEY);
    }
    auto reliability = codec::Reliability::parse(buf);
    if (!reliability.has_value()) {
        return fail(codec::Reliability::KEY);
    }
    builder.set_reliability(reliability.value());

    if (!read_attribute_value(attrs, codec::Durability::KEY, buf, sizeof(buf))) {
        return fail(codec::Durability::KEY);
    }
    auto durability = codec::Durability::parse(buf);
    if (!durability.has_value()) {
        return fail(codec::Durability::KEY);
    }
    builder.set_durability(durability.value());

    if (!read_attribute_value(attrs, codec::Deadline::KEY, buf, sizeof(buf))) {
        return fail(codec::Deadline::KEY);
    }
    auto deadline = codec::Deadline::parse(buf);
    if (!deadline.has_value()) {
        return fail(codec::Deadline::KEY);
    }
    builder.set_deadline(deadline.value());

    if (!read_attribute_value(attrs, codec::Lifespan::KEY, buf, sizeof(buf))) {
        return fail(codec::Lifespan::KEY);
    }
    auto lifespan = codec::Lifespan::parse(buf);
    if (!lifespan.has_value()) {
        return fail(codec::Lifespan::KEY);
    }
    builder.set_lifespan(lifespan.value());

    if (!read_attribute_value(attrs, codec::Liveliness::KEY, buf, sizeof(buf))) {
        return fail(codec::Liveliness::KEY);
    }
    auto liveliness = codec::Liveliness::parse(buf);
    if (!liveliness.has_value()) {
        return fail(codec::Liveliness::KEY);
    }
    builder.set_liveliness(liveliness.value().kind, liveliness.value().lease);

    return std::move(builder).build();
}

auto TryConvert<AttributeSpecifier>::from(const Qos& qos) -> Expected<AttributeSpecifier, QosError> {
    AttributeSpecifier specifier;
    if (!set_qos_attributes(specifier, qos)) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to define one or more QoS attributes on AttributeSpecifier");
        return err(QosError::ATTRIBUTE_DEFINITION_FAILURE);
    }
    return specifier;
}

auto TryConvert<AttributeVerifier>::from(const Qos& qos) -> Expected<AttributeVerifier, QosError> {
    AttributeVerifier verifier;
    if (!set_qos_attributes(verifier, qos)) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to require one or more QoS attributes on AttributeVerifier");
        return err(QosError::ATTRIBUTE_DEFINITION_FAILURE);
    }
    return verifier;
}

// ----------------------------------------------------------------------------
// Attribute helpers
// ----------------------------------------------------------------------------

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

} // namespace rmw::iox2
