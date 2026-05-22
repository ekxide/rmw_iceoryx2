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
#include "rmw_iceoryx2_cxx/impl/common/qos_matching.hpp"

#include <cstdio>

namespace rmw::iox2
{

namespace
{

namespace matching = ::rmw::iox2::matching;

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

auto TryConvert<Qos>::from(AttributeSetView attributes, ProfileKind kind) -> Expected<Qos, QosError> {
    (void)kind;

    Qos::Builder builder;
    char buf[256];

    auto fail = [](const char* key) -> Expected<Qos, QosError> {
        RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING("failed to decode attribute '%s'", key);
        return err(QosError::ATTRIBUTE_DECODING_FAILURE);
    };

    if (!read_attribute_value(attributes, codec::History::KEY, buf, sizeof(buf))) {
        return fail(codec::History::KEY);
    }
    auto depth = codec::History::parse(buf);
    if (!depth.has_value()) {
        return fail(codec::History::KEY);
    }
    builder.set_history(Qos::History::KEEP_LAST, depth.value());

    if (!read_attribute_value(attributes, codec::Reliability::KEY, buf, sizeof(buf))) {
        return fail(codec::Reliability::KEY);
    }
    auto reliability = codec::Reliability::parse(buf);
    if (!reliability.has_value()) {
        return fail(codec::Reliability::KEY);
    }
    builder.set_reliability(reliability.value());

    if (!read_attribute_value(attributes, codec::Durability::KEY, buf, sizeof(buf))) {
        return fail(codec::Durability::KEY);
    }
    auto durability = codec::Durability::parse(buf);
    if (!durability.has_value()) {
        return fail(codec::Durability::KEY);
    }
    builder.set_durability(durability.value());

    if (!read_attribute_value(attributes, codec::Deadline::KEY, buf, sizeof(buf))) {
        return fail(codec::Deadline::KEY);
    }
    auto deadline = codec::Deadline::parse(buf);
    if (!deadline.has_value()) {
        return fail(codec::Deadline::KEY);
    }
    builder.set_deadline(deadline.value());

    if (!read_attribute_value(attributes, codec::Lifespan::KEY, buf, sizeof(buf))) {
        return fail(codec::Lifespan::KEY);
    }
    auto lifespan = codec::Lifespan::parse(buf);
    if (!lifespan.has_value()) {
        return fail(codec::Lifespan::KEY);
    }
    builder.set_lifespan(lifespan.value());

    if (!read_attribute_value(attributes, codec::Liveliness::KEY, buf, sizeof(buf))) {
        return fail(codec::Liveliness::KEY);
    }
    auto liveliness = codec::Liveliness::parse(buf);
    if (!liveliness.has_value()) {
        return fail(codec::Liveliness::KEY);
    }
    builder.set_liveliness(liveliness.value().kind, liveliness.value().lease);

    return builder.build();
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

auto read_attribute_value(::iox2::AttributeSetView attributes, const char* key, char* out, size_t out_size) -> bool {
    auto key_obj = Attribute::Key::from_utf8_null_terminated_unchecked(key);
    if (!key_obj.has_value()) {
        return false;
    }
    auto val = attributes.key_value(key_obj.value(), 0);
    if (!val.has_value()) {
        return false;
    }
    // NOLINTNEXTLINE(cert-err33-c) source and destination both bounded by IOX2_ATTRIBUTE_VALUE_LENGTH
    std::snprintf(out, out_size, "%s", val.value().unchecked_access().c_str());
    return true;
}

} // namespace rmw::iox2
