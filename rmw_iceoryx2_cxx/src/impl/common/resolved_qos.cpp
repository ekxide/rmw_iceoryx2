// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/common/resolved_qos.hpp"

#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"

#include <utility>

namespace rmw::iox2
{

namespace
{

using ::iox2::bb::err;
using ::iox2::bb::Expected;

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

} // namespace

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

auto Convert<rmw_qos_profile_t>::from(const ResolvedQos& qos) noexcept -> rmw_qos_profile_t {
    rmw_qos_profile_t out{};
    out.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    out.depth = qos.depth();
    out.reliability = qos.reliability() == ResolvedQos::Reliability::RELIABLE ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                                                              : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    out.durability = qos.durability() == ResolvedQos::Durability::TRANSIENT_LOCAL
                         ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                         : RMW_QOS_POLICY_DURABILITY_VOLATILE;
    out.deadline.sec = qos.deadline().sec;
    out.deadline.nsec = qos.deadline().nsec;
    out.lifespan.sec = qos.lifespan().sec;
    out.lifespan.nsec = qos.lifespan().nsec;
    out.liveliness = qos.liveliness() == ResolvedQos::Liveliness::MANUAL_BY_TOPIC
                         ? RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
                         : RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    out.liveliness_lease_duration.sec = qos.liveliness_lease_duration().sec;
    out.liveliness_lease_duration.nsec = qos.liveliness_lease_duration().nsec;
    out.avoid_ros_namespace_conventions = qos.avoid_ros_namespace_conventions();
    return out;
}

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

} // namespace rmw::iox2
