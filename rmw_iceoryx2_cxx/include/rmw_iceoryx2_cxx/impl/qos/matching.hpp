// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_QOS_MATCHING_HPP_
#define RMW_IOX2_QOS_MATCHING_HPP_

#include "rmw/types.h"
#include "rmw/visibility_control.h"

#include <cstddef>
#include <cstdint>

/// QoS-policy classification and resolution.
///
/// Compatibility is evaluated on resolved values so the prediction matches
/// what endpoint creation does: two profiles whose indeterminate values
/// resolve to the same concrete values are compatible, even if those values
/// were `SYSTEM_DEFAULT` on input. `UNKNOWN` cannot be resolved thus is
/// a "cannot determine" condition.
namespace rmw::iox2
{

/// Selects which default profile is used when resolving `*_SYSTEM_DEFAULT`
/// for `rmw_qos_profile_t` inputs.
enum class ProfileKind : uint8_t { PUBLISH_SUBSCRIBE, SERVICE };

} // namespace rmw::iox2

namespace rmw::iox2::matching
{

/// Concrete `depth` substituted in place of `RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT`.
constexpr uint64_t DEFAULT_DEPTH = 10;

// ----------------------------------------------------------------------------
// Classification
// ----------------------------------------------------------------------------

/// `UNKNOWN` is opaque to this rmw and cannot be resolved.
RMW_PUBLIC auto is_unknown(rmw_qos_history_policy_t) -> bool;
RMW_PUBLIC auto is_unknown(rmw_qos_reliability_policy_t) -> bool;
RMW_PUBLIC auto is_unknown(rmw_qos_durability_policy_t) -> bool;
RMW_PUBLIC auto is_unknown(rmw_qos_liveliness_policy_t) -> bool;

/// True iff the time matches the `BEST_AVAILABLE`
RMW_PUBLIC auto is_best_available(rmw_time_t) -> bool;

// ----------------------------------------------------------------------------
// Resolution — collapse SYSTEM_DEFAULT / BEST_AVAILABLE to concrete values.
// UNKNOWN is returned unchanged.
// ----------------------------------------------------------------------------

RMW_PUBLIC auto resolve(rmw_qos_history_policy_t) -> rmw_qos_history_policy_t;
RMW_PUBLIC auto resolve_depth(size_t) -> size_t;
RMW_PUBLIC auto resolve(rmw_qos_reliability_policy_t) -> rmw_qos_reliability_policy_t;
RMW_PUBLIC auto resolve(rmw_qos_durability_policy_t) -> rmw_qos_durability_policy_t;
RMW_PUBLIC auto resolve(rmw_qos_liveliness_policy_t) -> rmw_qos_liveliness_policy_t;
RMW_PUBLIC auto resolve(rmw_time_t) -> rmw_time_t;

} // namespace rmw::iox2::matching

#endif // RMW_IOX2_QOS_MATCHING_HPP_
