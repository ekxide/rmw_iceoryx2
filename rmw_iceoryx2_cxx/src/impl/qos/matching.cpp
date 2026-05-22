// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/qos/matching.hpp"

namespace rmw::iox2::matching
{

namespace
{

constexpr rmw_time_t BEST_AVAILABLE_DURATION = RMW_QOS_DEADLINE_BEST_AVAILABLE;

} // namespace

auto is_unknown(rmw_qos_history_policy_t policy) -> bool {
    return policy == RMW_QOS_POLICY_HISTORY_UNKNOWN;
}

auto is_unknown(rmw_qos_reliability_policy_t policy) -> bool {
    return policy == RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
}

auto is_unknown(rmw_qos_durability_policy_t policy) -> bool {
    return policy == RMW_QOS_POLICY_DURABILITY_UNKNOWN;
}

auto is_unknown(rmw_qos_liveliness_policy_t policy) -> bool {
    return policy == RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
}

auto is_best_available(rmw_time_t time) -> bool {
    return time.sec == BEST_AVAILABLE_DURATION.sec && time.nsec == BEST_AVAILABLE_DURATION.nsec;
}

auto resolve(rmw_qos_history_policy_t policy) -> rmw_qos_history_policy_t {
    if (policy == RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT) {
        return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    }
    return policy;
}

auto resolve_depth(size_t depth) -> size_t {
    if (depth == RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT) {
        return DEFAULT_DEPTH;
    }
    return depth;
}

auto resolve(rmw_qos_reliability_policy_t policy) -> rmw_qos_reliability_policy_t {
    if (policy == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT || policy == RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE) {
        return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    }
    return policy;
}

auto resolve(rmw_qos_durability_policy_t policy) -> rmw_qos_durability_policy_t {
    if (policy == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT || policy == RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE) {
        return RMW_QOS_POLICY_DURABILITY_VOLATILE;
    }
    return policy;
}

auto resolve(rmw_qos_liveliness_policy_t policy) -> rmw_qos_liveliness_policy_t {
    if (policy == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT || policy == RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE) {
        return RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    }
    return policy;
}

auto resolve(rmw_time_t time) -> rmw_time_t {
    if (is_best_available(time)) {
        return {0U, 0U};
    }
    return time;
}

} // namespace rmw::iox2::matching
