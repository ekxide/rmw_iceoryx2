// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/common/resolved_qos.hpp"

namespace rmw::iox2
{

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

} // namespace rmw::iox2
