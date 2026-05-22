// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/common/qos_diagnostics.hpp"

#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"
#include "rmw_iceoryx2_cxx/impl/common/qos_attributes.hpp"

#include <cstdio>
#include <cstring>

namespace rmw::iox2
{

namespace
{

auto is_default_duration(Qos::Duration duration) -> bool {
    return duration.sec == 0U && duration.nsec == 0U;
}

} // namespace

void log_unsupported_policies(const Qos& qos, const char* topic) noexcept {
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

    if (qos.liveliness() == Qos::Liveliness::MANUAL_BY_TOPIC) {
        RMW_IOX2_LOG_WARN(
            "QoS policy 'liveliness' (=manual_by_topic) on topic '%s' is not honored by the iceoryx2 transport", topic);
    }
}

void log_attribute_mismatch(const Qos& requested, ::iox2::AttributeSetView attrs, const char* topic) noexcept {
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
