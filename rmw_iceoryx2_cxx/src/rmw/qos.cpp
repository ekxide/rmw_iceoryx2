// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/qos_profiles.h"
#include "rmw/ret_types.h"
#include "rmw/types.h"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>

namespace
{

// Sentinel that means "match whatever other endpoints have" from rmw/types.h.
constexpr rmw_time_t BEST_AVAILABLE_DURATION = RMW_QOS_DEADLINE_BEST_AVAILABLE;

/// One entry in the compatibility-check result list.
struct Diagnostic
{
    enum class Kind : uint8_t { ERROR, WARNING };

    Kind kind;
    char reason[256];
};

/// Fixed-capacity accumulator for `Diagnostic` entries produced during a
/// `rmw_qos_profile_check_compatible` call.
class DiagnosticList
{
    // Upper bound on diagnostics we may produce: one per QoS field, at most.
    static constexpr size_t MAX_DIAGNOSTICS = 9;

public:
    void push_error(const char* text) {
        push(Diagnostic::Kind::ERROR, text);
    }
    void push_warning(const char* text) {
        push(Diagnostic::Kind::WARNING, text);
    }

    auto size() const -> size_t {
        return m_count;
    }
    auto operator[](size_t index) const -> const Diagnostic& {
        return m_entries[index];
    }

    auto has_error() const -> bool {
        for (size_t i = 0; i < m_count; ++i) {
            if (m_entries[i].kind == Diagnostic::Kind::ERROR) {
                return true;
            }
        }
        return false;
    }
    auto has_warning() const -> bool {
        for (size_t i = 0; i < m_count; ++i) {
            if (m_entries[i].kind == Diagnostic::Kind::WARNING) {
                return true;
            }
        }
        return false;
    }

private:
    void push(Diagnostic::Kind kind, const char* text) {
        if (m_count >= MAX_DIAGNOSTICS) {
            return;
        }
        m_entries[m_count].kind = kind;
        // NOLINTNEXTLINE(cert-err33-c) Diagnostic::text sized at 256 bytes; longer messages truncate gracefully
        std::snprintf(m_entries[m_count].reason, sizeof(m_entries[m_count].reason), "%s", text);
        ++m_count;
    }

    Diagnostic m_entries[MAX_DIAGNOSTICS];
    size_t m_count{0};
};

// Indeterminacy checks --------------------------------------------------------

auto is_indeterminate_history(rmw_qos_history_policy_t policy) -> bool {
    return policy == RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT || policy == RMW_QOS_POLICY_HISTORY_UNKNOWN;
}

auto is_indeterminate_depth(size_t depth) -> bool {
    return depth == RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT;
}

auto is_indeterminate_reliability(rmw_qos_reliability_policy_t policy) -> bool {
    return policy == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT || policy == RMW_QOS_POLICY_RELIABILITY_UNKNOWN
           || policy == RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE;
}

auto is_indeterminate_durability(rmw_qos_durability_policy_t policy) -> bool {
    return policy == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT || policy == RMW_QOS_POLICY_DURABILITY_UNKNOWN
           || policy == RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE;
}

auto is_indeterminate_liveliness(rmw_qos_liveliness_policy_t policy) -> bool {
    return policy == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT || policy == RMW_QOS_POLICY_LIVELINESS_UNKNOWN
           || policy == RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE;
}

auto is_best_available_time(rmw_time_t time) -> bool {
    return time.sec == BEST_AVAILABLE_DURATION.sec && time.nsec == BEST_AVAILABLE_DURATION.nsec;
}

auto durations_equal(rmw_time_t lhs, rmw_time_t rhs) -> bool {
    return lhs.sec == rhs.sec && lhs.nsec == rhs.nsec;
}

// Compatibility checks ----------------------------------------------

void check_history(const rmw_qos_profile_t& pub, const rmw_qos_profile_t& sub, DiagnosticList& diagnostics) {
    if (pub.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL || sub.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        diagnostics.push_error("history KEEP_ALL is not supported by the iceoryx2 transport");
        return;
    }
    if (is_indeterminate_history(pub.history) || is_indeterminate_history(sub.history)) {
        diagnostics.push_warning("history policy is SYSTEM_DEFAULT or UNKNOWN; compatibility cannot be determined");
        return;
    }
    if (pub.history != sub.history) {
        diagnostics.push_error("history policy differs between publisher and subscription");
    }
}

void check_depth(const rmw_qos_profile_t& pub, const rmw_qos_profile_t& sub, DiagnosticList& diagnostics) {
    if (is_indeterminate_depth(pub.depth) || is_indeterminate_depth(sub.depth)) {
        diagnostics.push_warning("depth is SYSTEM_DEFAULT; compatibility cannot be determined");
        return;
    }
    if (pub.depth != sub.depth) {
        diagnostics.push_error("depth differs between publisher and subscription");
    }
}

void check_reliability(const rmw_qos_profile_t& pub, const rmw_qos_profile_t& sub, DiagnosticList& diagnostics) {
    if (is_indeterminate_reliability(pub.reliability) || is_indeterminate_reliability(sub.reliability)) {
        diagnostics.push_warning(
            "reliability is SYSTEM_DEFAULT / UNKNOWN / BEST_AVAILABLE; compatibility cannot be determined");
        return;
    }
    if (pub.reliability != sub.reliability) {
        diagnostics.push_error("reliability differs between publisher and subscription");
    }
}

void check_durability(const rmw_qos_profile_t& pub, const rmw_qos_profile_t& sub, DiagnosticList& diagnostics) {
    if (is_indeterminate_durability(pub.durability) || is_indeterminate_durability(sub.durability)) {
        diagnostics.push_warning(
            "durability is SYSTEM_DEFAULT / UNKNOWN / BEST_AVAILABLE; compatibility cannot be determined");
        return;
    }
    if (pub.durability != sub.durability) {
        diagnostics.push_error("durability differs between publisher and subscription");
    }
}

void check_deadline(const rmw_qos_profile_t& pub, const rmw_qos_profile_t& sub, DiagnosticList& diagnostics) {
    if (is_best_available_time(pub.deadline) || is_best_available_time(sub.deadline)) {
        diagnostics.push_warning("deadline is BEST_AVAILABLE; compatibility cannot be determined");
        return;
    }
    if (!durations_equal(pub.deadline, sub.deadline)) {
        diagnostics.push_error("deadline differs between publisher and subscription");
    }
}

void check_lifespan(const rmw_qos_profile_t& pub, const rmw_qos_profile_t& sub, DiagnosticList& diagnostics) {
    if (is_best_available_time(pub.lifespan) || is_best_available_time(sub.lifespan)) {
        diagnostics.push_warning("lifespan is BEST_AVAILABLE; compatibility cannot be determined");
        return;
    }
    if (!durations_equal(pub.lifespan, sub.lifespan)) {
        diagnostics.push_error("lifespan differs between publisher and subscription");
    }
}

void check_liveliness(const rmw_qos_profile_t& pub, const rmw_qos_profile_t& sub, DiagnosticList& diagnostics) {
    if (is_indeterminate_liveliness(pub.liveliness) || is_indeterminate_liveliness(sub.liveliness)) {
        diagnostics.push_warning(
            "liveliness is SYSTEM_DEFAULT / UNKNOWN / BEST_AVAILABLE; compatibility cannot be determined");
        return;
    }
    if (pub.liveliness != sub.liveliness) {
        diagnostics.push_error("liveliness differs between publisher and subscription");
    }
}

void check_liveliness_lease_duration(const rmw_qos_profile_t& publisher_profile,
                                     const rmw_qos_profile_t& subscriber_profile,
                                     DiagnosticList& diagnostics) {
    if (is_best_available_time(publisher_profile.liveliness_lease_duration)
        || is_best_available_time(subscriber_profile.liveliness_lease_duration)) {
        diagnostics.push_warning("liveliness_lease_duration is BEST_AVAILABLE; compatibility cannot be determined");
        return;
    }
    if (!durations_equal(publisher_profile.liveliness_lease_duration, subscriber_profile.liveliness_lease_duration)) {
        diagnostics.push_error("liveliness_lease_duration differs between publisher and subscription");
    }
}

void check_avoid_ros_namespace_conventions(const rmw_qos_profile_t& publisher_profile,
                                           const rmw_qos_profile_t& subscriber_profile,
                                           DiagnosticList& diagnostics) {
    if (publisher_profile.avoid_ros_namespace_conventions != subscriber_profile.avoid_ros_namespace_conventions) {
        diagnostics.push_error("avoid_ros_namespace_conventions differs between publisher and subscription");
    }
}

// Reason ----------------------------------------------------------------------

void write_reason(const DiagnosticList& diagnostics, char* reason, size_t reason_size) {
    if (reason == nullptr || reason_size == 0) {
        return;
    }
    size_t offset = 0;
    auto append = [&](const char* text) {
        if (offset >= reason_size) {
            return;
        }
        // NOLINTNEXTLINE(cert-err33-c) reason buffer is caller-sized; truncation only shortens the explanation
        int written = std::snprintf(reason + offset, reason_size - offset, "%s%s", offset == 0 ? "" : "; ", text);
        if (written > 0) {
            offset = std::min(offset + static_cast<size_t>(written), reason_size - 1);
        }
    };

    // Errors first, then warnings.
    for (size_t i = 0; i < diagnostics.size(); ++i) {
        if (diagnostics[i].kind == Diagnostic::Kind::ERROR) {
            append(diagnostics[i].reason);
        }
    }
    for (size_t i = 0; i < diagnostics.size(); ++i) {
        if (diagnostics[i].kind == Diagnostic::Kind::WARNING) {
            append(diagnostics[i].reason);
        }
    }
}

} // namespace

extern "C" {
rmw_ret_t rmw_qos_profile_check_compatible(const rmw_qos_profile_t publisher_profile,
                                           const rmw_qos_profile_t subscription_profile,
                                           rmw_qos_compatibility_type_t* compatibility,
                                           char* reason,
                                           size_t reason_size) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(compatibility, RMW_RET_INVALID_ARGUMENT);
    if (reason == nullptr && reason_size != 0) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    DiagnosticList diagnostics;
    check_history(publisher_profile, subscription_profile, diagnostics);
    check_depth(publisher_profile, subscription_profile, diagnostics);
    check_reliability(publisher_profile, subscription_profile, diagnostics);
    check_durability(publisher_profile, subscription_profile, diagnostics);
    check_deadline(publisher_profile, subscription_profile, diagnostics);
    check_lifespan(publisher_profile, subscription_profile, diagnostics);
    check_liveliness(publisher_profile, subscription_profile, diagnostics);
    check_liveliness_lease_duration(publisher_profile, subscription_profile, diagnostics);
    check_avoid_ros_namespace_conventions(publisher_profile, subscription_profile, diagnostics);

    if (diagnostics.has_error()) {
        *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    } else if (diagnostics.has_warning()) {
        *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
    } else {
        *compatibility = RMW_QOS_COMPATIBILITY_OK;
        // Un-terminated char array leads to crashes in rqt_graph.
        if (reason != nullptr && reason_size > 0) {
            reason[0] = '\0';
        }
        return RMW_RET_OK;
    }

    write_reason(diagnostics, reason, reason_size);
    return RMW_RET_OK;
}
}
