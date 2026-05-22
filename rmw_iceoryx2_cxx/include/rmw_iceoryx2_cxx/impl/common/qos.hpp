// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_COMMON_QOS_HPP_
#define RMW_IOX2_COMMON_QOS_HPP_

#include "rmw/types.h"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/convert.hpp"

#include <cstdint>

namespace rmw::iox2
{

/// Selects the default profile used to resolve `*_SYSTEM_DEFAULT`.
enum class ProfileKind : uint8_t { PUBLISH_SUBSCRIBE, SERVICE };

/// Validated QoS used throughout the C++ runtime wrappers.
class RMW_PUBLIC Qos
{
public:
    enum class History : uint8_t { KEEP_LAST };
    enum class Reliability : uint8_t { RELIABLE, BEST_EFFORT };
    enum class Durability : uint8_t { VOLATILE, TRANSIENT_LOCAL };
    enum class Liveliness : uint8_t { AUTOMATIC, MANUAL_BY_TOPIC };

    // Cannot use iox2::bb::duration as its nanos are uint32_t
    // wheras RMW uses uint64_t
    struct Duration
    {
        uint64_t sec;
        uint64_t nsec;
    };

    class Builder
    {
    public:
        void set_history(History history, uint64_t depth) noexcept {
            m_history = history;
            m_depth = depth;
        }
        void set_reliability(Reliability reliability) noexcept {
            m_reliability = reliability;
        }
        void set_durability(Durability durability) noexcept {
            m_durability = durability;
        }
        void set_deadline(Duration deadline) noexcept {
            m_deadline = deadline;
        }
        void set_lifespan(Duration lifespan) noexcept {
            m_lifespan = lifespan;
        }
        void set_liveliness(Liveliness liveliness, Duration lease) noexcept {
            m_liveliness = liveliness;
            m_liveliness_lease_duration = lease;
        }
        void set_avoid_ros_namespace_conventions(bool avoid) noexcept {
            m_avoid_ros_namespace_conventions = avoid;
        }

        auto build() && noexcept -> Qos {
            Qos qos;
            qos.m_history = m_history;
            qos.m_depth = m_depth;
            qos.m_reliability = m_reliability;
            qos.m_durability = m_durability;
            qos.m_deadline = m_deadline;
            qos.m_lifespan = m_lifespan;
            qos.m_liveliness = m_liveliness;
            qos.m_liveliness_lease_duration = m_liveliness_lease_duration;
            qos.m_avoid_ros_namespace_conventions = m_avoid_ros_namespace_conventions;
            return qos;
        }

    private:
        History m_history{History::KEEP_LAST};
        uint64_t m_depth{10};
        Reliability m_reliability{Reliability::RELIABLE};
        Durability m_durability{Durability::VOLATILE};
        Duration m_deadline{0, 0};
        Duration m_lifespan{0, 0};
        Liveliness m_liveliness{Liveliness::AUTOMATIC};
        Duration m_liveliness_lease_duration{0, 0};
        bool m_avoid_ros_namespace_conventions{false};
    };

public:
    Qos(const Qos&) = default;
    Qos(Qos&&) noexcept = default;
    auto operator=(const Qos&) -> Qos& = default;
    auto operator=(Qos&&) noexcept -> Qos& = default;
    ~Qos() = default;

    // Policy accessors -----------------------------------------------

    auto history() const noexcept -> History {
        return m_history;
    }
    auto depth() const noexcept -> uint64_t {
        return m_depth;
    }
    auto reliability() const noexcept -> Reliability {
        return m_reliability;
    }
    auto durability() const noexcept -> Durability {
        return m_durability;
    }
    auto deadline() const noexcept -> Duration {
        return m_deadline;
    }
    auto lifespan() const noexcept -> Duration {
        return m_lifespan;
    }
    auto liveliness() const noexcept -> Liveliness {
        return m_liveliness;
    }
    auto liveliness_lease_duration() const noexcept -> Duration {
        return m_liveliness_lease_duration;
    }
    auto avoid_ros_namespace_conventions() const noexcept -> bool {
        return m_avoid_ros_namespace_conventions;
    }

    // Convert to iceoryx2 terminology----------------------------------

    auto history_size() const noexcept -> uint64_t {
        return m_depth;
    }
    auto subscriber_max_buffer_size() const noexcept -> uint64_t {
        return m_depth;
    }
    auto enable_safe_overflow() const noexcept -> bool {
        return m_reliability == Reliability::BEST_EFFORT;
    }

private:
    Qos() = default;

    History m_history{History::KEEP_LAST};
    uint64_t m_depth{10};
    Reliability m_reliability{Reliability::RELIABLE};
    Durability m_durability{Durability::VOLATILE};
    Duration m_deadline{0, 0};
    Duration m_lifespan{0, 0};
    Liveliness m_liveliness{Liveliness::AUTOMATIC};
    Duration m_liveliness_lease_duration{0, 0};
    bool m_avoid_ros_namespace_conventions{false};
};

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

/// Infallible conversion `Qos` → `rmw_qos_profile_t`.
/// Used at the C API boundary (e.g. `rmw_*_get_actual_qos`).
template <>
struct RMW_PUBLIC Convert<rmw_qos_profile_t>
{
    static auto from(const Qos& qos) noexcept -> rmw_qos_profile_t;
};

} // namespace rmw::iox2

#endif // RMW_IOX2_COMMON_QOS_HPP_
