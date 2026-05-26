// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_CONTEXT_HPP_
#define RMW_IOX2_RUNTIME_CONTEXT_HPP_

#include "iox2/bb/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"

#include <atomic>

class rmw_context_impl_s;

namespace rmw::iox2
{

template <>
struct Error<rmw_context_impl_s>
{
    using Type = ContextError;
};

/// Controls how `rmw_create_*` reconciles requested QoS with an existing
/// iceoryx2 service.
enum class QosMatchingMode : uint8_t {
    STRICT,   /// require attribute equality (default)
    ADOPTIVE, /// substitute caller QoS with existing service attributes
};

/// Defaults applied unless overriden by environment variable.
constexpr size_t DEFAULT_MAX_PUBLISHERS_PER_TOPIC = 32U;
constexpr size_t DEFAULT_MAX_SUBSCRIBERS_PER_TOPIC = 32U;
constexpr size_t DEFAULT_MAX_NODES_PER_SERVICE = 32U;

} // namespace rmw::iox2

extern "C" {

/// @brief iceoryx2-specific init options.
class RMW_PUBLIC rmw_init_options_impl_s
{
public:
    /// QoS reconciliation policy
    ::rmw::iox2::QosMatchingMode qos_matching_mode{::rmw::iox2::QosMatchingMode::STRICT};

    /// Upper bound on publishers per service. Configured via
    /// `RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC`. Falls back to
    /// `DEFAULT_MAX_PUBLISHERS_PER_TOPIC` when empty.
    ::iox2::bb::Optional<size_t> max_publishers_per_topic;

    /// Upper bound on subscribers per service. Configured via
    /// `RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC`. Falls back to
    /// `DEFAULT_MAX_SUBSCRIBERS_PER_TOPIC` when empty.
    ::iox2::bb::Optional<size_t> max_subscribers_per_topic;

    /// Upper bound on nodes per service. Configured via
    /// `RMW_IOX2_MAX_NODES_PER_SERVICE`. Falls back to
    /// `DEFAULT_MAX_NODES_PER_SERVICE` when empty.
    ::iox2::bb::Optional<size_t> max_nodes_per_service;
};

/// @brief Implementation of the RMW context for iceoryx2
/// @details The context manages the lifetime of entities used to implement guard conditions
class RMW_PUBLIC rmw_context_impl_s
{
    using CreationLock = ::rmw::iox2::CreationLock;
    using Iceoryx2 = ::rmw::iox2::Iceoryx2;

public:
    using ErrorType = ::rmw::iox2::Error<rmw_context_impl_s>::Type;

public:
    /// @brief Constructor for the iceoryx2 implementation of an RMW context
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] id ID to use for this context
    /// @param[in] options Snapshot of init-time options that downstream
    ///                    builders (Node, Publisher, Subscriber) consume
    rmw_context_impl_s(CreationLock lock,
                       ::iox2::bb::Optional<ErrorType>& error,
                       const uint32_t id,
                       const rmw_init_options_impl_s& options);

    // Move ops are required because `iox2::bb::Optional::emplace` uses
    // move-construct then move-assign internally.
    rmw_context_impl_s(rmw_context_impl_s&& other) noexcept;
    auto operator=(rmw_context_impl_s&& other) noexcept -> rmw_context_impl_s&;
    rmw_context_impl_s(const rmw_context_impl_s&) = delete;
    auto operator=(const rmw_context_impl_s&) -> rmw_context_impl_s& = delete;
    ~rmw_context_impl_s() = default;

    /// @brief Get the ID of this context
    /// @return The context ID
    auto id() -> uint32_t;

    /// @brief Get the handle to the underlying iceoryx runtime
    /// @return Reference to the iceoryx handle
    auto iox2() -> Iceoryx2&;

    /// @brief Get the init-time options snapshot
    /// @return Reference to the options captured at rmw_init
    auto options() const -> const rmw_init_options_impl_s&;

    /// @brief Generate a new unique identifier for a guard condition
    /// @return The generated guard condition ID
    auto generate_guard_condition_id() -> uint32_t;

private:
    // m_id is logically const after construction. The `const` qualifier is omitted
    // only because the explicit move-assignment operator needs to overwrite it.
    // Do not mutate.
    uint32_t m_id;
    ::iox2::bb::Optional<Iceoryx2> m_iox2;
    rmw_init_options_impl_s m_options;
    std::atomic<uint32_t> m_guard_condition_counter{0};
};
}

namespace rmw::iox2
{

// Used throughout implementation to indicate it is C++
using Context = rmw_context_impl_s;

} // namespace rmw::iox2

#endif
