// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_CONTEXT_IMPL_HPP_
#define RMW_IOX2_CONTEXT_IMPL_HPP_

#include "iox/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/iceoryx2.hpp"

#include <atomic>

class rmw_context_impl_s;

namespace rmw::iox2
{

template <>
struct Error<rmw_context_impl_s>
{
    using Type = ContextError;
};

} // namespace rmw::iox2

extern "C" {

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
    rmw_context_impl_s(CreationLock lock, iox::optional<ErrorType>& error, const uint32_t id);

    /// @brief Get the ID of this context
    /// @return The context ID
    auto id() -> uint32_t;

    /// @brief Get the handle to the underlying iceoryx runtime
    /// @return Reference to the iceoryx handle
    auto iox2() -> Iceoryx2&;

    /// @brief Generate a new unique identifier for a guard condition
    /// @return The generated guard condition ID
    auto generate_guard_condition_id() -> uint32_t;

private:
    const uint32_t m_id;
    iox::optional<Iceoryx2> m_iox2;
    std::atomic<uint32_t> m_guard_condition_counter{0};
};
}

namespace rmw::iox2
{

// Used throughout implementation to indicate it is C++
using ContextImpl = rmw_context_impl_s;

} // namespace rmw::iox2

#endif
