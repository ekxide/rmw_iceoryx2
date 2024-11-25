// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_HANDLE_HPP_
#define RMW_IOX2_HANDLE_HPP_

#include "iox/assertions.hpp"
#include "iox2/node.hpp"
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"

#include <string>

namespace rmw::iox2
{

class IceoryxHandle;

template <>
struct Error<IceoryxHandle>
{
    using Type = HandleError;
};

/// @brief Handle to iceoryx2 for managing the lifetime of iceoryx2 entities
/// @details An iceoryx2 handle is responsible for managing the lifetime of iceoryx2 entities that it it creates.
///
/// Each unique iceoryx2 provides a decoupled lifetime for created entities.
/// Destruction of the handle ends the lifetime of all entities it created, thus the handle must
/// outlive them.
/// Individual entities can end their own lifetime independently without affecting other entities bound to the
/// handle's lifetime via destruction.
///
/// @note The entity for creating iceoryx2 entities is called a 'Node', however to avoid confusion with
///          RMW nodes, it is referred to in this codebase as a "Handle".
///
/// In the RMW the lifetime of entities are tied to either the RMW context or an RMW node.
///
class RMW_PUBLIC IceoryxHandle
{
    using Impl = ::iox2::Node<::iox2::ServiceType::Ipc>;

public:
    using ErrorType = Error<IceoryxHandle>::Type;

public:
    IceoryxHandle(CreationLock, iox::optional<ErrorType>& error, const std::string& instance_name);

    /// @brief Dereference operator to access the underlying IceoryxHandle
    /// @return Reference to the underlying IceoryxHandle
    /// @throws std::runtime_error if m_handle is empty
    Impl& operator*() {
        if (!m_impl.has_value()) {
            IOX_PANIC("Corrupted IceoryxHandle instance");
        }
        return m_impl.value();
    }

    /// @brief Arrow operator to access members of the underlying IceoryxHandle
    /// @return Pointer to the underlying IceoryxHandle
    /// @throws std::runtime_error if m_handle is empty
    Impl* operator->() {
        if (!m_impl.has_value()) {
            IOX_PANIC("Corrupted IceoryxHandle instance");
        }
        return &m_impl.value();
    }

private:
    iox::optional<Impl> m_impl;
};

} // namespace rmw::iox2

#endif
