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
#include "iox2/waitset.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"

#include <string>

namespace rmw::iox2
{

class Iceoryx2;

template <>
struct Error<Iceoryx2>
{
    using Type = HandleError;
};

/// @brief A handle to iceoryx2 for creating and managing the lifetimes of iceoryx2 entities.
/// @details An iceoryx2 handle is responsible for managing the lifetime of iceoryx2 entities that it it creates.
///
/// Each unique iceoryx2 instance is responsible for creation of iceoryx2 entities and provides a decoupled lifetime for
/// these entities. Destruction of an iceoryx2 instance ends the lifetime of all entities it created, thus the handle
/// must outlive them. Individual entities can end their own lifetime independently without affecting other entities
/// bound to the handle's lifetime via destruction.
///
/// @note The component for creating iceoryx2 entities is called a 'Node', however to avoid confusion with
///       RMW nodes, it is referred to in this codebase as an "IceoryxHandle".
///
/// In the RMW the lifetime of entities are tied to either the RMW context or an RMW node, thus these RMW entities
/// shall have their own iceoryx2 instance.
///
class RMW_PUBLIC Iceoryx2
{
public:
    using Local = ::iox2::Node<::iox2::ServiceType::Local>;

    struct InterProcess
    {
        using Handle = ::iox2::Node<::iox2::ServiceType::Ipc>;

        using Notifier = ::iox2::Notifier<::iox2::ServiceType::Ipc>;
        using Listener = ::iox2::Listener<::iox2::ServiceType::Ipc>;

        template <typename Payload>
        using Sample = ::iox2::Sample<::iox2::ServiceType::Ipc, Payload, void>;
        template <typename Payload>
        using SampleMut = ::iox2::SampleMut<::iox2::ServiceType::Ipc, Payload, void>;
        template <typename Payload>
        using SampleMutUninit = ::iox2::SampleMutUninit<::iox2::ServiceType::Ipc, Payload, void>;
        template <typename Payload>
        using Publisher = ::iox2::Publisher<::iox2::ServiceType::Ipc, Payload, void>;
        template <typename Payload>
        using Subscriber = ::iox2::Subscriber<::iox2::ServiceType::Ipc, Payload, void>;
    };

    struct WaitSet
    {
        using Builder = ::iox2::WaitSetBuilder;
        using Handle = ::iox2::WaitSet<::iox2::ServiceType::Ipc>;
        using Guard = ::iox2::WaitSetGuard<::iox2::ServiceType::Ipc>;
        using AttachmentId = ::iox2::WaitSetAttachmentId<::iox2::ServiceType::Ipc>;
    };

public:
    using ErrorType = Error<Iceoryx2>::Type;

public:
    /// @brief Creates an iceoryx2 instance with an independent lifetime to other instances
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] instance_name Unique name of the instance, used for book-keeping in iceoryx2
    Iceoryx2(CreationLock, iox::optional<ErrorType>& error, const std::string& instance_name);

    /// @brief Access factory methods for creation of IPC entities
    /// @return Factory to create IPC entities bound to the lifetime of this instance
    auto ipc() -> InterProcess::Handle&;

private:
    iox::optional<InterProcess::Handle> m_ipc;
};

} // namespace rmw::iox2

#endif
