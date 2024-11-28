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

#include "iox2/listener.hpp"
#include "iox2/node.hpp"
#include "iox2/notifier.hpp"
#include "iox2/publisher.hpp"
#include "iox2/sample.hpp"
#include "iox2/sample_mut.hpp"
#include "iox2/sample_mut_uninit.hpp"
#include "iox2/service_builder.hpp"
#include "iox2/service_type.hpp"
#include "iox2/subscriber.hpp"
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
    // In iceoryx2, a node is an instance of iceoryx2 with its own lifetime management.
    using InstanceName = ::iox2::NodeName;
    using InstanceBuilder = ::iox2::NodeBuilder;

    using Config = ::iox2::Config;

    using ServiceType = ::iox2::ServiceType;
    using ServiceName = ::iox2::ServiceName;
    using EventId = ::iox2::EventId;

    struct Local
    {
        using Handle = ::iox2::Node<::iox2::ServiceType::Local>;
        using Notifier = ::iox2::Notifier<::iox2::ServiceType::Local>;
        using Listener = ::iox2::Listener<::iox2::ServiceType::Local>;
    };

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

        template <typename Payload>
        static inline auto send =
            [](SampleMutUninit<Payload>&& sample) { return ::iox2::send(::iox2::assume_init(std::move(sample))); };
    };

    struct WaitSet
    {
        // NOTE: Service type is inconsequential. Will be removed from iceoryx2 soon.
        using Handle = ::iox2::WaitSet<::iox2::ServiceType::Local>;
        using Guard = ::iox2::WaitSetGuard<::iox2::ServiceType::Local>;
        using AttachmentId = ::iox2::WaitSetAttachmentId<::iox2::ServiceType::Local>;

        static inline auto create = []() {
            return ::iox2::WaitSetBuilder().template create<::iox2::ServiceType::Local>();
        };
    };

public:
    using ErrorType = Error<Iceoryx2>::Type;

public:
    /// @brief Creates an iceoryx2 instance with an independent lifetime to other instances
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] instance_name Unique name of the instance, used for book-keeping in iceoryx2
    Iceoryx2(CreationLock, iox::optional<ErrorType>& error, const std::string& instance_name);

    /// @brief Access factory methods for creation of entities communicating locally.
    /// @return Factory to create IPC entities bound to the lifetime of this instance
    auto local() -> Local::Handle&;

    /// @brief Access factory methods for creation of entities communicating inter-process.
    /// @return Factory to create IPC entities bound to the lifetime of this instance
    auto ipc() -> InterProcess::Handle&;

    /// @brief Creates a service builder for the specified service type and name
    /// @tparam ServiceType The type of service (Local or Ipc) to create
    /// @param[in] service_name Name of the service to create
    /// @return Service builder configured for the specified service type and name
    /// @note Local services can only communicate within the same process while IPC services can communicate across
    /// processes
    template <::iox2::ServiceType ServiceType>
    auto service_builder(const std::string& service_name) -> ::iox2::ServiceBuilder<ServiceType>;

private:
    iox::optional<Local::Handle> m_local;
    iox::optional<InterProcess::Handle> m_ipc;
};

// ===================================================================================================================

template <::iox2::ServiceType ServiceType>
auto Iceoryx2::service_builder(const std::string& service_name) -> ::iox2::ServiceBuilder<ServiceType> {
    auto name = ::iox2::ServiceName::create(service_name.c_str());
    if (name.has_error()) {
        // TODO: propagate error
    }
    if constexpr (ServiceType == ::iox2::ServiceType::Local) {
        return local().service_builder(name.value());
    } else if constexpr (ServiceType == ::iox2::ServiceType::Ipc) {
        return ipc().service_builder(name.value());
    } else {
        static_assert(std::false_type::value, "Attempted to build a service of unknown type");
    }
}

} // namespace rmw::iox2

#endif
