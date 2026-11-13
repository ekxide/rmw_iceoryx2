// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_MIDDLEWARE_ICEORYX2_HPP_
#define RMW_IOX2_MIDDLEWARE_ICEORYX2_HPP_

#include "iox2/active_request.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/client.hpp"
#include "iox2/legacy/type_traits.hpp"
#include "iox2/listener.hpp"
#include "iox2/messaging_pattern.hpp"
#include "iox2/node.hpp"
#include "iox2/notifier.hpp"
#include "iox2/pending_response.hpp"
#include "iox2/port_factory_request_response.hpp"
#include "iox2/publisher.hpp"
#include "iox2/request_mut_uninit.hpp"
#include "iox2/response.hpp"
#include "iox2/response_mut_uninit.hpp"
#include "iox2/sample.hpp"
#include "iox2/sample_mut.hpp"
#include "iox2/sample_mut_uninit.hpp"
#include "iox2/server.hpp"
#include "iox2/service.hpp"
#include "iox2/service_builder.hpp"
#include "iox2/service_name.hpp"
#include "iox2/service_type.hpp"
#include "iox2/subscriber.hpp"
#include "iox2/waitset.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"

#include <string>

namespace rmw::iox2
{

class Iceoryx2;

template <>
struct Error<::rmw::iox2::Iceoryx2>
{
    using Type = HandleError;
};

/// Defaults applied unless overriden by environment variable.
constexpr size_t DEFAULT_MAX_NODES_PER_SERVICE = 32U;
constexpr size_t DEFAULT_MAX_SERVERS_PER_SERVICE = 4U;
constexpr size_t DEFAULT_MAX_CLIENTS_PER_SERVICE = 16U;
constexpr size_t DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT = 4U;
constexpr size_t MAX_RESPONSES_PER_REQUEST = 1U;

/// @brief Limits of an event service when it is created
class EventLimits
{
public:
    auto max_notifiers(uint64_t value) -> EventLimits& {
        m_max_notifiers = value;
        return *this;
    }
    auto max_listeners(uint64_t value) -> EventLimits& {
        m_max_listeners = value;
        return *this;
    }
    auto max_nodes(uint64_t value) -> EventLimits& {
        m_max_nodes = value;
        return *this;
    }

private:
    friend class Iceoryx2;
    uint64_t m_max_notifiers{0U};
    uint64_t m_max_listeners{0U};
    uint64_t m_max_nodes{0U};
};

/// @brief Limits of a request-response service when it is created
class RequestResponseLimits
{
public:
    auto max_servers(uint64_t value) -> RequestResponseLimits& {
        m_max_servers = value;
        return *this;
    }
    auto max_clients(uint64_t value) -> RequestResponseLimits& {
        m_max_clients = value;
        return *this;
    }
    auto max_active_requests_per_client(uint64_t value) -> RequestResponseLimits& {
        m_max_active_requests_per_client = value;
        return *this;
    }
    auto max_response_buffer_size(uint64_t value) -> RequestResponseLimits& {
        m_max_response_buffer_size = value;
        return *this;
    }
    auto max_nodes(uint64_t value) -> RequestResponseLimits& {
        m_max_nodes = value;
        return *this;
    }

private:
    friend class Iceoryx2;
    uint64_t m_max_servers{DEFAULT_MAX_SERVERS_PER_SERVICE};
    uint64_t m_max_clients{DEFAULT_MAX_CLIENTS_PER_SERVICE};
    uint64_t m_max_active_requests_per_client{DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT};
    uint64_t m_max_response_buffer_size{MAX_RESPONSES_PER_REQUEST};
    uint64_t m_max_nodes{DEFAULT_MAX_NODES_PER_SERVICE};
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
    template <typename T>
    using Error = ::rmw::iox2::Error<T>;
    using CreationLock = ::rmw::iox2::CreationLock;

public:
    // In iceoryx2, a node is an instance of iceoryx2 with its own lifetime management.
    using InstanceName = ::iox2::NodeName;
    using InstanceBuilder = ::iox2::NodeBuilder;

    using Config = ::iox2::Config;

    using ServiceType = ::iox2::ServiceType;
    using ServiceName = ::iox2::ServiceName;
    using MessagingPattern = ::iox2::MessagingPattern;
    using EventId = ::iox2::EventId;

    template <ServiceType S>
    using ServiceDetails = ::iox2::ServiceDetails<S>;

    struct Local
    {
        using Handle = ::iox2::Node<::iox2::ServiceType::Local>;
        using Notifier = ::iox2::Notifier<::iox2::ServiceType::Local>;
        using Listener = ::iox2::Listener<::iox2::ServiceType::Local>;
    };

    struct InterProcess
    {
        using Handle = ::iox2::Node<::iox2::ServiceType::Ipc>;
        using Service = ::iox2::Service<::iox2::ServiceType::Ipc>;
        using Notifier = ::iox2::Notifier<::iox2::ServiceType::Ipc>;
        using Listener = ::iox2::Listener<::iox2::ServiceType::Ipc>;

        template <typename Payload, typename UserHeader = void>
        using Sample = ::iox2::Sample<::iox2::ServiceType::Ipc, Payload, UserHeader>;
        template <typename Payload, typename UserHeader = void>
        using SampleMut = ::iox2::SampleMut<::iox2::ServiceType::Ipc, Payload, UserHeader>;
        template <typename Payload, typename UserHeader = void>
        using SampleMutUninit = ::iox2::SampleMutUninit<::iox2::ServiceType::Ipc, Payload, UserHeader>;
        template <typename Payload, typename UserHeader = void>
        using PublishSubscribe = ::iox2::PortFactoryPublishSubscribe<::iox2::ServiceType::Ipc, Payload, UserHeader>;
        template <typename Payload, typename UserHeader = void>
        using Publisher = ::iox2::Publisher<::iox2::ServiceType::Ipc, Payload, UserHeader>;
        template <typename Payload, typename UserHeader = void>
        using Subscriber = ::iox2::Subscriber<::iox2::ServiceType::Ipc, Payload, UserHeader>;

        template <typename Payload, typename UserHeader = void>
        static inline auto send = [](SampleMutUninit<Payload, UserHeader>&& sample) {
            return ::iox2::send(::iox2::assume_init(std::move(sample)));
        };

        template <typename Payload, typename UserHeader>
        using RequestResponseService =
            ::iox2::PortFactoryRequestResponse<::iox2::ServiceType::Ipc, Payload, UserHeader, Payload, UserHeader>;
        template <typename Payload, typename UserHeader>
        using Server = ::iox2::Server<::iox2::ServiceType::Ipc, Payload, UserHeader, Payload, UserHeader>;
        template <typename Payload, typename UserHeader>
        using Client = ::iox2::Client<::iox2::ServiceType::Ipc, Payload, UserHeader, Payload, UserHeader>;
        template <typename Payload, typename UserHeader>
        using ActiveRequest = ::iox2::ActiveRequest<::iox2::ServiceType::Ipc, Payload, UserHeader, Payload, UserHeader>;
        template <typename Payload, typename UserHeader>
        using PendingResponse =
            ::iox2::PendingResponse<::iox2::ServiceType::Ipc, Payload, UserHeader, Payload, UserHeader>;
        template <typename Payload, typename UserHeader>
        using RequestMutUninit =
            ::iox2::RequestMutUninit<::iox2::ServiceType::Ipc, Payload, UserHeader, Payload, UserHeader>;
        template <typename Payload, typename UserHeader>
        using Response = ::iox2::Response<::iox2::ServiceType::Ipc, Payload, UserHeader>;
        template <typename Payload, typename UserHeader>
        using ResponseMutUninit = ::iox2::ResponseMutUninit<::iox2::ServiceType::Ipc, Payload, UserHeader>;
    };

    struct WaitSet
    {
        using Handle = ::iox2::WaitSet<::iox2::ServiceType::Local>;
        using Guard = ::iox2::WaitSetGuard<::iox2::ServiceType::Local>;
        using AttachmentId = ::iox2::WaitSetAttachmentId<::iox2::ServiceType::Local>;

        static inline auto create = [](Local::Handle& node) {
            return node
                .waitset_builder()
                // delegate signal handling to rcl
                .signal_handling_mode(::iox2::SignalHandlingMode::Disabled)
                .create();
        };
    };

public:
    using ErrorType = Error<Iceoryx2>::Type;

public:
    /// @brief Creates an iceoryx2 instance with an independent lifetime to other instances
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] instance_name Unique name of the instance, used for book-keeping in iceoryx2
    /// @param[in] config iceoryx2 configuration of the instance
    Iceoryx2(CreationLock,
             ::iox2::bb::Optional<ErrorType>& error,
             const std::string& instance_name,
             const Config& config);

    /// @brief Access factory methods for creation of entities communicating locally.
    /// @return Factory to create IPC entities bound to the lifetime of this instance
    auto local() -> Local::Handle&;

    /// @brief Access factory methods for creation of entities communicating inter-process.
    /// @return Factory to create IPC entities bound to the lifetime of this instance
    auto ipc() -> InterProcess::Handle&;

    /// @brief Open an inter-process event service as it is, or create it with the given limits if it does not exist
    /// @param[in] service_name Name of the event service
    /// @param[in] limits Limits of the service when it is created
    /// @return The event service or the error of creating it
    auto open_or_create_event_service(const ServiceName& service_name, const EventLimits& limits)
        -> ::iox2::bb::Expected<::iox2::PortFactoryEvent<::iox2::ServiceType::Ipc>, ::iox2::EventOpenOrCreateError>;

    /// @brief Open an inter-process request-response service as it is, or create it with the given limits if it does
    /// not exist
    /// @param[in] service_name Name of the request-response service
    /// @param[in] request_type Type details of the request payload
    /// @param[in] response_type Type details of the response payload
    /// @param[in] limits Limits of the service when it is created
    /// @return The request-response service or the error of creating it
    template <typename Payload, typename UserHeader>
    auto open_or_create_request_response_service(const ServiceName& service_name,
                                                 const ::iox2::TypeDetail& request_type,
                                                 const ::iox2::TypeDetail& response_type,
                                                 const RequestResponseLimits& limits)
        -> ::iox2::bb::Expected<InterProcess::RequestResponseService<Payload, UserHeader>,
                                ::iox2::RequestResponseOpenOrCreateError>;

    /// @brief Creates a service builder for the specified service type and name
    /// @tparam ServiceType The type of service (Local or Ipc) to create
    /// @param[in] service_name Name of the service to create
    /// @return Service builder configured for the specified service type and name
    /// @note Local services can only communicate within the same process while IPC services can communicate across
    /// processes
    template <::iox2::ServiceType ServiceType>
    auto service_builder(const std::string& service_name) -> ::iox2::ServiceBuilder<ServiceType>;

    /// @brief Look up an existing service by name on this instance.
    /// @tparam S `Local` or `Ipc`; the corresponding handle's config is
    /// used for the lookup.
    /// @param[in] service_name Name of the service (e.g. produced by `names::topic`)
    /// @param[in] pattern Messaging pattern of the service to look up
    /// @return Service details if the service exists; `nullopt` when the
    /// name is malformed, the iceoryx2 registry cannot be read, or no
    /// service with that name exists.
    template <ServiceType S>
    auto lookup_service(const std::string& service_name, MessagingPattern pattern)
        -> ::iox2::bb::Optional<ServiceDetails<S>>;

private:
    ::iox2::bb::Optional<Local::Handle> m_local;
    ::iox2::bb::Optional<InterProcess::Handle> m_ipc;
};

// ===================================================================================================================

template <::iox2::ServiceType S>
auto Iceoryx2::service_builder(const std::string& service_name) -> ::iox2::ServiceBuilder<S> {
    auto name = ::iox2::ServiceName::create(service_name.c_str());
    if (!name.has_value()) {
        // TODO: propagate error
    }
    if constexpr (S == ::iox2::ServiceType::Local) {
        return local().service_builder(name.value());
    } else if constexpr (S == ::iox2::ServiceType::Ipc) {
        return ipc().service_builder(name.value());
    } else {
        static_assert(::iox2::legacy::always_false_v<decltype(S)>, "Attempted to build a service of unknown type");
    }
}

template <::iox2::ServiceType S>
auto Iceoryx2::lookup_service(const std::string& service_name, MessagingPattern pattern)
    -> ::iox2::bb::Optional<ServiceDetails<S>> {
    auto name = ::iox2::ServiceName::create(service_name.c_str());
    if (!name.has_value()) {
        return ::iox2::bb::NULLOPT;
    }
    auto details = [&]() {
        if constexpr (S == ::iox2::ServiceType::Local) {
            return ::iox2::Service<S>::details(name.value(), local().config(), pattern);
        } else if constexpr (S == ::iox2::ServiceType::Ipc) {
            return ::iox2::Service<S>::details(name.value(), ipc().config(), pattern);
        } else {
            static_assert(::iox2::legacy::always_false_v<decltype(S)>,
                          "Attempted to look up a service of unknown type");
        }
    }();
    if (!details.has_value()) {
        return ::iox2::bb::NULLOPT;
    }
    return std::move(details.value());
}

template <typename Payload, typename UserHeader>
auto Iceoryx2::open_or_create_request_response_service(const ServiceName& service_name,
                                                       const ::iox2::TypeDetail& request_type,
                                                       const ::iox2::TypeDetail& response_type,
                                                       const RequestResponseLimits& limits)
    -> ::iox2::bb::Expected<InterProcess::RequestResponseService<Payload, UserHeader>,
                            ::iox2::RequestResponseOpenOrCreateError> {
    constexpr uint32_t OPEN_ATTEMPTS = 2U;
    auto service_builder = [&]() {
        auto builder = ipc()
                           .service_builder(service_name)
                           .template request_response<Payload, Payload>()
                           .template request_user_header<UserHeader>()
                           .template response_user_header<UserHeader>();
        ::iox2::set_request_payload_type_details(builder, request_type);
        ::iox2::set_response_payload_type_details(builder, response_type);
        return builder;
    };
    for (uint32_t attempt = 1U;; ++attempt) {
        auto open_builder = service_builder();
        auto opened = open_builder.resume_build().open();
        if (opened.has_value()) {
            return std::move(opened.value());
        }
        auto create_builder = service_builder();
        auto created = create_builder.resume_build()
                           .max_servers(limits.m_max_servers)
                           .max_clients(limits.m_max_clients)
                           .max_nodes(limits.m_max_nodes)
                           .max_active_requests_per_client(limits.m_max_active_requests_per_client)
                           .max_response_buffer_size(limits.m_max_response_buffer_size)
                           .open_or_create();
        if (created.has_value()) {
            return created;
        }
        switch (created.error()) {
        case ::iox2::RequestResponseOpenOrCreateError::OpenDoesNotSupportRequestedAmountOfActiveRequestsPerClient:
        case ::iox2::RequestResponseOpenOrCreateError::OpenDoesNotSupportRequestedResponseBufferSize:
        case ::iox2::RequestResponseOpenOrCreateError::OpenDoesNotSupportRequestedAmountOfServers:
        case ::iox2::RequestResponseOpenOrCreateError::OpenDoesNotSupportRequestedAmountOfClients:
        case ::iox2::RequestResponseOpenOrCreateError::OpenDoesNotSupportRequestedAmountOfNodes:
            if (attempt < OPEN_ATTEMPTS) {
                continue;
            }
            break;
        default:
            break;
        }
        return created;
    }
}

} // namespace rmw::iox2

#endif
