// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/client.hpp"

#include "iox2/bb/into.hpp"
#include "rcutils/time.h"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/message/message_info_header.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/payload_layout.hpp"

#include <algorithm>

namespace rmw::iox2
{

Client::Client(CreationLock,
               ::iox2::bb::Optional<ErrorType>& error,
               Node& node,
               const char* service,
               const rosidl_service_type_support_t* type_support,
               const rmw_qos_profile_t& qos)
    : m_callback_thread{node.context().callback_thread()}
    , m_service{service}
    , m_typesupport{type_support}
    , m_service_name{::rmw::iox2::names::service(service)}
    , m_qos{qos}
    , m_unserialized_request_size{::rmw::iox2::message_size(type_support->request_typesupport)}
    , m_is_request_self_contained{::rmw::iox2::is_self_contained(type_support->request_typesupport)}
    , m_is_response_self_contained{::rmw::iox2::is_self_contained(type_support->response_typesupport)} {
    auto iox2_service_name = Iceoryx2::ServiceName::create(m_service_name.c_str());
    if (!iox2_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service_name.error()));
        error.emplace(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        return;
    }

    const auto& options = node.context().options();

    auto service_builder = node.iox2()
                               .ipc()
                               .service_builder(iox2_service_name.value())
                               .request_response<Payload, Payload>()
                               .request_user_header<UserHeader>()
                               .response_user_header<UserHeader>();

    ::iox2::set_request_payload_type_details(service_builder,
                                             ::rmw::iox2::payload_type_details(m_typesupport->request_typesupport));
    ::iox2::set_response_payload_type_details(service_builder,
                                              ::rmw::iox2::payload_type_details(m_typesupport->response_typesupport));

    auto iox2_service = service_builder.resume_build()
                            .max_servers(DEFAULT_MAX_SERVERS_PER_SERVICE)
                            .max_clients(DEFAULT_MAX_CLIENTS_PER_SERVICE)
                            .max_nodes(options.max_nodes_per_service.value_or(DEFAULT_MAX_NODES_PER_SERVICE))
                            .max_active_requests_per_client(DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT)
                            .max_response_buffer_size(MAX_RESPONSES_PER_REQUEST)
                            .open_or_create();

    if (!iox2_service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto iox2_client = iox2_service.value()
                           .client_builder()
                           .initial_max_slice_len(m_is_request_self_contained ? SELF_CONTAINED_PAYLOAD_ELEMENT_COUNT
                                                                              : m_unserialized_request_size)
                           .allocation_strategy(::iox2::AllocationStrategy::PowerOfTwo)
                           .backpressure_strategy(::iox2::BackpressureStrategy::DiscardData)
                           .create();

    if (!iox2_client.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_client.error()));
        error.emplace(ErrorType::CLIENT_CREATION_FAILURE);
        return;
    }

    m_iox2_unique_id.emplace(iox2_client->id());
    m_iox2_client.emplace(std::move(iox2_client.value()));
    m_iox2_service.emplace(std::move(iox2_service.value()));

    auto iox2_event_service =
        node.iox2().open_or_create_event_service(iox2_service_name.value(),
                                                 DEFAULT_MAX_SERVERS_AND_CLIENTS_PER_SERVICE,
                                                 DEFAULT_MAX_SERVERS_AND_CLIENTS_PER_SERVICE,
                                                 options.max_nodes_per_service.value_or(DEFAULT_MAX_NODES_PER_SERVICE));
    if (!iox2_event_service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_event_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto iox2_notifier = iox2_event_service.value().notifier_builder().create();
    if (!iox2_notifier.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_notifier.error()));
        error.emplace(ErrorType::NOTIFIER_CREATION_FAILURE);
        return;
    }
    m_iox2_notifier.emplace(std::move(iox2_notifier.value()));

    auto iox2_listener = iox2_event_service.value().listener_builder().create();
    if (!iox2_listener.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_listener.error()));
        error.emplace(ErrorType::LISTENER_CREATION_FAILURE);
        return;
    }
    m_iox2_listener.emplace(std::move(iox2_listener.value()));
}

auto Client::listener() -> IceoryxListener& {
    return m_iox2_listener.value();
}

auto Client::set_callback(rmw_event_callback_t callback, const void* user_data)
    -> ::iox2::bb::Expected<void, CallbackThread::ErrorType> {
    using ::iox2::bb::err;

    std::lock_guard<std::mutex> lock{m_callback_mutex};
    m_callback.reset();
    if (callback == nullptr) {
        return {};
    }
    auto added = m_callback_thread->add(m_iox2_listener.value(), callback, user_data);
    if (!added.has_value()) {
        return err(added.error());
    }
    m_callback.emplace(std::move(added.value()));
    return {};
}

auto Client::unique_id() -> const ::iox2::bb::Optional<RawIdType>& {
    auto& bytes = m_iox2_unique_id->bytes();
    return bytes;
}

auto Client::service() const -> const std::string& {
    return m_service;
}

auto Client::typesupport() const -> const rosidl_service_type_support_t* {
    return m_typesupport;
}

auto Client::service_name() const -> const std::string& {
    return m_service_name;
}

auto Client::qos() const -> const rmw_qos_profile_t& {
    return m_qos;
}

auto Client::is_server_available() const -> bool {
    return m_iox2_service->dynamic_config().number_of_servers() > 0;
}

auto Client::is_request_self_contained() const -> bool {
    return m_is_request_self_contained;
}

auto Client::unserialized_request_size() const -> uint64_t {
    return m_unserialized_request_size;
}

auto Client::is_response_self_contained() const -> bool {
    return m_is_response_self_contained;
}

auto Client::loan_request(uint64_t number_of_bytes) -> ::iox2::bb::Expected<void*, ErrorType> {
    using ::iox2::bb::err;

    std::lock_guard<std::mutex> lock{m_mutex};

    const uint64_t number_of_elements =
        m_is_request_self_contained ? SELF_CONTAINED_PAYLOAD_ELEMENT_COUNT : number_of_bytes;
    auto request = m_iox2_client->loan_slice_uninit(number_of_elements);
    if (!request.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(request.error()));
        return err(ErrorType::LOAN_FAILURE);
    }

    return static_cast<void*>(m_requests.store(std::move(request.value())));
}

auto Client::return_request_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    std::lock_guard<std::mutex> lock{m_mutex};

    if (auto result = m_requests.release(static_cast<uint8_t*>(loaned_memory)); !result.has_value()) {
        return err(ErrorType::INVALID_PAYLOAD);
    }
    return {};
}

auto Client::send_request(void* loaned_memory) -> ::iox2::bb::Expected<uint64_t, ErrorType> {
    using ::iox2::bb::err;

    std::lock_guard<std::mutex> lock{m_mutex};

    // Requests of servers that are gone can no longer be responded to, unless they already were.
    for (auto it = m_pending_responses.begin(); it != m_pending_responses.end();) {
        const bool answerable = it->second.is_connected() || it->second.has_response();
        it = answerable ? std::next(it) : m_pending_responses.erase(it);
    }

    auto request = m_requests.release(static_cast<uint8_t*>(loaned_memory));
    if (!request.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("invalid payload pointer");
        return err(ErrorType::INVALID_PAYLOAD);
    }

    rcutils_time_point_value_t now = 0;
    if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
        now = 0;
    }
    auto sequence_number = ++m_sequence_number;
    request->user_header_mut().source_timestamp = now;
    request->user_header_mut().publication_sequence_number = sequence_number;

    auto pending_response = ::iox2::send(::iox2::assume_init(std::move(request.value())));
    if (!pending_response.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(pending_response.error()));
        return err(ErrorType::SEND_FAILURE);
    }

    if (pending_response->number_of_server_connections() > 0) {
        m_pending_responses.emplace(sequence_number, std::move(pending_response.value()));
    }

    if (auto notified = m_iox2_notifier->notify(); !notified.has_value()) {
        log_notify_error(notified.error(), "request to", m_service);
    }

    return sequence_number;
}

auto Client::has_responses() -> bool {
    std::lock_guard<std::mutex> lock{m_mutex};

    return std::any_of(m_pending_responses.begin(), m_pending_responses.end(), [](auto& pending_response) {
        return pending_response.second.has_response();
    });
}

auto Client::take_response() -> ::iox2::bb::Expected<::iox2::bb::Optional<ClientResponse>, ErrorType> {
    using ::iox2::bb::err;
    using ::iox2::bb::Optional;

    std::lock_guard<std::mutex> lock{m_mutex};

    for (auto it = m_pending_responses.begin(); it != m_pending_responses.end();) {
        auto result = it->second.receive();
        if (!result.has_value()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
            if (result.error() != ::iox2::ReceiveError::ExceedsMaxBorrows) {
                m_pending_responses.erase(it);
            }
            return err(ErrorType::RECV_FAILURE);
        }
        auto response = std::move(result.value());
        if (response.has_value()) {
            m_pending_responses.erase(it);

            auto* bytes = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(response->payload().data()));
            auto number_of_bytes = response->payload().number_of_bytes();
            auto message_info = response->user_header();
            m_responses.store(std::move(response.value()));

            return Optional<ClientResponse>(ClientResponse{bytes, number_of_bytes, message_info});
        }
        it = it->second.is_connected() ? std::next(it) : m_pending_responses.erase(it);
    }

    return Optional<ClientResponse>{::iox2::bb::NULLOPT};
}

auto Client::return_response_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    std::lock_guard<std::mutex> lock{m_mutex};

    if (auto result = m_responses.release(static_cast<uint8_t*>(loaned_memory)); !result.has_value()) {
        return err(ErrorType::INVALID_PAYLOAD);
    }
    return {};
}

} // namespace rmw::iox2
