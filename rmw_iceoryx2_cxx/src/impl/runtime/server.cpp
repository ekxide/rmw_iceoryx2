// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/server.hpp"

#include "iox2/bb/into.hpp"
#include "iox2/type_variant.hpp"
#include "rcutils/time.h"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/message/message_info_header.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/payload_layout.hpp"

#include <algorithm>

namespace rmw::iox2
{

Server::Server(CreationLock,
               ::iox2::bb::Optional<ErrorType>& error,
               Node& node,
               const char* service,
               const rosidl_service_type_support_t* type_support,
               const rmw_qos_profile_t& qos)
    : m_service{service}
    , m_typesupport{type_support}
    , m_service_name{::rmw::iox2::names::service(service)}
    , m_qos{qos} {
    auto iox2_service_name = Iceoryx2::ServiceName::create(m_service_name.c_str());
    if (!iox2_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service_name.error()));
        error.emplace(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        return;
    }

    const auto& options = node.context().options();
    const auto request_type_name = ::rmw::iox2::message_type_name(m_typesupport->request_typesupport);
    const auto response_type_name = ::rmw::iox2::message_type_name(m_typesupport->response_typesupport);

    auto service_builder = node.iox2()
                               .ipc()
                               .service_builder(iox2_service_name.value())
                               .request_response<Payload, Payload>()
                               .request_user_header<UserHeader>()
                               .response_user_header<UserHeader>();

    ::iox2::set_request_payload_type_details(service_builder,
                                             ::iox2::TypeDetail(::iox2::TypeVariant::Dynamic,
                                                                request_type_name.c_str(),
                                                                SERIALIZED_PAYLOAD_ELEMENT_SIZE,
                                                                SERIALIZED_PAYLOAD_ALIGNMENT));
    ::iox2::set_response_payload_type_details(service_builder,
                                              ::iox2::TypeDetail(::iox2::TypeVariant::Dynamic,
                                                                 response_type_name.c_str(),
                                                                 SERIALIZED_PAYLOAD_ELEMENT_SIZE,
                                                                 SERIALIZED_PAYLOAD_ALIGNMENT));

    auto iox2_service = service_builder.resume_build()
                            .max_servers(DEFAULT_MAX_SERVERS_PER_SERVICE)
                            .max_clients(DEFAULT_MAX_CLIENTS_PER_SERVICE)
                            .max_nodes(options.max_nodes_per_service.value_or(DEFAULT_MAX_NODES_PER_SERVICE))
                            .max_active_requests_per_client(DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT)
                            .max_response_buffer_size(MAX_RESPONSES_PER_REQUEST)
                            .request_payload_alignment(SERIALIZED_PAYLOAD_ALIGNMENT)
                            .response_payload_alignment(SERIALIZED_PAYLOAD_ALIGNMENT)
                            .open_or_create();

    if (!iox2_service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto iox2_server = iox2_service.value()
                           .server_builder()
                           .initial_max_slice_len(::rmw::iox2::message_size(m_typesupport->response_typesupport))
                           .allocation_strategy(::iox2::AllocationStrategy::PowerOfTwo)
                           .backpressure_strategy(::iox2::BackpressureStrategy::DiscardData)
                           .create();

    if (!iox2_server.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_server.error()));
        error.emplace(ErrorType::SERVER_CREATION_FAILURE);
        return;
    }

    m_iox2_server.emplace(std::move(iox2_server.value()));
}

auto Server::service() const -> const std::string& {
    return m_service;
}

auto Server::typesupport() const -> const rosidl_service_type_support_t* {
    return m_typesupport;
}

auto Server::service_name() const -> const std::string& {
    return m_service_name;
}

auto Server::qos() const -> const rmw_qos_profile_t& {
    return m_qos;
}

auto Server::take_request() -> ::iox2::bb::Expected<::iox2::bb::Optional<ServerRequest>, ErrorType> {
    using ::iox2::bb::err;
    using ::iox2::bb::Optional;

    std::lock_guard<std::mutex> lock{m_mutex};

    auto result = m_iox2_server->receive();
    if (!result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::RECV_FAILURE);
    }
    auto request = std::move(result.value());
    if (!request.has_value()) {
        return Optional<ServerRequest>{::iox2::bb::NULLOPT};
    }

    auto origin = request->origin();
    const auto& origin_bytes = origin.bytes();
    if (!origin_bytes.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("unable to retrieve UniquePortId of the requesting client");
        return err(ErrorType::RECV_FAILURE);
    }
    ClientId client_id{};
    std::copy_n(origin_bytes.value().unchecked_access().data(), client_id.size(), client_id.begin());

    // reinterpret_cast to obtain a byte pointer from the CustomPayloadMarker element type;
    // const_cast required because of the RMW API.
    auto* bytes = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(request->payload().data()));
    auto number_of_bytes = request->payload().number_of_bytes();
    auto message_info = request->user_header();

    m_active_requests.insert_or_assign(RequestId{client_id, message_info.publication_sequence_number},
                                       std::move(request.value()));

    return Optional<ServerRequest>(ServerRequest{bytes, number_of_bytes, message_info, client_id});
}

auto Server::discard_request(const ClientId& client_id, uint64_t sequence_number) -> void {
    std::lock_guard<std::mutex> lock{m_mutex};

    m_active_requests.erase(RequestId{client_id, sequence_number});
}

auto Server::loan_response(const ClientId& client_id, uint64_t sequence_number, uint64_t number_of_bytes)
    -> ::iox2::bb::Expected<::iox2::bb::Optional<void*>, ErrorType> {
    using ::iox2::bb::err;
    using ::iox2::bb::Optional;

    std::lock_guard<std::mutex> lock{m_mutex};

    const auto request_id = RequestId{client_id, sequence_number};
    auto active_request = m_active_requests.find(request_id);
    if (active_request == m_active_requests.end()) {
        RMW_IOX2_CHAIN_ERROR_MSG("no taken request matches the request id");
        return err(ErrorType::UNKNOWN_REQUEST);
    }
    if (!active_request->second.is_connected()) {
        m_active_requests.erase(active_request);
        return Optional<void*>{::iox2::bb::NULLOPT};
    }

    auto response = active_request->second.loan_slice_uninit(number_of_bytes);
    if (!response.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(response.error()));
        return err(ErrorType::LOAN_FAILURE);
    }

    auto* ptr = m_responses.store(std::move(response.value()));
    m_response_requests.emplace(ptr, request_id);

    return Optional<void*>(static_cast<void*>(ptr));
}

auto Server::return_response_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    std::lock_guard<std::mutex> lock{m_mutex};

    if (auto result = m_responses.release(static_cast<uint8_t*>(loaned_memory)); !result.has_value()) {
        return err(ErrorType::INVALID_PAYLOAD);
    }
    m_response_requests.erase(static_cast<uint8_t*>(loaned_memory));

    return {};
}

auto Server::send_response(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    std::lock_guard<std::mutex> lock{m_mutex};

    auto response = m_responses.release(static_cast<uint8_t*>(loaned_memory));
    if (!response.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("invalid payload pointer");
        return err(ErrorType::INVALID_PAYLOAD);
    }
    auto request_id = m_response_requests.find(static_cast<uint8_t*>(loaned_memory));

    rcutils_time_point_value_t now = 0;
    if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
        now = 0;
    }
    response->user_header_mut().source_timestamp = now;
    response->user_header_mut().publication_sequence_number = request_id->second.second;

    auto result = ::iox2::send(::iox2::assume_init(std::move(response.value())));
    m_active_requests.erase(request_id->second);
    m_response_requests.erase(request_id);
    if (!result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::SEND_FAILURE);
    }

    return {};
}

} // namespace rmw::iox2
