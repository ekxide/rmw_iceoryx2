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
#include "iox2/type_variant.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/message/message_info_header.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/payload_layout.hpp"

namespace rmw::iox2
{

Client::Client(CreationLock,
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

    auto iox2_client = iox2_service.value()
                           .client_builder()
                           .initial_max_slice_len(::rmw::iox2::message_size(m_typesupport->request_typesupport))
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

} // namespace rmw::iox2
