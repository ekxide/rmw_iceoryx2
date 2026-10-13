// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rcutils/time.h"
#include "rmw/allocators.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw/validate_full_topic_name.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/server.hpp"
#include "rmw_iceoryx2_cxx/rmw/node.hpp"

extern "C" {

rmw_service_t* rmw_create_service(const rmw_node_t* rmw_node,
                                  const rosidl_service_type_support_t* type_support,
                                  const char* service_name,
                                  const rmw_qos_profile_t* qos) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(type_support, nullptr);
    RMW_IOX2_ENSURE_VALID_SERVICE_TYPESUPPORT(type_support, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(service_name, nullptr);
    RMW_IOX2_ENSURE_VALID_SERVICE_NAME(service_name, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(qos, nullptr);
    RMW_IOX2_ENSURE_VALID_QOS(qos, nullptr);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::allocate_copy;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ::rmw::iox2::NodeData;
    using ServerImpl = ::rmw::iox2::Server;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Creating service '%s'", service_name);

    auto rmw_service = rmw_service_allocate();
    if (rmw_service == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_service_t");
        return nullptr;
    }
    rmw_service->implementation_identifier = rmw_get_implementation_identifier();

    if (auto ptr = allocate_copy(service_name); !ptr.has_value()) {
        rmw_service_free(rmw_service);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for service name");
        return nullptr;
    } else {
        rmw_service->service_name = ptr.value();
    }

    auto node_data = unsafe_cast<NodeData*>(rmw_node->data);
    if (!node_data.has_value()) {
        deallocate(rmw_service->service_name);
        rmw_service_free(rmw_service);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Node");
        return nullptr;
    }

    if (auto server_impl = allocate<ServerImpl>(); !server_impl.has_value()) {
        deallocate(rmw_service->service_name);
        rmw_service_free(rmw_service);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for Server");
        return nullptr;
    } else {
        if (auto result = create_in_place<ServerImpl>(
                server_impl.value(), node_data.value()->node.value(), service_name, type_support, *qos);
            !result.has_value()) {
            destruct<ServerImpl>(server_impl.value());
            deallocate<ServerImpl>(server_impl.value());
            deallocate(rmw_service->service_name);
            rmw_service_free(rmw_service);
            RMW_IOX2_CHAIN_ERROR_MSG("failed to construct Server");
            return nullptr;
        }
        rmw_service->data = server_impl.value();
    }

    return rmw_service;
}

rmw_ret_t rmw_destroy_service(rmw_node_t* rmw_node, rmw_service_t* rmw_service) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_service, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_service->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ServerImpl = ::rmw::iox2::Server;

    RMW_IOX2_LOG_DEBUG("Destroying service '%s'", rmw_service->service_name);

    if (rmw_service->data) {
        destruct<ServerImpl>(rmw_service->data);
        deallocate(rmw_service->data);
    }
    if (rmw_service->service_name != nullptr) {
        deallocate(rmw_service->service_name);
    }
    rmw_service_free(rmw_service);

    return RMW_RET_OK;
}

rmw_ret_t
rmw_take_request(const rmw_service_t* rmw_service, rmw_service_info_t* request_header, void* ros_request, bool* taken) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_service, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_service->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ServerImpl = ::rmw::iox2::Server;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Taking request from '%s'", rmw_service->service_name);

    auto server_impl = unsafe_cast<ServerImpl*>(rmw_service->data);
    if (!server_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Server");
        return RMW_RET_ERROR;
    }

    auto request = server_impl.value()->take_request();
    if (!request.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to take request");
        return RMW_RET_ERROR;
    }
    *taken = request.value().has_value();
    if (!*taken) {
        return RMW_RET_OK;
    }

    auto& loan = request.value().value();
    auto serialized_message = rmw_serialized_message_t{
        loan.bytes, loan.number_of_bytes, loan.number_of_bytes, rcutils_get_default_allocator()};
    if (auto result =
            rmw_deserialize(&serialized_message, server_impl.value()->typesupport()->request_typesupport, ros_request);
        result != RMW_RET_OK) {
        server_impl.value()->discard_request(loan.client_id, loan.message_info.publication_sequence_number);
        *taken = false;
        RMW_IOX2_CHAIN_ERROR_MSG("failed to deserialize received request");
        return RMW_RET_ERROR;
    }

    rcutils_time_point_value_t received = 0;
    if (rcutils_system_time_now(&received) != RCUTILS_RET_OK) {
        received = 0;
    }
    request_header->source_timestamp = loan.message_info.source_timestamp;
    request_header->received_timestamp = received;
    request_header->request_id.sequence_number = static_cast<int64_t>(loan.message_info.publication_sequence_number);
    std::copy(loan.client_id.begin(), loan.client_id.end(), request_header->request_id.writer_guid);

    return RMW_RET_OK;
}

rmw_ret_t rmw_send_response(const rmw_service_t* rmw_service, rmw_request_id_t* request_header, void* ros_response) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_service, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_service->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ServerImpl = ::rmw::iox2::Server;
    using ::rmw::iox2::ClientId;
    using ::rmw::iox2::serialized_message_size;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Sending response from '%s'", rmw_service->service_name);

    auto server_impl = unsafe_cast<ServerImpl*>(rmw_service->data);
    if (!server_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Server");
        return RMW_RET_ERROR;
    }

    ClientId client_id{};
    std::copy(request_header->writer_guid, request_header->writer_guid + client_id.size(), client_id.begin());
    auto type_support = server_impl.value()->typesupport()->response_typesupport;
    auto serialized_size = serialized_message_size(ros_response, type_support);

    auto loan = server_impl.value()->loan_response(
        client_id, static_cast<uint64_t>(request_header->sequence_number), serialized_size);
    if (!loan.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to loan bytes required for serialization");
        return RMW_RET_ERROR;
    }
    if (!loan.value().has_value()) {
        return RMW_RET_OK;
    }

    auto serialized_message = rmw_serialized_message_t{reinterpret_cast<uint8_t*>(loan.value().value()),
                                                       serialized_size,
                                                       serialized_size,
                                                       rcutils_get_default_allocator()};
    if (auto result = rmw_serialize(ros_response, type_support, &serialized_message); result != RMW_RET_OK) {
        (void)server_impl.value()->return_response_loan(loan.value().value());
        RMW_IOX2_CHAIN_ERROR_MSG("failed to serialize into loaned payload");
        return RMW_RET_ERROR;
    }
    if (auto result = server_impl.value()->send_response(loan.value().value()); !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to send response");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_service_request_subscription_get_actual_qos(const rmw_service_t* rmw_service, rmw_qos_profile_t* qos) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_service, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_service->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ServerImpl = ::rmw::iox2::Server;
    using ::rmw::iox2::unsafe_cast;

    auto server_impl = unsafe_cast<ServerImpl*>(rmw_service->data);
    if (!server_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Server");
        return RMW_RET_ERROR;
    }

    *qos = server_impl.value()->qos();

    return RMW_RET_OK;
}

rmw_ret_t rmw_service_response_publisher_get_actual_qos(const rmw_service_t* rmw_service, rmw_qos_profile_t* qos) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_service, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_service->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ServerImpl = ::rmw::iox2::Server;
    using ::rmw::iox2::unsafe_cast;

    auto server_impl = unsafe_cast<ServerImpl*>(rmw_service->data);
    if (!server_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Server");
        return RMW_RET_ERROR;
    }

    *qos = server_impl.value()->qos();

    return RMW_RET_OK;
}

rmw_ret_t rmw_service_set_on_new_request_callback(rmw_service_t* rmw_service,
                                                  rmw_event_callback_t callback,
                                                  const void* user_data) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_service, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_service->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(user_data, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}
}
