// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/client.hpp"
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
#include "rmw_iceoryx2_cxx/rmw/node.hpp"

extern "C" {

rmw_client_t* rmw_create_client(const rmw_node_t* rmw_node,
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
    using ClientImpl = ::rmw::iox2::Client;
    using ::rmw::iox2::NodeData;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Creating client to '%s'", service_name);

    auto rmw_client = rmw_client_allocate();
    if (rmw_client == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_client_t");
        return nullptr;
    }
    rmw_client->implementation_identifier = rmw_get_implementation_identifier();

    if (auto ptr = allocate_copy(service_name); !ptr.has_value()) {
        rmw_client_free(rmw_client);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for service name");
        return nullptr;
    } else {
        rmw_client->service_name = ptr.value();
    }

    auto node_data = unsafe_cast<NodeData*>(rmw_node->data);
    if (!node_data.has_value()) {
        deallocate(rmw_client->service_name);
        rmw_client_free(rmw_client);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Node");
        return nullptr;
    }

    if (auto client_impl = allocate<ClientImpl>(); !client_impl.has_value()) {
        deallocate(rmw_client->service_name);
        rmw_client_free(rmw_client);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for Client");
        return nullptr;
    } else {
        if (auto result = create_in_place<ClientImpl>(
                client_impl.value(), node_data.value()->node.value(), service_name, type_support, *qos);
            !result.has_value()) {
            destruct<ClientImpl>(client_impl.value());
            deallocate<ClientImpl>(client_impl.value());
            deallocate(rmw_client->service_name);
            rmw_client_free(rmw_client);
            RMW_IOX2_CHAIN_ERROR_MSG("failed to construct Client");
            return nullptr;
        }
        rmw_client->data = client_impl.value();
    }

    return rmw_client;
}

rmw_ret_t rmw_destroy_client(rmw_node_t* rmw_node, rmw_client_t* rmw_client) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_client->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ClientImpl = ::rmw::iox2::Client;

    RMW_IOX2_LOG_DEBUG("Destroying client to '%s'", rmw_client->service_name);

    if (rmw_client->data) {
        destruct<ClientImpl>(rmw_client->data);
        deallocate(rmw_client->data);
    }
    if (rmw_client->service_name != nullptr) {
        deallocate(rmw_client->service_name);
    }
    rmw_client_free(rmw_client);

    return RMW_RET_OK;
}

rmw_ret_t rmw_send_request(const rmw_client_t* rmw_client, const void* ros_request, int64_t* sequence_id) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_client->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ClientImpl = ::rmw::iox2::Client;
    using ::rmw::iox2::serialized_message_size;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Sending request to '%s'", rmw_client->service_name);

    auto client_impl = unsafe_cast<ClientImpl*>(rmw_client->data);
    if (!client_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Client");
        return RMW_RET_ERROR;
    }

    auto type_support = client_impl.value()->typesupport()->request_typesupport;
    auto serialized_size = serialized_message_size(ros_request, type_support);

    auto loan = client_impl.value()->loan_request(serialized_size);
    if (!loan.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to loan bytes required for serialization");
        return RMW_RET_ERROR;
    }

    auto serialized_message = rmw_serialized_message_t{
        reinterpret_cast<uint8_t*>(loan.value()), serialized_size, serialized_size, rcutils_get_default_allocator()};
    if (auto result = rmw_serialize(ros_request, type_support, &serialized_message); result != RMW_RET_OK) {
        (void)client_impl.value()->return_request_loan(loan.value());
        RMW_IOX2_CHAIN_ERROR_MSG("failed to serialize into loaned payload");
        return RMW_RET_ERROR;
    }

    auto sequence_number = client_impl.value()->send_request(loan.value());
    if (!sequence_number.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to send request");
        return RMW_RET_ERROR;
    }
    *sequence_id = static_cast<int64_t>(sequence_number.value());

    return RMW_RET_OK;
}

rmw_ret_t
rmw_take_response(const rmw_client_t* rmw_client, rmw_service_info_t* request_header, void* ros_response, bool* taken) {
    RMW_IOX2_ENSURE_NOT_NULL(rmw_client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_client->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_client_request_publisher_get_actual_qos(const rmw_client_t* rmw_client, rmw_qos_profile_t* qos) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_client->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ClientImpl = ::rmw::iox2::Client;
    using ::rmw::iox2::unsafe_cast;

    auto client_impl = unsafe_cast<ClientImpl*>(rmw_client->data);
    if (!client_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Client");
        return RMW_RET_ERROR;
    }

    *qos = client_impl.value()->qos();

    return RMW_RET_OK;
}

rmw_ret_t rmw_client_response_subscription_get_actual_qos(const rmw_client_t* rmw_client, rmw_qos_profile_t* qos) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_client->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ClientImpl = ::rmw::iox2::Client;
    using ::rmw::iox2::unsafe_cast;

    auto client_impl = unsafe_cast<ClientImpl*>(rmw_client->data);
    if (!client_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Client");
        return RMW_RET_ERROR;
    }

    *qos = client_impl.value()->qos();

    return RMW_RET_OK;
}

rmw_ret_t rmw_client_set_on_new_response_callback(rmw_client_t* rmw_client,
                                                  rmw_event_callback_t callback,
                                                  const void* user_data) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_client->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(user_data, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_service_server_is_available(const rmw_node_t* rmw_node, const rmw_client_t* rmw_client, bool* is_available) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_client->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(is_available, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ClientImpl = ::rmw::iox2::Client;
    using ::rmw::iox2::unsafe_cast;

    auto client_impl = unsafe_cast<ClientImpl*>(rmw_client->data);
    if (!client_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Client");
        return RMW_RET_ERROR;
    }

    *is_available = client_impl.value()->is_server_available();

    return RMW_RET_OK;
}
}
