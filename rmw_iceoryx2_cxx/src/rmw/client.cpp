// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "rmw/allocators.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/allocator.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"

extern "C" {

rmw_client_t* rmw_create_client(const rmw_node_t* node,
                                const rosidl_service_type_support_t* type_support,
                                const char* service_name,
                                const rmw_qos_profile_t* qos_policies) {
    using ::rmw::iox2::allocate_copy;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);

    /* NOT SUPPORTED (YET) */

    auto client = rmw_client_allocate();
    if (client == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_publisher_t");
        return nullptr;
    }

    client->implementation_identifier = rmw_get_implementation_identifier();

    if (auto ptr = allocate_copy(service_name); ptr.has_error()) {
        rmw_client_free(client);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic name");
        return nullptr;
    } else {
        client->service_name = ptr.value();
    }

    return client;
}

rmw_ret_t rmw_destroy_client(rmw_node_t* node, rmw_client_t* client) {
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);

    rmw_client_free(client);

    return RMW_RET_OK;
}

rmw_ret_t rmw_send_request(const rmw_client_t* client, const void* ros_request, int64_t* sequence_id) {
    return RMW_RET_OK;
}

rmw_ret_t
rmw_take_response(const rmw_client_t* client, rmw_service_info_t* request_header, void* ros_response, bool* taken) {
    return RMW_RET_OK;
}

rmw_ret_t rmw_client_request_publisher_get_actual_qos(const rmw_client_t* client, rmw_qos_profile_t* qos) {
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    *qos = rmw_qos_profile_default;

    return RMW_RET_OK;
}

rmw_ret_t rmw_client_response_subscription_get_actual_qos(const rmw_client_t* client, rmw_qos_profile_t* qos) {
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    *qos = rmw_qos_profile_default;

    return RMW_RET_OK;
}

rmw_ret_t
rmw_client_set_on_new_response_callback(rmw_client_t* client, rmw_event_callback_t callback, const void* user_data) {
    return RMW_RET_OK;
}
}
