// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"

extern "C" {

rmw_client_t* rmw_create_client(const rmw_node_t* node,
                                const rosidl_service_type_support_t* type_support,
                                const char* service_name,
                                const rmw_qos_profile_t* qos_policies) {
    /* NOT SUPPORTED (YET) */
    IOX_TODO();
}

rmw_ret_t rmw_destroy_client(rmw_node_t* node, rmw_client_t* client) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_send_request(const rmw_client_t* client, const void* ros_request, int64_t* sequence_id) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_response(const rmw_client_t* client, rmw_service_info_t* request_header, void* ros_response, bool* taken) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_client_request_publisher_get_actual_qos(const rmw_client_t* client, rmw_qos_profile_t* qos) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_client_response_subscription_get_actual_qos(const rmw_client_t* client, rmw_qos_profile_t* qos) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_client_set_on_new_response_callback(rmw_client_t* client, rmw_event_callback_t callback, const void* user_data) {
    return RMW_RET_UNSUPPORTED;
}
}
