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
rmw_service_t* rmw_create_service(const rmw_node_t* node,
                                  const rosidl_service_type_support_t* type_support,
                                  const char* service_name,
                                  const rmw_qos_profile_t* qos_profile) {
    IOX_TODO();
}

rmw_ret_t rmw_destroy_service(rmw_node_t* node, rmw_service_t* service) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_request(const rmw_service_t* service, rmw_service_info_t* request_header, void* ros_request, bool* taken) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_send_response(const rmw_service_t* service, rmw_request_id_t* request_header, void* ros_response) {
    return RMW_RET_UNSUPPORTED;
}


rmw_ret_t rmw_service_request_subscription_get_actual_qos(const rmw_service_t* service, rmw_qos_profile_t* qos) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_service_response_publisher_get_actual_qos(const rmw_service_t* service, rmw_qos_profile_t* qos) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_service_set_on_new_request_callback(rmw_service_t* service, rmw_event_callback_t callback, const void* user_data) {
    return RMW_RET_UNSUPPORTED;
}
}
