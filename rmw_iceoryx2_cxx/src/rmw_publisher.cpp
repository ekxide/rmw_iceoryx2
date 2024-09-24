// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/get_network_flow_endpoints.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"

extern "C" {

rmw_ret_t rmw_init_publisher_allocation(const rosidl_message_type_support_t* type_support,
                                        const rosidl_runtime_c__Sequence__bound* message_bounds,
                                        rmw_publisher_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_fini_publisher_allocation(rmw_publisher_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_publisher_t* rmw_create_publisher(const rmw_node_t* node,
                                      const rosidl_message_type_support_t* type_support,
                                      const char* topic_name,
                                      const rmw_qos_profile_t* qos_profile,
                                      const rmw_publisher_options_t* publisher_options)
{
    return NULL;
}

rmw_ret_t rmw_destroy_publisher(rmw_node_t* node, rmw_publisher_t* publisher)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_borrow_loaned_message(const rmw_publisher_t* publisher,
                                    const rosidl_message_type_support_t* type_support,
                                    void** ros_message)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_return_loaned_message_from_publisher(const rmw_publisher_t* publisher, void* loaned_message)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_publish(const rmw_publisher_t* publisher, const void* ros_message, rmw_publisher_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t
rmw_publish_loaned_message(const rmw_publisher_t* publisher, void* ros_message, rmw_publisher_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t* publisher, size_t* subscription_count)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t* publisher, rmw_qos_profile_t* qos)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_publish_serialized_message(const rmw_publisher_t* publisher,
                                         const rmw_serialized_message_t* serialized_message,
                                         rmw_publisher_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_serialized_message_size(const rosidl_message_type_support_t* type_support,
                                          const rosidl_runtime_c__Sequence__bound* message_bounds,
                                          size_t* size)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t* publisher)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_publisher_wait_for_all_acked(const rmw_publisher_t* publisher, rmw_time_t wait_timeout)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_publisher_get_network_flow_endpoints(const rmw_publisher_t* publisher,
                                                   rcutils_allocator_t* allocator,
                                                   rmw_network_flow_endpoint_array_t* network_flow_endpoint_array)
{
    return RMW_RET_ERROR;
}
}
