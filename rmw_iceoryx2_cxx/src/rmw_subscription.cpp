// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/dynamic_message_type_support.h"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"

extern "C" {

rmw_ret_t rmw_init_subscription_allocation(const rosidl_message_type_support_t* type_support,
                                           const rosidl_runtime_c__Sequence__bound* message_bounds,
                                           rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_fini_subscription_allocation(rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_subscription_t* rmw_create_subscription(const rmw_node_t* node,
                                            const rosidl_message_type_support_t* type_support,
                                            const char* topic_name,
                                            const rmw_qos_profile_t* qos_policies,
                                            const rmw_subscription_options_t* subscription_options)
{
    return NULL;
}

rmw_ret_t rmw_destroy_subscription(rmw_node_t* node, rmw_subscription_t* subscription)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_subscription_count_matched_publishers(const rmw_subscription_t* subscription, size_t* publisher_count)
{
    return RMW_RET_ERROR;
}


rmw_ret_t rmw_subscription_get_actual_qos(const rmw_subscription_t* subscription, rmw_qos_profile_t* qos)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_subscription_set_content_filter(rmw_subscription_t* subscription,
                                              const rmw_subscription_content_filter_options_t* options)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_subscription_get_content_filter(const rmw_subscription_t* subscription,
                                              rcutils_allocator_t* allocator,
                                              rmw_subscription_content_filter_options_t* options)
{
    return RMW_RET_ERROR;
}


rmw_ret_t rmw_take(const rmw_subscription_t* subscription,
                   void* ros_message,
                   bool* taken,
                   rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_take_with_info(const rmw_subscription_t* subscription,
                             void* ros_message,
                             bool* taken,
                             rmw_message_info_t* message_info,
                             rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}


rmw_ret_t rmw_take_sequence(const rmw_subscription_t* subscription,
                            size_t count,
                            rmw_message_sequence_t* message_sequence,
                            rmw_message_info_sequence_t* message_info_sequence,
                            size_t* taken,
                            rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_take_serialized_message(const rmw_subscription_t* subscription,
                                      rmw_serialized_message_t* serialized_message,
                                      bool* taken,
                                      rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_take_serialized_message_with_info(const rmw_subscription_t* subscription,
                                                rmw_serialized_message_t* serialized_message,
                                                bool* taken,
                                                rmw_message_info_t* message_info,
                                                rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_take_loaned_message(const rmw_subscription_t* subscription,
                                  void** loaned_message,
                                  bool* taken,
                                  rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_take_loaned_message_with_info(const rmw_subscription_t* subscription,
                                            void** loaned_message,
                                            bool* taken,
                                            rmw_message_info_t* message_info,
                                            rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_return_loaned_message_from_subscription(const rmw_subscription_t* subscription, void* loaned_message)
{
    return RMW_RET_ERROR;
}


rmw_ret_t rmw_take_dynamic_message(const rmw_subscription_t* subscription,
                                   rosidl_dynamic_typesupport_dynamic_data_t* dynamic_message,
                                   bool* taken,
                                   rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_take_dynamic_message_with_info(const rmw_subscription_t* subscription,
                                             rosidl_dynamic_typesupport_dynamic_data_t* dynamic_message,
                                             bool* taken,
                                             rmw_message_info_t* message_info,
                                             rmw_subscription_allocation_t* allocation)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_subscription_set_on_new_message_callback(rmw_subscription_t* subscription,
                                                       rmw_event_callback_t callback,
                                                       const void* user_data)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_subscription_get_network_flow_endpoints(const rmw_subscription_t* subscription,
                                                      rcutils_allocator_t* allocator,
                                                      rmw_network_flow_endpoint_array_t* network_flow_endpoint_array)
{
    return RMW_RET_ERROR;
}
}
