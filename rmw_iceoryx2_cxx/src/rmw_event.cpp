// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/ret_types.h"
#include "rmw/rmw.h"

extern "C" {

rmw_ret_t
rmw_publisher_event_init(rmw_event_t* rmw_event, const rmw_publisher_t* publisher, rmw_event_type_t event_type)
{
    return RMW_RET_ERROR;
}

rmw_ret_t
rmw_subscription_event_init(rmw_event_t* rmw_event, const rmw_subscription_t* subscription, rmw_event_type_t event_type)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_take_event(const rmw_event_t* event_handle, void* event_info, bool* taken)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_event_fini(rmw_event_t* event)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_event_set_callback(rmw_event_t* event, rmw_event_callback_t callback, const void* user_data)
{
    return RMW_RET_ERROR;
}
}
