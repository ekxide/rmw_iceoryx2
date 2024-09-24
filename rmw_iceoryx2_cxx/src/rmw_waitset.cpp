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
rmw_wait_set_t* rmw_create_wait_set(rmw_context_t* context, size_t max_conditions)
{
    return NULL;
}

rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t* wait_set)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_wait(rmw_subscriptions_t* subscriptions,
                   rmw_guard_conditions_t* guard_conditions,
                   rmw_services_t* services,
                   rmw_clients_t* clients,
                   rmw_events_t* events,
                   rmw_wait_set_t* wait_set,
                   const rmw_time_t* wait_timeout)
{
    return RMW_RET_ERROR;
}
}
