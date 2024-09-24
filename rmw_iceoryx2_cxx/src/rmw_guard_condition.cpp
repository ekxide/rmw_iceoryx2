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
rmw_guard_condition_t* rmw_create_guard_condition(rmw_context_t* context)
{
    return NULL;
}

rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t* guard_condition)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t* guard_condition)
{
    return RMW_RET_ERROR;
}
}
