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
rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity)
{
    return RMW_RET_ERROR;
}
}
