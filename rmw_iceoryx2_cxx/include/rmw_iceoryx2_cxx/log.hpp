// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_LOG_HPP_
#define RMW_IOX2_LOG_HPP_

#include "rcutils/logging_macros.h"

#define RMW_IOX2_LOG_DEBUG(...) RCUTILS_LOG_DEBUG_NAMED("rmw_iceoryx2", __VA_ARGS__)
#define RMW_IOX2_LOG_INFO(...) RCUTILS_LOG_INFO_NAMED("rmw_iceoryx2", __VA_ARGS__)
#define RMW_IOX2_LOG_WARN(...) RCUTILS_LOG_WARN_NAMED("rmw_iceoryx2", __VA_ARGS__)
#define RMW_IOX2_LOG_ERROR(...) RCUTILS_LOG_ERROR_NAMED("rmw_iceoryx2", __VA_ARGS__)

#endif
