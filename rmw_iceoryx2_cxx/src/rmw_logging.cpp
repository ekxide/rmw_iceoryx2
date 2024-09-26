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

#include "iox2/log.hpp"

extern "C" {
rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity) {
    using iox2::LogLevel;
    using iox2::set_log_level;

    switch (severity) {
    case RMW_LOG_SEVERITY_DEBUG:
        set_log_level(LogLevel::DEBUG);
        break;
    case RMW_LOG_SEVERITY_INFO:
        set_log_level(LogLevel::INFO);
        break;
    case RMW_LOG_SEVERITY_WARN:
        set_log_level(LogLevel::WARN);
        break;
    case RMW_LOG_SEVERITY_ERROR:
        set_log_level(LogLevel::ERROR);
        break;
    case RMW_LOG_SEVERITY_FATAL:
        set_log_level(LogLevel::FATAL);
        break;
    default:
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}
}
