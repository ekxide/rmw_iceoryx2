// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_COMMON_LOG_HPP_
#define RMW_IOX2_COMMON_LOG_HPP_

#include "iox2/bb/into.hpp"
#include "iox2/enum_translation.hpp"
#include "iox2/notifier_error.hpp"
#include "rcutils/logging_macros.h"

#include <string>

#define RMW_IOX2_LOG_DEBUG(...) RCUTILS_LOG_DEBUG_NAMED("rmw_iceoryx2", __VA_ARGS__)
#define RMW_IOX2_LOG_INFO(...) RCUTILS_LOG_INFO_NAMED("rmw_iceoryx2", __VA_ARGS__)
#define RMW_IOX2_LOG_WARN(...) RCUTILS_LOG_WARN_NAMED("rmw_iceoryx2", __VA_ARGS__)
#define RMW_IOX2_LOG_ERROR(...) RCUTILS_LOG_ERROR_NAMED("rmw_iceoryx2", __VA_ARGS__)

namespace rmw::iox2
{

/// @brief Logs why waking up the receivers failed after a message, request or response was sent
/// @param[in] error The error of the notification
/// @param[in] sent What was sent, e.g. "request to"
/// @param[in] name The topic or service it was sent to
inline void log_notify_error(::iox2::NotifierNotifyError error, const char* sent, const std::string& name) {
    switch (error) {
    case ::iox2::NotifierNotifyError::MissedDeadline:
        RMW_IOX2_LOG_DEBUG("%s '%s' sent, deadline missed", sent, name.c_str());
        break;
    case ::iox2::NotifierNotifyError::UnableToAcquireElapsedTime:
        RMW_IOX2_LOG_DEBUG("%s '%s' sent, deadline unchecked (clock read failed)", sent, name.c_str());
        break;
    case ::iox2::NotifierNotifyError::EventIdOutOfBounds:
    case ::iox2::NotifierNotifyError::InvalidListenerKey:
        RMW_IOX2_LOG_WARN(
            "%s '%s' sent, but wake-up failed: %s", sent, name.c_str(), ::iox2::bb::into<const char*>(error));
        break;
    }
}

} // namespace rmw::iox2

#endif
