// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_SERVICE_NAMES_HPP_
#define RMW_IOX2_SERVICE_NAMES_HPP_

#include "rmw/visibility_control.h"

#include <cstdint>
#include <string>

namespace rmw::iox2::names
{

RMW_PUBLIC
std::string context(const uint32_t context_id);

RMW_PUBLIC
std::string node(const uint32_t context_id, const char* name, const char* ns);

RMW_PUBLIC
std::string guard_condition(const uint32_t context_id, const uint32_t guard_condition_id);

RMW_PUBLIC
std::string topic(const char* topic);

} // namespace rmw::iox2::names

#endif // RMW_IOX2_SERVICE_NAMES_HPP_
