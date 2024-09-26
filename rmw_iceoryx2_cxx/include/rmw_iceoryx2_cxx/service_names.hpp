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

#include "iox2/node_name.hpp"
#include "iox2/service_name.hpp"
#include "rmw/visibility_control.h"

#include <cstdint>
#include <string>

namespace rmw::iox2
{

RMW_PUBLIC
std::string context_node_name(const uint32_t context_id);

RMW_PUBLIC
std::string guard_condition_service_name(const uint32_t context_id, const uint32_t guard_condition_id);

} // namespace rmw::iox2

#endif // RMW_IOX2_IDENTIFIER_HPP_
