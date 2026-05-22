// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_QOS_DIAGNOSTICS_HPP_
#define RMW_IOX2_QOS_DIAGNOSTICS_HPP_

#include "iox2/attribute_set.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"

namespace rmw::iox2
{

/// Emit `RMW_IOX2_LOG_WARN` for every policy that cannot be mapped to
/// iceoryx2.
RMW_PUBLIC
void log_unsupported_policies(const Qos& qos, const char* topic) noexcept;

/// Chain a per-key QoS mismatch error message via `RMW_IOX2_CHAIN_ERROR_MSG`,
/// comparing `requested` against the values in `existing` (attributes of the
/// iceoryx2 service).
RMW_PUBLIC
void log_attribute_mismatch(const Qos& requested, ::iox2::AttributeSetView existing, const char* topic) noexcept;

} // namespace rmw::iox2

#endif // RMW_IOX2_QOS_DIAGNOSTICS_HPP_
