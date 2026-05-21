// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_COMMON_QOS_HPP_
#define RMW_IOX2_COMMON_QOS_HPP_

#include "iox2/attribute_set.hpp"
#include "iox2/attribute_specifier.hpp"
#include "iox2/attribute_verifier.hpp"
#include "iox2/bb/expected.hpp"
#include "iox2/bb/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/convert.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/common/resolved_qos.hpp"

#include <cstdint>

namespace rmw::iox2
{

/// Controls how `rmw_create_*` reconciles requested QoS with an existing
/// iceoryx2 service.
/// TODO: Move to init_options
enum class QosMatchMode : uint8_t {
    STRICT, ///< require attribute equality (default)
    ADOPT,  ///< substitute caller QoS with existing service attributes
};

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

/// Fallible conversion `ResolvedQos` → `iox2::AttributeSpecifier`.
template <>
struct RMW_PUBLIC TryConvert<::iox2::AttributeSpecifier>
{
    static auto from(const ResolvedQos& qos) -> ::iox2::bb::Expected<::iox2::AttributeSpecifier, QosError>;
};

/// Fallible conversion `ResolvedQos` → `iox2::AttributeVerifier`.
template <>
struct RMW_PUBLIC TryConvert<::iox2::AttributeVerifier>
{
    static auto from(const ResolvedQos& qos) -> ::iox2::bb::Expected<::iox2::AttributeVerifier, QosError>;
};

// ----------------------------------------------------------------------------
// Diagnostics
// ----------------------------------------------------------------------------

/// Emit `RMW_IOX2_LOG_WARN` for every policy that cannot be mapped to
/// iceoryx2.
RMW_PUBLIC
void warn_unmapped(const ResolvedQos& qos, const char* topic) noexcept;

/// First-mismatch diff between the locally requested QoS and the
/// attributes of an existing service. Returned strings are
/// null-terminated and sized to the iceoryx2 attribute limits.
struct AttributeDiff
{
    char key[64];
    char requested[256];
    char existing[256];
};

/// Find the first `rmw.qos.local.*` key whose requested value differs
/// from the existing service's attribute.
///
/// Returns NULLOPT when every key matches.
RMW_PUBLIC
auto diff_attributes(const ResolvedQos& required,
                     ::iox2::AttributeSetView existing) -> ::iox2::bb::Optional<AttributeDiff>;

} // namespace rmw::iox2

#endif // RMW_IOX2_COMMON_QOS_HPP_
