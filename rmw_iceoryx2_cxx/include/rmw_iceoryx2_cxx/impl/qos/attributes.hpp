// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_QOS_ATTRIBUTES_HPP_
#define RMW_IOX2_QOS_ATTRIBUTES_HPP_

#include "iox2/attribute_set.hpp"
#include "iox2/attribute_specifier.hpp"
#include "iox2/attribute_verifier.hpp"
#include "iox2/bb/expected.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/convert.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/matching.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"

namespace rmw::iox2
{

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

template <>
struct RMW_PUBLIC Convert<rmw_qos_profile_t>
{
    /// Infallible conversion `Qos` → `rmw_qos_profile_t`.
    /// Used at the C API boundary (e.g. `rmw_*_get_actual_qos`).
    static auto from(const Qos& qos) noexcept -> rmw_qos_profile_t;
};

template <>
struct RMW_PUBLIC TryConvert<Qos>
{
    /// Fallible conversion `rmw_qos_profile_t` → `Qos`.
    static auto from(const rmw_qos_profile_t& profile, ProfileKind kind) -> ::iox2::bb::Expected<Qos, QosError>;
    /// Fallible conversion `::iox2::AttributeSetView` → `Qos`.
    static auto from(::iox2::AttributeSetView attributes, ProfileKind kind) -> ::iox2::bb::Expected<Qos, QosError>;
};

template <>
struct RMW_PUBLIC TryConvert<::iox2::AttributeSpecifier>
{
    /// Fallible conversion `Qos` → `iox2::AttributeSpecifier`.
    static auto from(const Qos& qos) -> ::iox2::bb::Expected<::iox2::AttributeSpecifier, QosError>;
};

template <>
struct RMW_PUBLIC TryConvert<::iox2::AttributeVerifier>
{
    /// Fallible conversion `Qos` → `iox2::AttributeVerifier`.
    static auto from(const Qos& qos) -> ::iox2::bb::Expected<::iox2::AttributeVerifier, QosError>;
};

// ----------------------------------------------------------------------------
// Attribute helpers
// ----------------------------------------------------------------------------

/// Copy the value of `key` from attributes into the caller-provided buffer
/// (null-terminated). Returns false when the key is absent from `attrs`.
RMW_PUBLIC
auto read_attribute_value(::iox2::AttributeSetView attributes, const char* key, char* out, size_t out_size) -> bool;

} // namespace rmw::iox2

#endif // RMW_IOX2_QOS_ATTRIBUTES_HPP_
