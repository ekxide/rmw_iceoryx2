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
#include "iox2/bb/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/convert.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/matching.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"

#include <cstdint>

/// Per-policy descriptors for the `rmw.qos.local.*` attribute namespace.
/// Each struct owns its attribute key, its allowed string values, and the
/// encode/decode translation between `Qos` and the string stored in iceoryx2
/// services.
namespace rmw::iox2::qos::attributes
{

struct History
{
    static constexpr char KEY[] = "rmw.qos.local.history";
    static constexpr char VALUE_KEEP_LAST[] = "keep_last";

    RMW_PUBLIC static void encode(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<uint64_t>;
};

struct Reliability
{
    static constexpr char KEY[] = "rmw.qos.local.reliability";
    static constexpr char VALUE_RELIABLE[] = "reliable";
    static constexpr char VALUE_BEST_EFFORT[] = "best_effort";

    RMW_PUBLIC static void encode(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Qos::Reliability>;
};

struct Durability
{
    static constexpr char KEY[] = "rmw.qos.local.durability";
    static constexpr char VALUE_VOLATILE[] = "volatile";
    static constexpr char VALUE_TRANSIENT_LOCAL[] = "transient_local";

    RMW_PUBLIC static void encode(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Qos::Durability>;
};

struct Deadline
{
    static constexpr char KEY[] = "rmw.qos.local.deadline";
    static constexpr char VALUE_DURATION[] = "duration";

    RMW_PUBLIC static void encode(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
};

struct Lifespan
{
    static constexpr char KEY[] = "rmw.qos.local.lifespan";
    static constexpr char VALUE_DURATION[] = "duration";

    RMW_PUBLIC static void encode(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
};

struct Liveliness
{
    struct Value
    {
        Qos::Liveliness kind;
        Qos::Duration lease;
    };

    static constexpr char KEY[] = "rmw.qos.local.liveliness";
    static constexpr char VALUE_AUTOMATIC[] = "automatic";
    static constexpr char VALUE_MANUAL_BY_TOPIC[] = "manual_by_topic";

    RMW_PUBLIC static void encode(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Value>;
};

} // namespace rmw::iox2::qos::attributes

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
