// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_COMMON_QOS_ATTRIBUTES_HPP_
#define RMW_IOX2_COMMON_QOS_ATTRIBUTES_HPP_

#include "iox2/attribute_set.hpp"
#include "iox2/attribute_specifier.hpp"
#include "iox2/attribute_verifier.hpp"
#include "iox2/bb/expected.hpp"
#include "iox2/bb/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/convert.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/common/qos.hpp"

#include <cstdint>

namespace rmw::iox2
{

// ----------------------------------------------------------------------------
// Schema
// ----------------------------------------------------------------------------

/// Per-policy codecs for the `rmw.qos.local.*` attribute namespace. Each
/// struct owns its key, the sentinel constants of its value vocabulary,
/// and the `format`/`parse` statics that translate between `Qos` and the
/// string stored in an iceoryx2 service attribute.

struct History
{
    static constexpr char KEY[] = "rmw.qos.local.history";
    static constexpr char KEEP_LAST[] = "keep_last";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    /// Parses the depth from a `keep_last:N` attribute string.
    static auto parse(const char* str) -> ::iox2::bb::Optional<uint64_t>;
};

struct Reliability
{
    static constexpr char KEY[] = "rmw.qos.local.reliability";
    static constexpr char RELIABLE[] = "reliable";
    static constexpr char BEST_EFFORT[] = "best_effort";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Reliability>;
};

struct Durability
{
    static constexpr char KEY[] = "rmw.qos.local.durability";
    static constexpr char VOLATILE[] = "volatile";
    static constexpr char TRANSIENT_LOCAL[] = "transient_local";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Durability>;
};

struct Deadline
{
    static constexpr char KEY[] = "rmw.qos.local.deadline";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
};

struct Lifespan
{
    static constexpr char KEY[] = "rmw.qos.local.lifespan";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
};

struct Liveliness
{
    /// The two fields liveliness encodes together: the kind enum and the
    /// lease duration. Returned from `parse` so that both land in the
    /// caller in one shot.
    struct Value
    {
        Qos::Liveliness kind;
        Qos::Duration lease;
    };

    static constexpr char KEY[] = "rmw.qos.local.liveliness";
    static constexpr char AUTOMATIC[] = "automatic";
    static constexpr char MANUAL_BY_TOPIC[] = "manual_by_topic";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Value>;
};

// ----------------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------------

/// Fallible conversion to `Qos`. Lives here (not in `qos.hpp`) because
/// the `AttributeSetView` overload depends on the policy schema above.
template <>
struct RMW_PUBLIC TryConvert<Qos>
{
    static auto from(const rmw_qos_profile_t& profile, ProfileKind kind) -> ::iox2::bb::Expected<Qos, QosError>;
    static auto from(::iox2::AttributeSetView attrs, ProfileKind kind) -> ::iox2::bb::Expected<Qos, QosError>;
};

/// Fallible conversion `Qos` → `iox2::AttributeSpecifier`.
template <>
struct RMW_PUBLIC TryConvert<::iox2::AttributeSpecifier>
{
    static auto from(const Qos& qos) -> ::iox2::bb::Expected<::iox2::AttributeSpecifier, QosError>;
};

/// Fallible conversion `Qos` → `iox2::AttributeVerifier`.
template <>
struct RMW_PUBLIC TryConvert<::iox2::AttributeVerifier>
{
    static auto from(const Qos& qos) -> ::iox2::bb::Expected<::iox2::AttributeVerifier, QosError>;
};

// ----------------------------------------------------------------------------
// Attribute helpers
// ----------------------------------------------------------------------------

/// Copy the value of `key` from `attrs` into the caller-provided buffer
/// (null-terminated). Returns false when the key is absent from `attrs`.
RMW_PUBLIC
auto read_attribute_value(::iox2::AttributeSetView attrs, const char* key, char* out, size_t out_size) -> bool;

} // namespace rmw::iox2

#endif // RMW_IOX2_COMMON_QOS_ATTRIBUTES_HPP_
