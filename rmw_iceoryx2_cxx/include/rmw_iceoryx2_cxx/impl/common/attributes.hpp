// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_COMMON_ATTRIBUTES_HPP_
#define RMW_IOX2_COMMON_ATTRIBUTES_HPP_

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
#include "rosidl_runtime_c/type_hash.h"

#include <cstdint>
#include <utility>

/// Per-policy descriptors for the `rmw.qos.local.*` attribute namespace.
/// Each struct owns its attribute key, its allowed string values, and the
/// encode/decode translation between `Qos` and the string stored in iceoryx2
/// services.
namespace rmw::iox2::attributes
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

/// Descriptor for the `rmw.ros.type_hash` attribute carrying the REP-2011 type
/// hash as a RIHS string (`RIHS01_...`). Unlike the QoS descriptors above it
/// encodes/decodes a `rosidl_type_hash_t` rather than a `Qos`. Endpoints of the
/// same topic share the same value, so it is a required service attribute.
struct TypeHash
{
    static constexpr char KEY[] = "rmw.ros.type_hash";

    RMW_PUBLIC static void encode(const rosidl_type_hash_t& type_hash, char* buf, size_t len);
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<rosidl_type_hash_t>;
};

/// Invoke `callback(const char*)` with the raw value stored under `key`, if
/// present. The pointer is valid only for the duration of the call. Does
/// nothing when `key` is absent or its key form cannot be constructed.
template <typename Callback>
void visit_attribute_value(::iox2::AttributeSetView attribute_set, const char* key, Callback&& callback) {
    auto key_obj = ::iox2::Attribute::Key::from_utf8_null_terminated_unchecked(key);
    if (!key_obj.has_value()) {
        return;
    }
    auto val = attribute_set.key_value(key_obj.value(), 0);
    if (!val.has_value()) {
        return;
    }
    std::forward<Callback>(callback)(val.value().unchecked_access().c_str());
}

} // namespace rmw::iox2::attributes

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
    /// Fallible conversion of a service's QoS (and optionally its type hash) into
    /// the set of attributes defined when the service is created.
    static auto from(const Qos& qos, const ::iox2::bb::Optional<rosidl_type_hash_t>& type_hash = ::iox2::bb::NULLOPT)
        -> ::iox2::bb::Expected<::iox2::AttributeSpecifier, QosError>;
};

template <>
struct RMW_PUBLIC TryConvert<::iox2::AttributeVerifier>
{
    /// Fallible conversion of a service's QoS (and optionally its type hash) into
    /// the set of attributes required when the service is opened.
    static auto from(const Qos& qos, const ::iox2::bb::Optional<rosidl_type_hash_t>& type_hash = ::iox2::bb::NULLOPT)
        -> ::iox2::bb::Expected<::iox2::AttributeVerifier, QosError>;
};

} // namespace rmw::iox2

#endif // RMW_IOX2_COMMON_ATTRIBUTES_HPP_
