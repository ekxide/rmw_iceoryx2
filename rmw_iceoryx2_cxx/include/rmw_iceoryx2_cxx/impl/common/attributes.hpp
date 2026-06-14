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

/// Per-policy descriptors for attributes.
/// Each struct owns its attribute key, its allowed string values, and the
/// encode/decode logic.
namespace rmw::iox2::attributes
{

/// Descriptor for the `ros.qos.history` attribute carrying the history
/// policy and, for `keep_last`, the queue depth.
struct History
{
    static constexpr char KEY[] = "ros.qos.history";
    static constexpr char VALUE_KEEP_LAST[] = "keep_last";

    RMW_PUBLIC static auto encode(const Qos& qos, char* buf, size_t len) -> void;
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<uint64_t>;
};

/// Descriptor for the `ros.qos.reliability` attribute carrying the delivery
/// guarantee (`reliable` or `best_effort`).
struct Reliability
{
    static constexpr char KEY[] = "ros.qos.reliability";
    static constexpr char VALUE_RELIABLE[] = "reliable";
    static constexpr char VALUE_BEST_EFFORT[] = "best_effort";

    RMW_PUBLIC static auto encode(const Qos& qos, char* buf, size_t len) -> void;
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Qos::Reliability>;
};

/// Descriptor for the `ros.qos.durability` attribute carrying whether samples
/// are retained for late-joining subscribers (`volatile` or `transient_local`).
struct Durability
{
    static constexpr char KEY[] = "ros.qos.durability";
    static constexpr char VALUE_VOLATILE[] = "volatile";
    static constexpr char VALUE_TRANSIENT_LOCAL[] = "transient_local";

    RMW_PUBLIC static auto encode(const Qos& qos, char* buf, size_t len) -> void;
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Qos::Durability>;
};

/// Descriptor for the `ros.qos.deadline` attribute carrying the maximum
/// expected duration between consecutive messages.
struct Deadline
{
    static constexpr char KEY[] = "ros.qos.deadline";
    static constexpr char VALUE_DURATION[] = "duration";

    RMW_PUBLIC static auto encode(const Qos& qos, char* buf, size_t len) -> void;
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
};

/// Descriptor for the `ros.qos.lifespan` attribute carrying the maximum
/// duration a sample remains valid after publication.
struct Lifespan
{
    static constexpr char KEY[] = "ros.qos.lifespan";
    static constexpr char VALUE_DURATION[] = "duration";

    RMW_PUBLIC static auto encode(const Qos& qos, char* buf, size_t len) -> void;
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
};

/// Descriptor for the `ros.qos.liveliness` attribute carrying the liveliness
/// kind (`automatic` or `manual_by_topic`) and lease duration.
struct Liveliness
{
    struct Value
    {
        Qos::Liveliness kind;
        Qos::Duration lease;
    };

    static constexpr char KEY[] = "ros.qos.liveliness";
    static constexpr char VALUE_AUTOMATIC[] = "automatic";
    static constexpr char VALUE_MANUAL_BY_TOPIC[] = "manual_by_topic";

    RMW_PUBLIC static auto encode(const Qos& qos, char* buf, size_t len) -> void;
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<Value>;
};

/// Descriptor for the `ros.type_hash` attribute carrying the REP-2011 type
/// hash as a RIHS string (`RIHS01_...`). U
struct TypeHash
{
    static constexpr char KEY[] = "ros.type_hash";

    RMW_PUBLIC static auto encode(const rosidl_type_hash_t& type_hash, char* buf, size_t len) -> void;
    RMW_PUBLIC static auto decode(const char* str) -> ::iox2::bb::Optional<rosidl_type_hash_t>;
};

/// Invoke `callback(const char*)` with the raw value stored under `key`, if
/// present. The pointer is valid only for the duration of the call. Does
/// nothing when `key` is absent or its key form cannot be constructed.
template <typename Callback>
auto visit_attribute_value(::iox2::AttributeSetView attribute_set, const char* key, Callback&& callback) -> void {
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
