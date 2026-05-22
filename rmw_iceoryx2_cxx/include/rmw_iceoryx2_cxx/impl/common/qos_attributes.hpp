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
#include <functional>

namespace rmw::iox2
{

// ----------------------------------------------------------------------------
// Schema
// ----------------------------------------------------------------------------

/// One entry in the `rmw.qos.local.*` attribute schema: its key plus the
/// function pointer that encodes a `Qos` field into the string
/// stored in an iceoryx2 service attribute.
///
/// Decoding is type-specific (each policy returns its own value type) and
/// lives as a `parse` static on the per-policy structs below; it is not
/// part of this uniform table because no uniform return type fits.
struct PolicyCodec
{
    const char* key;
    void (*format)(const Qos& qos, char* buf, size_t len);
};

struct History
{
    static constexpr char KEY[] = "rmw.qos.local.history";
    static constexpr char KEEP_LAST[] = "keep_last";

    static void format(const Qos& qos, char* buf, size_t len);
    /// Parses the depth from a `keep_last:N` attribute string.
    static auto parse(const char* str) -> ::iox2::bb::Optional<uint64_t>;
};

struct Reliability
{
    static constexpr char KEY[] = "rmw.qos.local.reliability";
    static constexpr char RELIABLE[] = "reliable";
    static constexpr char BEST_EFFORT[] = "best_effort";

    static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Reliability>;
};

struct Durability
{
    static constexpr char KEY[] = "rmw.qos.local.durability";
    static constexpr char VOLATILE[] = "volatile";
    static constexpr char TRANSIENT_LOCAL[] = "transient_local";

    static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Durability>;
};

struct Deadline
{
    static constexpr char KEY[] = "rmw.qos.local.deadline";

    static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
};

struct Lifespan
{
    static constexpr char KEY[] = "rmw.qos.local.lifespan";

    static void format(const Qos& qos, char* buf, size_t len);
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

    static void format(const Qos& qos, char* buf, size_t len);
    static auto parse(const char* str) -> ::iox2::bb::Optional<Value>;
};

inline constexpr PolicyCodec POLICIES[] = {
    {History::KEY, &History::format},
    {Reliability::KEY, &Reliability::format},
    {Durability::KEY, &Durability::format},
    {Deadline::KEY, &Deadline::format},
    {Lifespan::KEY, &Lifespan::format},
    {Liveliness::KEY, &Liveliness::format},
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
// Diagnostics
// ----------------------------------------------------------------------------

/// Emit `RMW_IOX2_LOG_WARN` for every policy that cannot be mapped to
/// iceoryx2.
RMW_PUBLIC
void log_unsupported_policies(const Qos& qos, const char* topic) noexcept;

/// Iterate the `rmw.qos.local.*` policy schema in canonical order
/// (history, reliability, durability, deadline, lifespan, liveliness).
/// The callback receives each `PolicyCodec`; the caller decides what to do
/// with it (format, parse, compare, encode, …).
RMW_PUBLIC
void for_each_policy(const std::function<void(const PolicyCodec&)>& on_policy);

/// Copy the value of `key` from `attrs` into the caller-provided buffer
/// (null-terminated). Returns false when the key is absent from `attrs`.
RMW_PUBLIC
auto read_attribute_value(::iox2::AttributeSetView attrs, const char* key, char* out, size_t out_size) -> bool;

/// Chain a per-key QoS mismatch error message via `RMW_IOX2_CHAIN_ERROR_MSG`,
/// comparing `requested` against the values in `existing` (typically the
/// attributes of the in-place iceoryx2 service). Intended for the C API
/// layer to call after the runtime reports `QOS_INCOMPATIBLE` and the
/// service lookup has succeeded.
RMW_PUBLIC
void chain_attribute_mismatch_error(const Qos& requested,
                                    ::iox2::AttributeSetView existing,
                                    const char* topic) noexcept;

} // namespace rmw::iox2

#endif // RMW_IOX2_COMMON_QOS_ATTRIBUTES_HPP_
