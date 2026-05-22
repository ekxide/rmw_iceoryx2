// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_QOS_CODEC_HPP_
#define RMW_IOX2_QOS_CODEC_HPP_

#include "iox2/bb/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"

#include <cstdint>

/// Per-policy codecs for the `rmw.qos.local.*` attribute namespace.
/// Provides translation between `Qos` and strings stored in `iceoryx2`
/// services.
namespace rmw::iox2::codec
{

struct History
{
    static constexpr char KEY[] = "rmw.qos.local.history";
    static constexpr char VALUE_KEEP_LAST[] = "keep_last";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto parse(const char* str) -> ::iox2::bb::Optional<uint64_t>;
};

struct Reliability
{
    static constexpr char KEY[] = "rmw.qos.local.reliability";
    static constexpr char VALUE_RELIABLE[] = "reliable";
    static constexpr char VALUE_BEST_EFFORT[] = "best_effort";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Reliability>;
};

struct Durability
{
    static constexpr char KEY[] = "rmw.qos.local.durability";
    static constexpr char VALUE_VOLATILE[] = "volatile";
    static constexpr char VALUE_TRANSIENT_LOCAL[] = "transient_local";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Durability>;
};

struct Deadline
{
    static constexpr char KEY[] = "rmw.qos.local.deadline";
    static constexpr char VALUE_DURATION[] = "duration";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
};

struct Lifespan
{
    static constexpr char KEY[] = "rmw.qos.local.lifespan";
    static constexpr char VALUE_DURATION[] = "duration";

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto parse(const char* str) -> ::iox2::bb::Optional<Qos::Duration>;
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

    RMW_PUBLIC static void format(const Qos& qos, char* buf, size_t len);
    RMW_PUBLIC static auto parse(const char* str) -> ::iox2::bb::Optional<Value>;
};

} // namespace rmw::iox2::codec

#endif // RMW_IOX2_QOS_CODEC_HPP_
