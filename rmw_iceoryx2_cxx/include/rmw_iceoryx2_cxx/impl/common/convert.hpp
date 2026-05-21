// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_COMMON_CONVERT_HPP_
#define RMW_IOX2_COMMON_CONVERT_HPP_

namespace rmw::iox2
{

/// Trait-style template for infallible conversions.
///
/// Specialize per destination type with a static `from(source, ...)` method
/// returning the destination by value:
/// @code{.cpp}
/// template <> struct Convert<MyDest>
/// {
///     static auto from(const MySource&) noexcept -> MyDest;
/// };
///
/// auto value = Convert<MyDest>::from(source);
/// @endcode
template <typename Dest>
struct Convert;

/// Trait-style template for fallible conversions.
///
/// Specialize per destination type with a static `from(source, ...)` method
/// returning `iox2::bb::Expected<Dest, ErrorType>`. Overload `from` on the
/// source type to support multiple inputs producing the same destination:
/// @code{.cpp}
/// template <> struct TryConvert<MyDest>
/// {
///     static auto from(const SourceA&) -> Expected<MyDest, MyError>;
///     static auto from(const SourceB&) -> Expected<MyDest, MyError>;
/// };
///
/// auto result = TryConvert<MyDest>::from(source);
/// @endcode
template <typename Dest>
struct TryConvert;

} // namespace rmw::iox2

#endif // RMW_IOX2_COMMON_CONVERT_HPP_
