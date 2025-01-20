// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_COMMON_ERROR_HPP_
#define RMW_IOX2_COMMON_ERROR_HPP_

#include <cstdint>

namespace rmw::iox2
{

enum class MemoryError : uint8_t { ALLOCATION, CONSTRUCTION, CAST };
enum class HandleError : uint8_t { INVALID_INSTANCE_NAME, ICEORYX_HANDLE_CREATION_FAILURE };
enum class ContextError : uint8_t {
    INVARIANT_VIOLATION,
    HANDLE_CREATION_FAILURE,
};
enum class NodeError : uint8_t { INVARIANT_VIOLATION, HANDLE_CREATION_FAILURE, GRAPH_GUARD_CONDITION_CREATION_FAILURE };
enum class GuardConditionError : uint8_t {
    INVARIANT_VIOLATION,
    SERVICE_NAME_CREATION_FAILURE,
    SERVICE_CREATION_FAILURE,
    NOTIFIER_CREATION_FAILURE,
    NOTIFICATION_FAILURE
};
enum class SampleRegistryError : uint8_t { INVALID_PAYLOAD };
enum class PublisherError : uint8_t {
    INVARIANT_VIOLATION,
    SERVICE_NAME_CREATION_FAILURE,
    SERVICE_CREATION_FAILURE,
    PUBLISHER_CREATION_FAILURE,
    NOTIFIER_CREATION_FAILURE,
    LOAN_FAILURE,
    SEND_FAILURE,
    NOTIFICATION_FAILURE,
    INVALID_PAYLOAD,
};
enum class SubscriberError : uint8_t {
    INVARIANT_VIOLATION,
    SERVICE_NAME_CREATION_FAILURE,
    SERVICE_CREATION_FAILURE,
    SUBSCRIBER_CREATION_FAILURE,
    RECV_FAILURE,
    INVALID_PAYLOAD,
};
enum class WaitSetError : uint8_t {
    INVARIANT_VIOLATION,
    WAITSET_CREATION_FAILURE,
    SERVICE_NAME_CREATION_FAILURE,
    SERVICE_CREATION_FAILURE,
    LISTENER_CREATION_FAILURE,
    INVALID_WAITABLE_TYPE,
    INVALID_STORAGE_INDEX,
    ATTACHMENT_FAILURE,
    LISTENER_FAILURE,
    WAIT_FAILURE
};
enum class SerializationError : uint8_t { TYPESUPPORT_FAILURE, SERIALIZATION_FAILURE };
enum class DeserializationError : uint8_t { TYPESUPPORT_FAILURE, DESERIALIZATION_FAILURE };

/**
 * @brief Trait to determine the error type of a given type T
 * @detail All fallable constructors must provide an overload of this trait:
 * @code{.cpp}
 * template<>
 * struct Error<MyType>{
 *  using Type = MyErrorType;
 * }
 * @endcode
 *
 * @tparam T The type to determine the error type of
 */
template <typename T>
struct Error;

} // namespace rmw::iox2

#endif
