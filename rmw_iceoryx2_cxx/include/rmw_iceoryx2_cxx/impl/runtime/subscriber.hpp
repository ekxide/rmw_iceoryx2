// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_SUBSCRIBER_IMPL_HPP_
#define RMW_IOX2_SUBSCRIBER_IMPL_HPP_

#include "iox/optional.hpp"
#include "iox/slice.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/sample_registry.hpp"

namespace rmw::iox2
{

class Subscriber;

template <>
struct Error<Subscriber>
{
    using Type = SubscriberError;
};

/// @brief Implementation of the RMW subscriber for iceoryx2
/// @details The implementation supports both copy and loan-based data access patterns,
///          allowing for efficient zero-copy communication when possible.
///
/// It manages the lifecycle of loaned memory and handles the interaction with the
/// iceoryx2 middleware layer.
class RMW_PUBLIC Subscriber
{
public:
    using ErrorType = Error<Subscriber>::Type;
    using Payload = ::iox::Slice<uint8_t>;

private:
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniqueSubscriberId;
    using IceoryxSubscriber = Iceoryx2::InterProcess::Subscriber<Payload>;
    using IceoryxSample = Iceoryx2::InterProcess::Sample<Payload>;
    using SampleRegistry = SampleRegistry<IceoryxSample>;

public:
    /// @brief Constructor for SubscriberImpl
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] node The node that owns this subscriber
    /// @param[in] topic The topic name to subscribe to
    /// @param[in] type The message type name
    Subscriber(CreationLock, iox::optional<ErrorType>& error, Node& node, const char* topic, const char* type);

    /// @brief Get the unique identifier of the subscriber
    /// @return Optional containing the raw ID of the subscriber
    auto unique_id() -> const iox::optional<RawIdType>&;

    /// @brief Get the topic name
    /// @return The topic name as string reference
    auto topic() const -> const std::string&;

    /// @brief Get the message type name
    /// @return The type name as string reference
    auto type() const -> const std::string&;

    /// @brief Get the service name used internally, required for matching via iceoryx2
    /// @return The service name as string
    auto service_name() const -> const std::string&;

    /// @brief Take a message by copying it to the destination buffer
    /// @param[out] dest Pointer to the destination buffer
    /// @return Expected containing true if a message was taken, false if no message available
    auto take_copy(void* dest) -> iox::expected<bool, ErrorType>;

    /// @brief Take a loaned message without copying
    /// @return Expected containing optional pointer to the loaned message memory
    auto take_loan() -> iox::expected<iox::optional<const void*>, ErrorType>;

    /// @brief Return previously loaned message memory
    /// @param[in] loaned_memory Pointer to the loaned memory to return
    /// @return Expected containing void if successful
    auto return_loan(void* loaned_memory) -> iox::expected<void, ErrorType>;

private:
    const std::string m_topic;
    const std::string m_type;
    const std::string m_service_name;

    iox::optional<IdType> m_iox2_unique_id;
    iox::optional<IceoryxSubscriber> m_iox2_subscriber;
    SampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
