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
#include "iox2/sample.hpp"
#include "iox2/service_type.hpp"
#include "iox2/subscriber.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/sample_registry.hpp"

namespace rmw::iox2
{

class SubscriberImpl;

template <>
struct Error<SubscriberImpl>
{
    using Type = SubscriberError;
};

/// @brief Implementation of the RMW subscriber for iceoryx2
/// @details The implementation supports both copy and loan-based data access patterns,
///          allowing for efficient zero-copy communication when possible.
///
/// It manages the lifecycle of loaned memory and handles the interaction with the
/// iceoryx2 middleware layer.
class RMW_PUBLIC SubscriberImpl
{
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniqueSubscriberId;
    using Payload = ::iox::Slice<uint8_t>;
    using Sample = ::iox2::Sample<::iox2::ServiceType::Ipc, Payload, void>;
    using SampleRegistry = ::rmw::iox2::SampleRegistry<Sample>;
    using IceoryxSubscriber = ::iox2::Subscriber<::iox2::ServiceType::Ipc, Payload, void>;

public:
    using ErrorType = Error<SubscriberImpl>::Type;

public:
    /// @brief Constructor for SubscriberImpl
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] node The node that owns this subscriber
    /// @param[in] topic The topic name to subscribe to
    /// @param[in] type The message type name
    SubscriberImpl(CreationLock, iox::optional<ErrorType>& error, NodeImpl& node, const char* topic, const char* type);

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

    iox::optional<IdType> m_unique_id;
    iox::optional<IceoryxSubscriber> m_subscriber;
    SampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
