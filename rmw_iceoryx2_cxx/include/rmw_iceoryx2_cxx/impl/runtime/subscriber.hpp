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

#include "iox2/bb/expected.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/bb/slice.hpp"
#include "iox2/marker.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/message/message_info_header.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/sample_registry.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rmw::iox2
{

class Subscriber;

template <>
struct Error<Subscriber>
{
    using Type = SubscriberError;
};

/// @brief Message info of a received sample
struct SampleInfo
{
    ::rmw_iceoryx2_interoperability::MessageInfoHeader header;
    ::iox2::RawIdType publisher_id;
};

struct SubscriberLoan
{
    uint8_t* bytes;
    size_t number_of_bytes;
    SampleInfo message_info;
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
    using Payload = ::iox2::bb::Slice<::iox2::CustomPayloadMarker>;
    using UserHeader = ::rmw_iceoryx2_interoperability::MessageInfoHeader;
    using ErrorType = Error<Subscriber>::Type;

private:
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniqueSubscriberId;
    using IceoryxService = Iceoryx2::InterProcess::PublishSubscribe<Payload, UserHeader>;
    using IceoryxSubscriber = Iceoryx2::InterProcess::Subscriber<Payload, UserHeader>;
    using IceoryxSample = Iceoryx2::InterProcess::Sample<Payload, UserHeader>;
    using IceoryxListener = Iceoryx2::InterProcess::Listener;
    using IceoryxSampleRegistry = SampleRegistry<IceoryxSample>;

public:
    /// @brief Constructor for SubscriberImpl
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] node The node that owns this subscriber
    /// @param[in] topic The topic name to subscribe to
    /// @param[in] type_support The message typesupport
    /// @param[in] qos The resolved QoS for this subscriber
    Subscriber(CreationLock,
               ::iox2::bb::Optional<ErrorType>& error,
               Node& node,
               const char* topic,
               const rosidl_message_type_support_t* type_support,
               const Qos& qos);

    /// @brief Get the unique identifier of the subscriber
    /// @return Optional containing the raw ID of the subscriber
    auto unique_id() -> const ::iox2::bb::Optional<RawIdType>&;

    /// @brief Get the topic name
    /// @return The topic name as string reference
    auto topic() const -> const std::string&;

    /// @brief Get the typesupport used by the publisher
    /// @return Pointer to the typesupport stored in the loaded typesupport library
    auto typesupport() const -> const rosidl_message_type_support_t*;

    /// @brief Get the service name used internally, required for matching via iceoryx2
    /// @return The service name as string
    auto service_name() const -> const std::string&;

    /// @brief Check if samples are available to take
    /// @return True if at least one sample can be taken
    auto has_samples() const -> bool;

    /// @brief Get the listener that is notified about sent samples, to wait on it
    /// @return Reference to the listener
    auto listener() -> IceoryxListener&;

    /// @brief Get the resolved QoS used to create this subscriber
    /// @return Reference to the resolved QoS
    auto qos() const -> const Qos&;

    /// @brief Get the number of publishers connected to this subscriber's topic
    /// @return The number of publishers
    auto number_of_publishers() const -> uint64_t;

    /// @brief Take a message by copying its payload to the destination buffer
    /// @param[out] dest Pointer to the destination buffer
    /// @return Expected containing the sample's message info if a message was taken, empty otherwise
    auto take_copy(void* dest) -> ::iox2::bb::Expected<::iox2::bb::Optional<SampleInfo>, ErrorType>;

    /// @brief Take a loaned message without copying
    /// @return Expected containing optional pointer to the loaned message memory
    auto take_loan() -> ::iox2::bb::Expected<::iox2::bb::Optional<SubscriberLoan>, ErrorType>;

    /// @brief Return previously loaned message memory
    /// @param[in] loaned_memory Pointer to the loaned memory to return
    /// @return Expected containing void if successful
    auto return_loan(void* loan) -> ::iox2::bb::Expected<void, ErrorType>;

private:
    // m_topic, m_service_name, and m_qos are logically const after
    // construction. The `const` qualifier is omitted only because storing
    // this class in `iox2::bb::Optional` requires it to be move-assignable.
    // Do not mutate them.
    std::string m_topic;
    const rosidl_message_type_support_t* m_typesupport;
    std::string m_service_name;
    Qos m_qos;

    ::iox2::bb::Optional<IdType> m_iox2_unique_id;
    ::iox2::bb::Optional<IceoryxService> m_iox2_service;
    ::iox2::bb::Optional<IceoryxSubscriber> m_iox2_subscriber;
    ::iox2::bb::Optional<IceoryxListener> m_iox2_listener;
    IceoryxSampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
