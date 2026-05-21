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
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/resolved_qos.hpp"
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

struct SubscriberLoan
{
    uint8_t* bytes;
    size_t number_of_bytes;
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
    using Payload = ::iox2::bb::Slice<uint8_t>;

private:
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniqueSubscriberId;
    using IceoryxSubscriber = Iceoryx2::InterProcess::Subscriber<Payload>;
    using IceoryxSample = Iceoryx2::InterProcess::Sample<Payload>;
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
               const ResolvedQos& qos);

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

    /// @brief Get the resolved QoS used to create this subscriber
    /// @return Reference to the resolved QoS
    auto qos() const -> const ResolvedQos&;

    /// @brief Take a message by copying it to the destination buffer
    /// @param[out] dest Pointer to the destination buffer
    /// @return Expected containing true if a message was taken, false if no message available
    auto take_copy(void* dest) -> ::iox2::bb::Expected<bool, ErrorType>;

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
    ResolvedQos m_qos;

    ::iox2::bb::Optional<IdType> m_iox2_unique_id;
    ::iox2::bb::Optional<IceoryxSubscriber> m_iox2_subscriber;
    IceoryxSampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
