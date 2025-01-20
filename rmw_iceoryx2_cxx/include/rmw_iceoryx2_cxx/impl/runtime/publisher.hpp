// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_PUBLISHER_HPP_
#define RMW_IOX2_RUNTIME_PUBLISHER_HPP_

#include "iox/optional.hpp"
#include "iox/slice.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/sample_registry.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rmw::iox2
{

class Publisher;

template <>
struct Error<Publisher>
{
    using Type = PublisherError;
};

/// @brief Implementation of the RMW publisher for iceoryx2
///
/// @details The implementation supports both copy and loan-based publishing mechanisms,
///          allowing for efficient zero-copy communication when possible.
///
/// It manages the lifecycle of loaned memory and handles the interaction with the
/// iceoryx2 middleware layer.
class RMW_PUBLIC Publisher
{
public:
    using Payload = ::iox::Slice<uint8_t>;
    using ErrorType = Error<Publisher>::Type;

private:
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniquePublisherId;

    using IceoryxNotifier = Iceoryx2::InterProcess::Notifier;
    using IceoryxPublisher = Iceoryx2::InterProcess::Publisher<Payload>;
    using IceoryxSample = Iceoryx2::InterProcess::SampleMutUninit<Payload>;
    using SampleRegistry = SampleRegistry<IceoryxSample>;

public:
    /// @brief Constructor for PublisherImpl
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] node The node that owns this publisher
    /// @param[in] topic The topic name to publish to
    /// @param[in] type The message type name
    /// @param[in] size The maximum message payload size
    Publisher(CreationLock,
              iox::optional<ErrorType>& error,
              Node& node,
              const char* topic,
              const rosidl_message_type_support_t* type_support);

    /// @brief Get the unique identifier of this publisher
    /// @return The unique id or empty optional if failing to retrieve it from iceoryx2
    auto unique_id() -> const iox::optional<RawIdType>&;

    /// @brief Get the topic name
    /// @return The topic name as string
    auto topic() const -> const std::string&;

    /// @brief Get the typesupport used by the publisher
    /// @return Pointer to the typesupport stored in the loaded typesupport library
    auto typesupport() const -> const rosidl_message_type_support_t*;

    /// @brief Get the service name used internally, required for matching via iceoryx2
    /// @return The service name as string
    auto service_name() const -> const std::string&;

    /// @brief Loan memory for zero-copy publishing
    /// @return Expected containing pointer to loaned memory or error
    auto loan(uint64_t num_bytes) -> iox::expected<void*, ErrorType>;

    /// @brief Return previously loaned memory without publishing
    /// @param[in] loaned_memory Pointer to the loaned memory to return
    /// @return Expected containing void or error if return failed
    auto return_loan(void* loaned_memory) -> iox::expected<void, ErrorType>;

    /// @brief Publish previously loaned memory
    /// @param[in] loaned_memory Pointer to the loaned memory to publish
    /// @note The memory must be initialized before publishing
    /// @return Expected containing void or error if publish failed
    auto publish_loan(void* loaned_memory) -> iox::expected<void, ErrorType>;

    /// @brief Publish data by copying
    /// @param[in] msg Pointer to the message data to copy
    /// @param[in] size Size of the message data in bytes
    /// @return Expected containing void or error if publish failed
    auto publish_copy(const void* msg, uint64_t size) -> iox::expected<void, ErrorType>;

private:
    const std::string m_topic;
    const rosidl_message_type_support_t* m_typesupport;
    const std::string m_service_name;

    iox::optional<IdType> m_iox_unique_id;
    iox::optional<IceoryxNotifier> m_iox2_notifier;
    iox::optional<IceoryxPublisher> m_iox2_publisher;
    SampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
