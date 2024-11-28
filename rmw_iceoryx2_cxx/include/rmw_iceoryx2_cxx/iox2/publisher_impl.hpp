// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_PUBLISHER_IMPL_HPP_
#define RMW_IOX2_PUBLISHER_IMPL_HPP_

#include "iox/optional.hpp"
#include "iox/slice.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/sample_registry.hpp"

namespace rmw::iox2
{

class PublisherImpl;

template <>
struct Error<PublisherImpl>
{
    using Type = PublisherError;
};

/// @brief Implementation of the RMW publisher for iceoryx2
///
/// @details The publisher supports both zero-copy loans and copy-based publishing mechanisms.
/// @details The implementation supports both copy and loan-based publishing mechanisms,
///          allowing for efficient zero-copy communication when possible.
///
/// It manages the lifecycle of loaned memory and handles the interaction with the
/// iceoryx2 middleware layer.
class RMW_PUBLIC PublisherImpl
{
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniquePublisherId;
    using Payload = ::iox::Slice<uint8_t>;
    using Sample = Iceoryx2::InterProcess::SampleMutUninit<Payload>;
    using SampleRegistry = ::rmw::iox2::SampleRegistry<Sample>;
    using Notifier = Iceoryx2::InterProcess::Notifier;
    using Publisher = Iceoryx2::InterProcess::Publisher<Payload>;

public:
    using ErrorType = Error<PublisherImpl>::Type;

public:
    /// @brief Constructor for PublisherImpl
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] node The node that owns this publisher
    /// @param[in] topic The topic name to publish to
    /// @param[in] type The message type name
    /// @param[in] size The maximum message payload size
    PublisherImpl(CreationLock,
                  iox::optional<ErrorType>& error,
                  NodeImpl& node,
                  const char* topic,
                  const char* type,
                  const uint64_t size);

    /// @brief Get the unique identifier of this publisher
    /// @return The unique id or empty optional if failing to retrieve it from iceoryx2
    auto unique_id() -> const iox::optional<RawIdType>&;

    /// @brief Get the topic name
    /// @return The topic name as string
    auto topic() const -> const std::string&;

    /// @brief Get the message type name
    /// @return The message type name as string
    auto type() const -> const std::string&;

    /// @brief Get the service name used internally, required for matching via iceoryx2
    /// @return The service name as string
    auto service_name() const -> const std::string&;

    /// @brief Get the maximum payload size
    /// @return The maximum payload size in bytes
    auto payload_size() const -> uint64_t;

    /// @brief Loan memory for zero-copy publishing
    /// @return Expected containing pointer to loaned memory or error
    auto loan() -> iox::expected<void*, ErrorType>;

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
    const std::string m_type;
    const std::string m_service_name;
    const uint64_t m_payload_size;

    iox::optional<IdType> m_iox_unique_id;
    iox::optional<Notifier> m_iox2_notifier;
    iox::optional<Publisher> m_iox2_publisher;
    SampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
