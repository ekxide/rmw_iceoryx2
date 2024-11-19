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
#include "iox2/node.hpp"
#include "iox2/publisher.hpp"
#include "iox2/sample_mut_uninit.hpp"
#include "iox2/service_type.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
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

/**
 * @brief Implementation of an RMW publisher using iceoryx2.
 *
 * @tparam PayloadSize The maximum size of a sample.
 */
class RMW_PUBLIC PublisherImpl
{
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniquePublisherId;
    using Payload = ::iox::Slice<uint8_t>;
    using Sample = ::iox2::SampleMutUninit<::iox2::ServiceType::Ipc, Payload, void>;
    using SampleRegistry = ::rmw::iox2::SampleRegistry<Sample>;
    using IceoryxNotifier = ::iox2::Notifier<::iox2::ServiceType::Ipc>;
    using IceoryxPublisher = ::iox2::Publisher<::iox2::ServiceType::Ipc, Payload, void>;
    // TODO: IntraPublisher

public:
    using ErrorType = Error<PublisherImpl>::Type;

public:
    PublisherImpl(CreationLock,
                  iox::optional<ErrorType>& error,
                  NodeImpl& node,
                  const char* topic,
                  const char* type,
                  const uint64_t size);

    /**
     * @brief Get the unique identifier of the publisher.
     *
     * @return A reference to the Id of the publisher.
     */
    auto unique_id() -> const iox::optional<RawIdType>&;

    /**
     * @brief Get the topic name.
     */
    auto topic() const -> const std::string&;

    /**
     * @brief Get the type name.
     */
    auto type() const -> const std::string&;

    /**
     * @brief Get the iceoryx2 service name.
     *
     * @return Name of the service representing this subscriber.
     */
    auto service_name() const -> const std::string&;

    auto payload_size() const -> uint64_t;

    /**
     * @brief Loan memory from iceoryx2 to be populated with data to publish.
     *
     * @return Pointer to the beginning of the loaned memory which resides in shared memory.
     */
    auto loan() -> iox::expected<void*, ErrorType>;

    /**
     * @brief Returns previously loaned, unpublished memory to iceoryx2.
     * @detail Does nothing if loaned_memory does not point to memory that was previously loaned, or if it was already
     * dropped.
     *
     * @param loaned_memory Pointer to memory loaned from iceoryx.
     *
     * @return Error if the provided loaned_memory was not previously loaned.
     */
    auto return_loan(void* loaned_memory) -> iox::expected<void, ErrorType>;

    /**
     * @brief Publish memory previously loaned by this publisher.
     * @details The loaned memory is automatically released on successful publish.
     *
     * @param ros_message Pointer to the beginning of the loaned memory
     *
     * @return Error if unable to publish the provided loaned_memory.
     */
    auto publish_loan(void* loaned_memory) -> iox::expected<void, ErrorType>;

    auto publish_copy(const void* msg, uint64_t size) -> iox::expected<void, ErrorType>;

private:
    const std::string m_topic;
    const std::string m_type;
    const std::string m_service_name;
    const uint64_t m_payload_size;

    iox::optional<IdType> m_unique_id;
    iox::optional<IceoryxNotifier> m_notifier;
    iox::optional<IceoryxPublisher> m_publisher;
    SampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
