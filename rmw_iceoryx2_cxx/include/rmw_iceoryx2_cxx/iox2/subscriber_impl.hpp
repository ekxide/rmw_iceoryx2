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

class RMW_PUBLIC SubscriberImpl
{
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniqueSubscriberId;
    using Payload = ::iox::Slice<uint8_t>;
    using Sample = ::iox2::Sample<::iox2::ServiceType::Ipc, Payload, void>;
    using SampleRegistry = ::rmw::iox2::SampleRegistry<Sample>;
    using IceoryxSubscriber = ::iox2::Subscriber<::iox2::ServiceType::Ipc, Payload, void>;
    // TODO: IntraSubscriber

public:
    using ErrorType = Error<SubscriberImpl>::Type;

public:
    SubscriberImpl(CreationLock,
                   iox::optional<ErrorType>& error,
                   NodeImpl& node,
                   const uint32_t context_id,
                   const char* topic,
                   const char* type);

    /**
     * @brief Get the unique identifier of the subscriber.
     *
     * @return A reference to the Id of the subscriber.
     */
    auto unique_id() -> iox::optional<RawIdType>&;

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

    /**
     * @brief Take a copy of the next available sample.
     *
     * This function attempts to take a copy of the next available sample from the subscriber
     * and store it in the provided destination buffer.
     *
     * @param dest Pointer to the destination buffer where the sample will be copied.
     * @return An expected object containing void if successful, or an ErrorType if an error occurred.
     */
    auto take_copy(void* dest) -> iox::expected<bool, ErrorType>;

    /**
     * @brief Take a loan to the next sample in shared memory.
     *
     * This function attempts to take a loan to the next available sample in shared memory.
     * If a sample is available, it returns a pointer to the sample data. If no sample
     * is available, it returns an empty optional.
     *
     * @return An expected object containing an optional pointer to const void.
     *         If successful and a sample is available, the optional contains a pointer to the sample data.
     *         If successful but no sample is available, the optional is empty.
     *         If an error occurred, the expected contains an ErrorType.
     */
    auto take_loan() -> iox::expected<iox::optional<const void*>, ErrorType>;

    /**
     * @brief Return a previously taken loan of a sample.
     *
     * This function is used to return a loan that was previously taken using the take_loan() method.
     * It informs the subscriber that the loaned memory is no longer being used by the application.
     *
     * @param loaned_memory A pointer to the loaned memory that is being returned.
     * @return An expected object containing void if successful, or an ErrorType if an error occurred.
     */
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
