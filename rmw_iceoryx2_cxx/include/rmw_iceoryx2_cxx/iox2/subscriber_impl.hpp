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
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/sample_registry.hpp"

namespace rmw::iox2
{

class RMW_PUBLIC SubscriberImpl
{
    using Payload = ::iox::Slice<uint8_t>;
    using Sample = ::iox2::Sample<::iox2::ServiceType::Ipc, Payload, void>;
    using SampleRegistry = ::rmw::iox2::SampleRegistry<Sample>;
    using IceoryxSubscriber = ::iox2::Subscriber<::iox2::ServiceType::Ipc, Payload, void>;
    // TODO: IntraSubscriber

public:
    explicit SubscriberImpl(NodeImpl& node, const uint32_t context_id, const char* topic, const char* type);

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
     * @brief Take a loan to the next sample in shared memory.
     *
     * @return Empty optional if no samples available
     */
    auto take() -> iox::expected<iox::optional<const void*>, LoanError>;

    auto return_loan(void* loaned_memory) -> iox::expected<void, LoanError>;

private:
    const std::string m_topic;
    const std::string m_type;
    const std::string m_service_name;

    iox::optional<IceoryxSubscriber> m_subscriber;
    SampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
