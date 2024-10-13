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

namespace rmw::iox2
{

class RMW_PUBLIC SubscriberImpl
{
    using Payload = ::iox::Slice<uint8_t>;
    using Sample = ::iox2::Sample<::iox2::ServiceType::Ipc, Payload, void>;
    using IceoryxSubscriber = ::iox2::Subscriber<::iox2::ServiceType::Ipc, Payload, void>;

public:
    explicit SubscriberImpl(NodeImpl& node, const char* topic, const char* type);

    /**
     * @brief Get the topic name.
     */
    auto topic() -> const std::string&;

    /**
     * @brief Get the type name.
     */
    auto type() -> const std::string&;

    /**
     * @brief Take a loan to the next sample in shared memory.
     *
     * @return Empty optional if no data available
     */
    auto take_loan() -> iox::optional<Sample>;

    /**
     * @brief Take a copy of the next sample's payload.
     *
     * @return Empty optional if no data available
     */
    auto take_copy() -> iox::optional<Payload>;

private:
    std::string m_topic;
    std::string m_type;

    iox::optional<IceoryxSubscriber> m_subscriber;
};

} // namespace rmw::iox2

#endif
