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

#include "iox/slice.hpp"
#include "iox2/service_type.hpp"
#include "iox2/subscriber.hpp"
#include "rmw/visibility_control.h"

namespace rmw::iox2
{

class RMW_PUBLIC SubscriberImpl
{
    using IceoryxSubscriber = ::iox2::Subscriber<::iox2::ServiceType::Ipc, ::iox::Slice<uint8_t>, void>;

public:
    explicit SubscriberImpl(const std::string name, IceoryxSubscriber&& subscriber);

private:
    IceoryxSubscriber m_publisher;
};

} // namespace rmw::iox2

#endif
