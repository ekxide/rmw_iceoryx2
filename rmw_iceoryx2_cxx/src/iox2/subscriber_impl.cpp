// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"

namespace rmw::iox2
{

SubscriberImpl::SubscriberImpl(NodeImpl& node, const char* topic, const char* type)
    : m_topic{topic}
    , m_type{type} {
    using ::iox2::ServiceName;

    // TODO: make human-readable name
    auto service_name = ServiceName::create(topic).expect("TODO: propagate");
    auto service = node.as_iox2()
                       .service_builder(service_name)
                       .publish_subscribe<Payload>()
                       .open_or_create()
                       .expect("TODO: propagate");
    auto subscriber = service.subscriber_builder().create().expect("TODO: propagate");
    m_subscriber.emplace(std::move(subscriber));
}

auto SubscriberImpl::topic() -> const std::string& {
    return m_topic;
}

auto SubscriberImpl::type() -> const std::string& {
    return m_type;
}

auto SubscriberImpl::take_loan() -> iox::optional<Sample> {
    return iox::nullopt;
}

auto SubscriberImpl::take_copy() -> iox::optional<Payload> {
    return iox::nullopt;
}

} // namespace rmw::iox2
