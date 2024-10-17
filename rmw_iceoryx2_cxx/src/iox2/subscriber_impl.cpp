// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/names.hpp"

namespace rmw::iox2
{

SubscriberImpl::SubscriberImpl(NodeImpl& node, const uint32_t context_id, const char* topic, const char* type)
    : m_topic{topic}
    , m_type{type}
    , m_service_name{::rmw::iox2::names::topic(context_id, topic)} {
    using ::iox2::ServiceName;

    auto service_name = ServiceName::create(m_service_name.c_str()).expect("TODO: propagate");
    auto service = node.as_iox2()
                       .service_builder(service_name)
                       .publish_subscribe<Payload>()
                       .payload_alignment(alignof(std::max_align_t)) // TODO: How to provide the correct alignment?
                       .open_or_create()
                       .expect("TODO: propagate");
    auto subscriber = service.subscriber_builder().create().expect("TODO: propagate");
    m_subscriber.emplace(std::move(subscriber));
}

auto SubscriberImpl::topic() const -> const std::string& {
    return m_topic;
}

auto SubscriberImpl::type() const -> const std::string& {
    return m_type;
}

auto SubscriberImpl::service_name() const -> const std::string& {
    return m_service_name;
}

auto SubscriberImpl::take_loan() -> iox::optional<Sample> {
    return iox::nullopt;
}

auto SubscriberImpl::take_copy() -> iox::optional<Payload> {
    return iox::nullopt;
}

} // namespace rmw::iox2
