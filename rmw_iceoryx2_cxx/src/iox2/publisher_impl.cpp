// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/publisher_impl.hpp"
#include "iox2/sample_mut_uninit.hpp"

namespace rmw::iox2
{

PublisherImpl::PublisherImpl(NodeImpl& node, const char* topic, const char* type)
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
    auto publisher = service.publisher_builder().create().expect("TODO: propagate");
    m_publisher.emplace(std::move(publisher));
}

auto PublisherImpl::topic() -> const std::string& {
    return m_topic;
}

auto PublisherImpl::type() -> const std::string& {
    return m_type;
}
auto PublisherImpl::loan() -> iox::expected<void*, LoanError> {
    using iox::err;
    using iox::ok;

    auto sample = m_publisher->loan();
    if (sample.has_error()) {
        return err(LoanError::IOX2_ERROR);
    }

    auto payload_ptr = sample.value().payload_mut().begin();

    // Store the sample for later use when publishing
    m_registry.store(std::move(sample.value()));

    return ok(static_cast<void*>(payload_ptr));
}

auto PublisherImpl::return_loan(void* loaned_memory) -> iox::expected<void, LoanError> {
    return m_registry.release(loaned_memory);
}

auto PublisherImpl::publish(void* loaned_memory) -> iox::expected<void, PublishError> {
    using iox::err;
    using iox::ok;
    return ok();
}

} // namespace rmw::iox2
