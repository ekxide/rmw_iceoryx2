// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/publisher_impl.hpp"
#include "iox/memory.hpp"
#include "iox2/sample_mut.hpp"
#include "rmw_iceoryx2_cxx/iox2/names.hpp"
#include <cstddef>
#include <iox2/sample_mut.hpp>

namespace rmw::iox2
{

PublisherImpl::PublisherImpl(
    NodeImpl& node, const uint32_t context_id, const char* topic, const char* type, const uint64_t payload_size)
    : m_topic{topic}
    , m_type{type}
    , m_service_name{::rmw::iox2::names::topic(context_id, topic)}
    , m_payload_size{payload_size} {
    using ::iox2::ServiceName;

    auto service_name = ServiceName::create(m_service_name.c_str()).expect("TODO: propagate");
    auto payload_service = node.as_iox2()
                               .service_builder(service_name)
                               .publish_subscribe<Payload>()
                               .payload_alignment(8) // All ROS2 messages have alignment 8. Maybe?
                               .open_or_create()
                               .expect("TODO: propagate");
    auto publisher =
        payload_service.publisher_builder().max_slice_len(m_payload_size).create().expect("TODO: propagate");
    m_publisher.emplace(std::move(publisher));

    auto event_service =
        node.as_iox2().service_builder(service_name).event().open_or_create().expect("TODO: propagate");
    auto notifier = event_service.notifier_builder().create().expect("TODO: propagate");
    m_notifier.emplace(std::move(notifier));
}

auto PublisherImpl::topic() const -> const std::string& {
    return m_topic;
}

auto PublisherImpl::type() const -> const std::string& {
    return m_type;
}

auto PublisherImpl::service_name() const -> const std::string& {
    return m_service_name;
}

auto PublisherImpl::loan() -> iox::expected<void*, LoanError> {
    using iox::err;
    using iox::ok;

    auto sample = m_publisher->loan_slice_uninit(m_payload_size);
    if (sample.has_error()) {
        return err(LoanError::FAILED_TO_LOAN);
    }

    // Store the sample for later use when publishing
    auto ptr = m_registry.store(std::move(sample.value()));

    return ok(static_cast<void*>(ptr));
}

auto PublisherImpl::return_loan(void* loaned_memory) -> iox::expected<void, LoanError> {
    return m_registry.release(static_cast<uint8_t*>(loaned_memory));
}

auto PublisherImpl::publish(void* loaned_memory) -> iox::expected<void, PublishError> {
    using ::iox::err;
    using ::iox::ok;
    using ::iox2::assume_init;
    using ::iox2::send;

    // Send
    auto sample = m_registry.release(static_cast<uint8_t*>(loaned_memory));
    if (!sample.has_value()) {
        return err(PublishError::INVALID_PAYLOAD);
    }
    if (auto result = send(assume_init(std::move(sample.value()))); result.has_error()) {
        return err(PublishError::FAILED_TO_SEND);
    }

    // Notify
    m_notifier->notify().expect("TODO: propagate");

    return ok();
}

} // namespace rmw::iox2
