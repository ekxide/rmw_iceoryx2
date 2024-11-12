// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/publisher_impl.hpp"

#include "iox2/sample_mut.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/names.hpp"

namespace rmw::iox2
{

PublisherImpl::PublisherImpl(CreationLock,
                             iox::optional<ErrorType>& error,
                             NodeImpl& node,
                             const uint32_t context_id,
                             const char* topic,
                             const char* type,
                             const uint64_t payload_size)
    : m_topic{topic}
    , m_type{type}
    , m_service_name{::rmw::iox2::names::topic(context_id, topic)}
    , m_payload_size{payload_size} {
    using ::iox2::ServiceName;

    std::cout << "Creating PublisherImpl" << std::endl;

    auto service_name = ServiceName::create(m_service_name.c_str());
    if (service_name.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(service_name.error()));
        error.emplace(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        return;
    }

    auto payload_service = node.as_iox2()
                               .service_builder(service_name.value())
                               .publish_subscribe<Payload>()
                               .payload_alignment(8) // All ROS2 messages have alignment 8. Maybe?
                               .open_or_create();
    if (payload_service.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(payload_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto publisher = payload_service.value().publisher_builder().max_slice_len(m_payload_size).create();
    if (publisher.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(publisher.error()));
        error.emplace(ErrorType::PUBLISHER_CREATION_FAILURE);
        return;
    }
    m_publisher.emplace(std::move(publisher.value()));

    auto event_service = node.as_iox2().service_builder(service_name.value()).event().open_or_create();
    if (event_service.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(event_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto notifier = event_service.value().notifier_builder().create();
    if (notifier.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(notifier.error()));
        error.emplace(ErrorType::NOTIFIER_CREATION_FAILURE);
        return;
    }
    m_notifier.emplace(std::move(notifier.value()));
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

auto PublisherImpl::loan() -> iox::expected<void*, ErrorType> {
    using iox::err;
    using iox::ok;

    auto sample = m_publisher->loan_slice_uninit(m_payload_size);
    if (sample.has_error()) {
        return err(ErrorType::LOAN_FAILURE);
    }

    // Store the sample for later use when publishing
    auto ptr = m_registry.store(std::move(sample.value()));

    return ok(static_cast<void*>(ptr));
}

auto PublisherImpl::return_loan(void* loaned_memory) -> iox::expected<void, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    if (auto result = m_registry.release(static_cast<uint8_t*>(loaned_memory)); result.has_error()) {
        switch (result.error()) {
        case SampleRegistryError::INVALID_PAYLOAD:
            return err(ErrorType::INVALID_PAYLOAD);
        }
    }
    return ok();
}

auto PublisherImpl::publish(void* loaned_memory) -> iox::expected<void, ErrorType> {
    using ::iox::err;
    using ::iox::ok;
    using ::iox2::assume_init;
    using ::iox2::send;

    // Send
    auto sample = m_registry.release(static_cast<uint8_t*>(loaned_memory));
    if (!sample.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("invalid payload pointer");
        return err(ErrorType::INVALID_PAYLOAD);
    }
    if (auto result = send(assume_init(std::move(sample.value()))); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(result.error()));
        return err(ErrorType::SEND_FAILURE);
    }

    // Notify
    if (auto result = m_notifier->notify(); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(result.error()));
        return err(ErrorType::NOTIFICATION_FAILURE);
    }

    return ok();
}

} // namespace rmw::iox2
