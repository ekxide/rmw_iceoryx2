// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/publisher.hpp"

#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"

namespace rmw::iox2
{

Publisher::Publisher(CreationLock,
                     iox::optional<ErrorType>& error,
                     Node& node,
                     const char* topic,
                     const rosidl_message_type_support_t* type_support)
    : m_topic{topic}
    , m_typesupport{type_support}
    , m_unserialized_size{::rmw::iox2::message_size(type_support)}
    , m_service_name{::rmw::iox2::names::topic(topic)} {
    auto iox2_service_name = Iceoryx2::ServiceName::create(m_service_name.c_str());
    if (iox2_service_name.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(iox2_service_name.error()));
        error.emplace(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        return;
    }

    auto iox2_pubsub_service = node.iox2()
                                   .ipc()
                                   .service_builder(iox2_service_name.value())
                                   .publish_subscribe<Payload>()
                                   // TODO: make configurable
                                   .max_publishers(64)
                                   .max_subscribers(64)
                                   .payload_alignment(8) // All ROS2 messages have alignment 8. Maybe?
                                   .open_or_create();    // TODO: set attribute for ROS typename

    if (iox2_pubsub_service.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(iox2_pubsub_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto publisher = iox2_pubsub_service.value()
                         .publisher_builder()
                         .initial_max_slice_len(::rmw::iox2::message_size(m_typesupport))
                         .create();
    if (publisher.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(publisher.error()));
        error.emplace(ErrorType::PUBLISHER_CREATION_FAILURE);
        return;
    }
    m_iox_unique_id.emplace(publisher->id());
    m_iox2_publisher.emplace(std::move(publisher.value()));

    auto iox2_event_service = node.iox2().ipc().service_builder(iox2_service_name.value()).event().open_or_create();
    if (iox2_event_service.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(iox2_event_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto notifier = iox2_event_service.value().notifier_builder().create();
    if (notifier.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(notifier.error()));
        error.emplace(ErrorType::NOTIFIER_CREATION_FAILURE);
        return;
    }
    m_iox2_notifier.emplace(std::move(notifier.value()));
}


auto Publisher::unique_id() -> const iox::optional<RawIdType>& {
    auto& bytes = m_iox_unique_id->bytes();
    return bytes;
}

auto Publisher::topic() const -> const std::string& {
    return m_topic;
}

auto Publisher::typesupport() const -> const rosidl_message_type_support_t* {
    return m_typesupport;
}


auto Publisher::unserialized_size() const -> uint64_t {
    return m_unserialized_size;
}

auto Publisher::service_name() const -> const std::string& {
    return m_service_name;
}

auto Publisher::loan(uint64_t num_bytes) -> iox::expected<void*, ErrorType> {
    using iox::err;
    using iox::ok;

    auto sample = m_iox2_publisher->loan_slice_uninit(num_bytes);
    if (sample.has_error()) {
        return err(ErrorType::LOAN_FAILURE);
    }

    // Store the sample for later use when publishing
    auto ptr = m_registry.store(std::move(sample.value()));

    return ok(static_cast<void*>(ptr));
}

auto Publisher::return_loan(void* loaned_memory) -> iox::expected<void, ErrorType> {
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

auto Publisher::publish_loan(void* loaned_memory) -> iox::expected<void, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    // Send
    auto sample = m_registry.release(static_cast<uint8_t*>(loaned_memory));
    if (!sample.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("invalid payload pointer");
        return err(ErrorType::INVALID_PAYLOAD);
    }
    if (auto result = Iceoryx2::InterProcess::send<Payload>(std::move(sample.value())); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(result.error()));
        return err(ErrorType::SEND_FAILURE);
    }

    // Notify
    if (auto result = m_iox2_notifier->notify(); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(result.error()));
        return err(ErrorType::NOTIFICATION_FAILURE);
    }

    return ok();
}

auto Publisher::publish_copy(const void* msg, uint64_t size) -> iox::expected<void, ErrorType> {
    using ::iox::err;
    using ::iox::ImmutableSlice;
    using ::iox::ok;

    // Send
    auto payload = ImmutableSlice<uint8_t>{static_cast<const uint8_t*>(msg), size};

    if (auto result = m_iox2_publisher->send_slice_copy(payload); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(result.error()));
        return err(ErrorType::SEND_FAILURE);
    }

    // Notify
    if (auto result = m_iox2_notifier->notify(); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(result.error()));
        return err(ErrorType::NOTIFICATION_FAILURE);
    }

    return ok();
}

} // namespace rmw::iox2
