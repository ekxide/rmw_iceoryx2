// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/subscriber.hpp"

#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"

namespace rmw::iox2
{

Subscriber::Subscriber(CreationLock,
                       iox::optional<ErrorType>& error,
                       Node& node,
                       const char* topic,
                       const rosidl_message_type_support_t* type_support)
    : m_topic{topic}
    , m_typesupport{type_support}
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
                                   .history_size(10)
                                   .subscriber_max_buffer_size(10)
                                   .payload_alignment(8) // All ROS2 messages have alignment 8. Maybe?
                                   .open_or_create();    // TODO: set attribute for ROS typename

    if (iox2_pubsub_service.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(iox2_pubsub_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto iox2_subscriber = iox2_pubsub_service.value().subscriber_builder().create();

    if (iox2_subscriber.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(iox2_subscriber.error()));
        error.emplace(ErrorType::SUBSCRIBER_CREATION_FAILURE);
        return;
    }

    m_iox2_unique_id.emplace(iox2_subscriber->id());
    m_iox2_subscriber.emplace(std::move(iox2_subscriber.value()));
}

auto Subscriber::unique_id() -> const iox::optional<RawIdType>& {
    auto& bytes = m_iox2_unique_id->bytes();
    return bytes;
}

auto Subscriber::topic() const -> const std::string& {
    return m_topic;
}

auto Subscriber::typesupport() const -> const rosidl_message_type_support_t* {
    return m_typesupport;
}

auto Subscriber::service_name() const -> const std::string& {
    return m_service_name;
}

auto Subscriber::take_copy(void* dest) -> iox::expected<bool, ErrorType> {
    using iox::err;
    using iox::nullopt;
    using iox::ok;
    using iox::optional;

    if (auto result = m_iox2_subscriber->receive(); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(result.error()));
        return err(ErrorType::RECV_FAILURE);
    } else {
        auto sample = std::move(result.value());

        if (sample.has_value()) {
            auto payload = sample.value().payload();
            auto number_of_bytes = payload.number_of_bytes();
            std::memcpy(dest, payload.data(), number_of_bytes);
        }

        return ok(sample.has_value());
    }
}

auto Subscriber::take_loan() -> iox::expected<iox::optional<SubscriberLoan>, ErrorType> {
    using iox::err;
    using iox::nullopt;
    using iox::ok;
    using iox::optional;

    auto result = m_iox2_subscriber->receive();
    if (result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(result.error()));
        return err(ErrorType::RECV_FAILURE);
    }
    auto sample = std::move(result.value());

    if (sample.has_value()) {
        auto data = sample->payload().data();
        auto number_of_bytes = sample->payload().number_of_bytes();
        m_registry.store(std::move(sample.value()));

        // Const cast required because of RMW API
        return ok(optional<SubscriberLoan>({const_cast<uint8_t*>(data), number_of_bytes}));
    } else {
        return ok(optional<SubscriberLoan>{nullopt});
    }
}

auto Subscriber::return_loan(void* loaned_memory) -> iox::expected<void, ErrorType> {
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

} // namespace rmw::iox2
