// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/subscriber.hpp"

#include "iox2/bb/into.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/attributes.hpp"

namespace rmw::iox2
{

Subscriber::Subscriber(CreationLock,
                       ::iox2::bb::Optional<ErrorType>& error,
                       Node& node,
                       const char* topic,
                       const rosidl_message_type_support_t* type_support,
                       const Qos& qos)
    : m_topic{topic}
    , m_typesupport{type_support}
    , m_service_name{::rmw::iox2::names::topic(topic)}
    , m_qos{qos} {
    auto iox2_service_name = Iceoryx2::ServiceName::create(m_service_name.c_str());

    if (!iox2_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service_name.error()));
        error.emplace(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        return;
    }

    const auto& options = node.context().options();

    // Adopt QoS settings from existing service in adoptive matching mode.
    // If the service does not exist, use QoS provided by caller.
    if (options.qos_matching_mode == QosMatchingMode::ADOPTIVE) {
        if (auto existing = node.iox2().lookup_service<Iceoryx2::ServiceType::Ipc>(
                m_service_name, Iceoryx2::MessagingPattern::PublishSubscribe);
            existing.has_value()) {
            auto adopted_qos =
                TryConvert<Qos>::from(existing.value().static_details.attributes(), ProfileKind::PUBLISH_SUBSCRIBE);
            if (!adopted_qos.has_value()) {
                RMW_IOX2_CHAIN_ERROR_MSG("failed to decode attributes of existing service for adoption");
                error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
                return;
            }
            m_qos = std::move(adopted_qos.value());
        }
    }

    auto verifier = TryConvert<::iox2::AttributeVerifier>::from(m_qos);
    if (!verifier.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to build QoS attribute verifier");
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto iox2_pubsub_service =
        node.iox2()
            .ipc()
            .service_builder(iox2_service_name.value())
            .publish_subscribe<Payload>()
            .max_publishers(options.max_publishers_per_topic.value_or(DEFAULT_MAX_PUBLISHERS_PER_TOPIC))
            .max_subscribers(options.max_subscribers_per_topic.value_or(DEFAULT_MAX_SUBSCRIBERS_PER_TOPIC))
            .max_nodes(options.max_nodes_per_service.value_or(DEFAULT_MAX_NODES_PER_SERVICE))
            .history_size(m_qos.history_size())
            .subscriber_max_buffer_size(m_qos.subscriber_max_buffer_size())
            .enable_safe_overflow(m_qos.enable_safe_overflow())
            .payload_alignment(8) // All ROS2 messages have alignment 8. Maybe?
            .open_or_create_with_attributes(verifier.value());

    if (!iox2_pubsub_service.has_value()) {
        if (iox2_pubsub_service.error() == ::iox2::PublishSubscribeOpenOrCreateError::OpenIncompatibleAttributes) {
            // Caller (the rmw C API layer) formats the per-key diff.
            error.emplace(ErrorType::QOS_INCOMPATIBLE);
        } else {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_pubsub_service.error()));
            error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        }
        return;
    }

    auto iox2_subscriber =
        iox2_pubsub_service.value().subscriber_builder().buffer_size(m_qos.subscriber_max_buffer_size()).create();

    if (!iox2_subscriber.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_subscriber.error()));
        error.emplace(ErrorType::SUBSCRIBER_CREATION_FAILURE);
        return;
    }

    m_iox2_unique_id.emplace(iox2_subscriber->id());
    m_iox2_subscriber.emplace(std::move(iox2_subscriber.value()));
}

auto Subscriber::unique_id() -> const ::iox2::bb::Optional<RawIdType>& {
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

auto Subscriber::qos() const -> const Qos& {
    return m_qos;
}

auto Subscriber::take_copy(void* dest) -> ::iox2::bb::Expected<bool, ErrorType> {
    using ::iox2::bb::err;

    if (auto result = m_iox2_subscriber->receive(); !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::RECV_FAILURE);
    } else {
        auto sample = std::move(result.value());

        if (sample.has_value()) {
            auto payload = sample.value().payload();
            auto number_of_bytes = payload.number_of_bytes();
            std::memcpy(dest, payload.data(), number_of_bytes);
        }

        return sample.has_value();
    }
}

auto Subscriber::take_loan() -> ::iox2::bb::Expected<::iox2::bb::Optional<SubscriberLoan>, ErrorType> {
    using ::iox2::bb::err;
    using ::iox2::bb::Optional;

    auto result = m_iox2_subscriber->receive();
    if (!result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::RECV_FAILURE);
    }
    auto sample = std::move(result.value());

    if (sample.has_value()) {
        auto data = sample->payload().data();
        auto number_of_bytes = sample->payload().number_of_bytes();
        m_registry.store(std::move(sample.value()));

        // Const cast required because of RMW API
        return Optional<SubscriberLoan>(SubscriberLoan{const_cast<uint8_t*>(data), number_of_bytes});
    } else {
        return Optional<SubscriberLoan>{::iox2::bb::NULLOPT};
    }
}

auto Subscriber::return_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    if (auto result = m_registry.release(static_cast<uint8_t*>(loaned_memory)); !result.has_value()) {
        switch (result.error()) {
        case SampleRegistryError::INVALID_PAYLOAD:
            return err(ErrorType::INVALID_PAYLOAD);
        }
    }

    return {};
}

} // namespace rmw::iox2
