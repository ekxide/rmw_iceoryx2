// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/publisher.hpp"

#include "iox2/bb/into.hpp"
#include "iox2/bb/slice.hpp"
#include "iox2/message_type_details.hpp"
#include "iox2/type_variant.hpp"
#include "rcutils/time.h"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/message/message_info_header.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/attributes.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/payload_layout.hpp"

#include <cstring>

namespace rmw::iox2
{

Publisher::Publisher(CreationLock,
                     ::iox2::bb::Optional<ErrorType>& error,
                     Node& node,
                     const char* topic,
                     const rosidl_message_type_support_t* type_support,
                     const Qos& qos)
    : m_topic{topic}
    , m_typesupport{type_support}
    , m_unserialized_size{::rmw::iox2::message_size(type_support)}
    , m_is_self_contained{::rmw::iox2::is_self_contained(type_support)}
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

    const auto payload_type_name = ::rmw::iox2::message_type_name(m_typesupport);
    const auto payload_type_details = m_is_self_contained ? ::iox2::TypeDetail(::iox2::TypeVariant::FixedSize,
                                                                               payload_type_name.c_str(),
                                                                               m_unserialized_size,
                                                                               SELF_CONTAINED_PAYLOAD_ALIGNMENT)
                                                          : ::iox2::TypeDetail(::iox2::TypeVariant::Dynamic,
                                                                               payload_type_name.c_str(),
                                                                               SERIALIZED_PAYLOAD_ELEMENT_SIZE,
                                                                               SERIALIZED_PAYLOAD_ALIGNMENT);
    const uint64_t payload_alignment =
        m_is_self_contained ? SELF_CONTAINED_PAYLOAD_ALIGNMENT : SERIALIZED_PAYLOAD_ALIGNMENT;

    auto service_builder = node.iox2()
                               .ipc()
                               .service_builder(iox2_service_name.value())
                               .publish_subscribe<Payload>()
                               .user_header<UserHeader>();

    ::iox2::set_payload_type_details(service_builder, payload_type_details);

    auto iox2_pubsub_service =
        service_builder.resume_build()
            .max_publishers(options.max_publishers_per_topic.value_or(DEFAULT_MAX_PUBLISHERS_PER_TOPIC))
            .max_subscribers(options.max_subscribers_per_topic.value_or(DEFAULT_MAX_SUBSCRIBERS_PER_TOPIC))
            .max_nodes(options.max_nodes_per_service.value_or(DEFAULT_MAX_NODES_PER_SERVICE))
            .history_size(m_qos.history_size())
            .subscriber_max_buffer_size(m_qos.subscriber_max_buffer_size())
            .enable_safe_overflow(m_qos.enable_safe_overflow())
            .payload_alignment(payload_alignment)
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

    auto iox2_publisher =
        iox2_pubsub_service.value()
            .publisher_builder()
            .initial_max_slice_len(m_is_self_contained ? SELF_CONTAINED_PAYLOAD_ELEMENT_COUNT : m_unserialized_size)
            .allocation_strategy(::iox2::AllocationStrategy::PowerOfTwo)
            .create();

    if (!iox2_publisher.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_publisher.error()));
        error.emplace(ErrorType::PUBLISHER_CREATION_FAILURE);
        return;
    }

    m_iox2_unique_id.emplace(iox2_publisher->id());
    m_iox2_publisher.emplace(std::move(iox2_publisher.value()));

    auto iox2_event_service = node.iox2().ipc().service_builder(iox2_service_name.value()).event().open_or_create();
    if (!iox2_event_service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_event_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto iox2_notifier = iox2_event_service.value().notifier_builder().create();
    if (!iox2_notifier.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_notifier.error()));
        error.emplace(ErrorType::NOTIFIER_CREATION_FAILURE);
        return;
    }

    m_iox2_notifier.emplace(std::move(iox2_notifier.value()));
}


auto Publisher::unique_id() -> const ::iox2::bb::Optional<RawIdType>& {
    auto& bytes = m_iox2_unique_id->bytes();
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

auto Publisher::qos() const -> const Qos& {
    return m_qos;
}

auto Publisher::loan(uint64_t number_of_bytes) -> ::iox2::bb::Expected<void*, ErrorType> {
    using ::iox2::bb::err;

    const uint64_t number_of_elements = m_is_self_contained ? SELF_CONTAINED_PAYLOAD_ELEMENT_COUNT : number_of_bytes;
    auto sample = m_iox2_publisher->loan_slice_uninit(number_of_elements);
    if (!sample.has_value()) {
        return err(ErrorType::LOAN_FAILURE);
    }

    // Store the sample for later use when publishing
    auto ptr = m_registry.store(std::move(sample.value()));

    return static_cast<void*>(ptr);
}

auto Publisher::return_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    if (auto result = m_registry.release(static_cast<uint8_t*>(loaned_memory)); !result.has_value()) {
        switch (result.error()) {
        case SampleRegistryError::INVALID_PAYLOAD:
            return err(ErrorType::INVALID_PAYLOAD);
        }
    }
    return {};
}

auto Publisher::publish_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    // Send
    auto sample = m_registry.release(static_cast<uint8_t*>(loaned_memory));
    if (!sample.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("invalid payload pointer");
        return err(ErrorType::INVALID_PAYLOAD);
    }

    populate_message_info(sample.value().user_header_mut());

    if (auto result = Iceoryx2::InterProcess::send<Payload, UserHeader>(std::move(sample.value()));
        !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::SEND_FAILURE);
    }

    // Notify
    if (auto result = m_iox2_notifier->notify(); !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::NOTIFICATION_FAILURE);
    }

    return {};
}

auto Publisher::publish_copy(const void* data, uint64_t number_of_bytes) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    const uint64_t number_of_elements = m_is_self_contained ? SELF_CONTAINED_PAYLOAD_ELEMENT_COUNT : number_of_bytes;
    auto sample = m_iox2_publisher->loan_slice_uninit(number_of_elements);
    if (!sample.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(sample.error()));
        return err(ErrorType::LOAN_FAILURE);
    }

    populate_message_info(sample.value().user_header_mut());
    std::memcpy(sample.value().payload_mut().data(), data, number_of_bytes);

    if (auto result = Iceoryx2::InterProcess::send<Payload, UserHeader>(std::move(sample.value()));
        !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::SEND_FAILURE);
    }

    // Notify
    if (auto result = m_iox2_notifier->notify(); !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::NOTIFICATION_FAILURE);
    }

    return {};
}

void Publisher::populate_message_info(UserHeader& header) {
    rcutils_time_point_value_t now = 0;
    if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
        now = 0;
    }
    header.source_timestamp = now;
    header.publication_sequence_number = m_publication_sequence_number++;
}

} // namespace rmw::iox2
