// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/topic.hpp"

#include "iox2/bb/into.hpp"
#include "iox2/message_type_details.hpp"
#include "iox2/type_variant.hpp"
#include "rmw_iceoryx2_cxx/impl/common/attributes.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/payload_layout.hpp"

#include <cstdint>

namespace rmw::iox2
{

namespace
{

constexpr uint32_t OPEN_ATTEMPTS = 2U;

auto existing_topic_qos(Iceoryx2& iox2, const std::string& service_name)
    -> ::iox2::bb::Expected<::iox2::bb::Optional<Qos>, QosError> {
    using ::iox2::bb::err;

    auto existing =
        iox2.lookup_service<Iceoryx2::ServiceType::Ipc>(service_name, Iceoryx2::MessagingPattern::PublishSubscribe);
    if (!existing.has_value()) {
        return ::iox2::bb::Optional<Qos>{::iox2::bb::NULLOPT};
    }

    auto existing_qos =
        TryConvert<Qos>::from(existing.value().static_details.attributes(), ProfileKind::PUBLISH_SUBSCRIBE);
    if (!existing_qos.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to decode attributes of existing service");
        return err(existing_qos.error());
    }
    return ::iox2::bb::Optional<Qos>{std::move(existing_qos.value())};
}

} // namespace

auto open_or_create_topic(Node& node,
                          const std::string& service_name,
                          const rosidl_message_type_support_t* type_support,
                          Qos& qos) -> ::iox2::bb::Expected<TopicService, TopicError> {
    using ::iox2::bb::err;

    auto iox2_service_name = Iceoryx2::ServiceName::create(service_name.c_str());
    if (!iox2_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service_name.error()));
        return err(TopicError::SERVICE_CREATION_FAILURE);
    }

    const auto& options = node.context().options();
    const uint64_t max_depth = options.max_depth_per_topic.value_or(DEFAULT_MAX_DEPTH_PER_TOPIC);

    const bool is_self_contained = ::rmw::iox2::is_self_contained(type_support);
    const auto payload_type_name = ::rmw::iox2::message_type_name(type_support);
    const auto payload_type_details = is_self_contained ? ::iox2::TypeDetail(::iox2::TypeVariant::FixedSize,
                                                                             payload_type_name.c_str(),
                                                                             ::rmw::iox2::message_size(type_support),
                                                                             SELF_CONTAINED_PAYLOAD_ALIGNMENT)
                                                        : ::iox2::TypeDetail(::iox2::TypeVariant::Dynamic,
                                                                             payload_type_name.c_str(),
                                                                             SERIALIZED_PAYLOAD_ELEMENT_SIZE,
                                                                             SERIALIZED_PAYLOAD_ALIGNMENT);
    const uint64_t payload_alignment =
        is_self_contained ? SELF_CONTAINED_PAYLOAD_ALIGNMENT : SERIALIZED_PAYLOAD_ALIGNMENT;

    for (uint32_t attempt = 1U;; ++attempt) {
        auto existing_qos = existing_topic_qos(node.iox2(), service_name);
        if (!existing_qos.has_value()) {
            return err(TopicError::SERVICE_CREATION_FAILURE);
        }

        // Adopt QoS settings from existing service in adoptive matching mode.
        // If the service does not exist, use QoS provided by caller.
        auto service_qos = qos;
        switch (options.qos_matching_mode) {
        case QosMatchingMode::STRICT:
            break;
        case QosMatchingMode::ROS:
            service_qos = existing_qos.value().value_or(qos);
            break;
        case QosMatchingMode::ADOPTIVE:
            qos = existing_qos.value().value_or(qos);
            service_qos = qos;
            break;
        }

        // The type hash is stored alongside QoS so graph introspection can report it.
        auto verifier =
            TryConvert<::iox2::AttributeVerifier>::from(service_qos, ::rmw::iox2::message_type_hash(type_support));
        if (!verifier.has_value()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to build service attribute verifier");
            return err(TopicError::SERVICE_CREATION_FAILURE);
        }

        auto service_builder = node.iox2()
                                   .ipc()
                                   .service_builder(iox2_service_name.value())
                                   .publish_subscribe<TopicPayload>()
                                   .user_header<TopicUserHeader>();

        ::iox2::set_payload_type_details(service_builder, payload_type_details);

        auto iox2_pubsub_service =
            service_builder.resume_build()
                .max_publishers(options.max_publishers_per_topic.value_or(DEFAULT_MAX_PUBLISHERS_PER_TOPIC))
                .max_subscribers(options.max_subscribers_per_topic.value_or(DEFAULT_MAX_SUBSCRIBERS_PER_TOPIC))
                .max_nodes(options.max_nodes_per_service.value_or(DEFAULT_MAX_NODES_PER_SERVICE))
                .history_size(max_depth)
                .subscriber_max_buffer_size(max_depth)
                .enable_safe_overflow(qos.enable_safe_overflow())
                .payload_alignment(payload_alignment)
                .open_or_create_with_attributes(verifier.value());

        if (iox2_pubsub_service.has_value()) {
            return std::move(iox2_pubsub_service.value());
        }
        if (iox2_pubsub_service.error() != ::iox2::PublishSubscribeOpenOrCreateError::OpenIncompatibleAttributes) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_pubsub_service.error()));
            return err(TopicError::SERVICE_CREATION_FAILURE);
        }
        if (options.qos_matching_mode == QosMatchingMode::STRICT || attempt == OPEN_ATTEMPTS) {
            return err(TopicError::QOS_INCOMPATIBLE);
        }
    }
}

} // namespace rmw::iox2
