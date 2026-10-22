// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_TOPIC_HPP_
#define RMW_IOX2_RUNTIME_TOPIC_HPP_

#include "iox2/bb/expected.hpp"
#include "iox2/bb/slice.hpp"
#include "iox2/marker.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/message/message_info_header.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"

#include <string>

namespace rmw::iox2
{

using TopicPayload = ::iox2::bb::Slice<::iox2::CustomPayloadMarker>;
using TopicUserHeader = ::rmw_iceoryx2_interoperability::MessageInfoHeader;
using TopicService = Iceoryx2::InterProcess::PublishSubscribe<TopicPayload, TopicUserHeader>;

/// @brief Open the service of a topic, or create it with the QoS of an endpoint
/// @param[in] node The node of the endpoint
/// @param[in] service_name Name of the topic service
/// @param[in] type_support Type support of the messages
/// @param[in,out] qos QoS of the endpoint, replaced by the QoS of the service in adoptive matching mode
/// @return The service of the topic
RMW_PUBLIC auto open_or_create_topic(Node& node,
                                     const std::string& service_name,
                                     const rosidl_message_type_support_t* type_support,
                                     Qos& qos) -> ::iox2::bb::Expected<TopicService, TopicError>;

} // namespace rmw::iox2

#endif // RMW_IOX2_RUNTIME_TOPIC_HPP_
