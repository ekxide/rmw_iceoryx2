// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_PAYLOAD_LAYOUT_HPP_
#define RMW_IOX2_RUNTIME_PAYLOAD_LAYOUT_HPP_

#include "iox2/message_type_details.hpp"
#include "rmw/visibility_control.h"
#include "rosidl_runtime_c/message_type_support_struct.h"

#include <cstdint>

// Payload layout values shared by all endpoints. They must agree on both ends or
// iceoryx2 rejects the service as incompatible.
namespace rmw::iox2
{

/// A self-contained message is a single fixed-size element.
constexpr uint64_t SELF_CONTAINED_PAYLOAD_ELEMENT_COUNT = 1U;

/// Maximum alignment of an unserialized ROS 2 message.
constexpr uint64_t SELF_CONTAINED_PAYLOAD_ALIGNMENT = 8U;

/// Serialized payloads are CDR byte streams: one byte per element.
constexpr uint64_t SERIALIZED_PAYLOAD_ELEMENT_SIZE = 1U;

/// CDR alignment is relative to the buffer start, so byte alignment suffices.
constexpr uint64_t SERIALIZED_PAYLOAD_ALIGNMENT = 1U;

/// @brief The iceoryx2 payload type of the message: the struct if it is self-contained, CDR bytes otherwise.
RMW_PUBLIC auto payload_type_details(const rosidl_message_type_support_t* type_support) -> ::iox2::TypeDetail;

} // namespace rmw::iox2

#endif // RMW_IOX2_RUNTIME_PAYLOAD_LAYOUT_HPP_
