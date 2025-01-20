// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/expected.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rmw::iox2
{

auto serialize(const void* ros_message,
               const rosidl_message_type_support_t* type_support,
               void* buffer,
               size_t buffer_size) -> iox::expected<void, SerializationError>;

auto deserialize(const void* serialized_message,
                 const size_t serialized_size,
                 const rosidl_message_type_support_t* typesupport,
                 void* ros_message) -> iox::expected<void, DeserializationError>;

} // namespace rmw::iox2
