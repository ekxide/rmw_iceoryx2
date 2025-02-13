// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_MESSAGE_TYPESUPPORT_HPP_
#define RMW_IOX2_MESSAGE_TYPESUPPORT_HPP_

#include "rmw/visibility_control.h"
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

#define RMW_ICEORYX2_CXX_TYPESUPPORT_C rosidl_typesupport_fastrtps_c__identifier
#define RMW_ICEORYX2_CXX_TYPESUPPORT_CPP rosidl_typesupport_fastrtps_cpp::typesupport_identifier


/// @brief Retrieves a handle for the desired typesupport to use, if supported by the ros message typesupport provided
/// @param type_support The typesupport of a ROS message
/// @param identifier Identifier for the desired typesupport to use
/// @return Pointer to the desired type support structure if found, nullptr if not supported
///
/// This function traverses the type support structure to find the matching identifier.
/// If the ros message's type support doesn't match the identifier, it continues searching
/// through the type support hierarchy until a match is found or the end is reached.
RMW_PUBLIC const rosidl_message_type_support_t* get_handle(const rosidl_message_type_support_t* type_support,
                                                           const char* identifier);

#endif
