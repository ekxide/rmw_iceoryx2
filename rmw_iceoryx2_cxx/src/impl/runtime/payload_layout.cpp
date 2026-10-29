// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/payload_layout.hpp"

#include "iox2/type_variant.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"

namespace rmw::iox2
{

auto payload_type_details(const rosidl_message_type_support_t* type_support) -> ::iox2::TypeDetail {
    const auto type_name = message_type_name(type_support);
    if (is_self_contained(type_support)) {
        return ::iox2::TypeDetail(::iox2::TypeVariant::FixedSize,
                                  type_name.c_str(),
                                  message_size(type_support),
                                  SELF_CONTAINED_PAYLOAD_ALIGNMENT);
    }
    return ::iox2::TypeDetail(
        ::iox2::TypeVariant::Dynamic, type_name.c_str(), SERIALIZED_PAYLOAD_ELEMENT_SIZE, SERIALIZED_PAYLOAD_ALIGNMENT);
}

} // namespace rmw::iox2
