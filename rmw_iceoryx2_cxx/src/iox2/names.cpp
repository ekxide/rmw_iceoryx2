// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/names.hpp"

namespace rmw::iox2::names
{


// TODO: Better convention for these names...
std::string context(const uint32_t context_id) {
    return "::ros2::" + std::to_string(context_id) + "::context";
}

std::string node(const uint32_t context_id, const char* name, const char* ns) {
    auto s = "::ros2::" + std::to_string(context_id) + "::node::" + std ::string(ns) + "::" + std::string(name);
    return s;
}

std::string topic(const uint32_t context_id, const char* topic) {
    auto s = "::ros2::" + std::to_string(context_id) + "::topic::" + std::string(topic);
    return s;
}

std::string guard_condition(const uint32_t context_id, const uint32_t guard_condition_id) {
    return "::ros2::" + std::to_string(context_id) + "::guard_condition::" + std::to_string(guard_condition_id);
}

} // namespace rmw::iox2::names
