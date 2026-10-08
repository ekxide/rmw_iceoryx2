// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/common/names.hpp"

#include <cstring>

#include <unistd.h>

namespace rmw::iox2::names
{

std::string context(const uint32_t context_id) {
    return "ros2://context/" + std::to_string(context_id);
}

std::string node(const uint32_t context_id, const char* name, const char* node_namespace, const char* enclave) {
    auto s = "ros2://context/" + std::to_string(context_id) + "/nodes/";
    if (node_namespace && node_namespace[0] != '\0') {
        s += std::string(node_namespace) + "/";
    }
    s += std::string(name);
    if (enclave && enclave[0] != '\0' && std::strcmp(enclave, "/") != 0) {
        s += std::string("?enclave=") + enclave;
    }
    return s;
}

std::string topic(const char* topic) {
    auto s = "ros2://topics" + std::string(topic);
    return s;
}

std::string graph() {
    return "ros2://graph";
}

std::string service(const char* service) {
    return "ros2://services" + std::string(service);
}

} // namespace rmw::iox2::names
