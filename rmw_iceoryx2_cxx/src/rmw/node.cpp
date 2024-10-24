// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/check_type_identifiers_match.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/allocator_helpers.hpp"
#include "rmw_iceoryx2_cxx/create.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/names.hpp"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"
#include "rmw_iceoryx2_cxx/rmw/identifier.hpp"

extern "C" {

namespace
{

void inline destroy_node_impl(rmw_node_t* node) noexcept {
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;
    using rmw::iox2::NodeImpl;

    if (node) {
        deallocate(node->name);
        deallocate(node->namespace_);
        if (node->data) {
            destruct<NodeImpl>(node->data);
            deallocate(node->data);
        }
        rmw_node_free(node);
    }
}

} // namespace

rmw_node_t* rmw_create_node(rmw_context_t* context, const char* name, const char* namespace_) {
    using rmw::iox2::allocate;
    using rmw::iox2::allocate_copy;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using rmw::iox2::NodeImpl;
    namespace names = rmw::iox2::names;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(name, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(namespace_, nullptr);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_create_node: context",
                                          context->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return nullptr);

    rmw_node_t* node = rmw_node_allocate();
    if (!node) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node handle");
        return nullptr;
    }
    node->context = context;
    node->implementation_identifier = rmw_get_implementation_identifier();

    auto name_copy = allocate_copy(name);
    if (name_copy.has_error()) {
        destroy_node_impl(node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node name");
        return nullptr;
    }
    node->name = name_copy.value();

    auto namespace_copy = allocate_copy(namespace_);
    if (namespace_copy.has_error()) {
        destroy_node_impl(node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node namespace");
        return nullptr;
    }
    node->namespace_ = namespace_copy.value();

    auto ptr = allocate<NodeImpl>();
    if (ptr.has_error()) {
        destroy_node_impl(node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for NodeImpl");
        return nullptr;
    }

    if (auto construction = create_in_place<NodeImpl>(
            ptr.value(), names::node(context->instance_id, node->name, node->namespace_).c_str());
        construction.has_error()) {
        destruct<NodeImpl>(ptr.value());
        deallocate<NodeImpl>(ptr.value());
        destroy_node_impl(node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to construct NodeImpl");
        return nullptr;
    }
    node->data = ptr.value();

    // TODO: graph guard condition

    return node;
}

rmw_ret_t rmw_destroy_node(rmw_node_t* node) {
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_destroy_node: node",
                                          node->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    destroy_node_impl(node);

    return RMW_RET_OK;
}

const rmw_guard_condition_t* rmw_node_get_graph_guard_condition(const rmw_node_t*) {
    return nullptr;
}
}
