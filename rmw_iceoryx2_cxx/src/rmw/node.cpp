// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"
#include "rmw/allocators.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/rmw/identifier.hpp"

extern "C" {

namespace
{

void inline destroy_node_impl(rmw_node_t* node) noexcept {
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;
    using rmw::iox2::Node;

    if (node) {
        deallocate(node->name);
        deallocate(node->namespace_);
        if (node->data) {
            destruct<Node>(node->data);
            deallocate(node->data);
        }
        rmw_node_free(node);
    }
}

} // namespace

rmw_node_t* rmw_create_node(rmw_context_t* context, const char* name, const char* namespace_) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(context, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(name, nullptr);
    RMW_IOX2_ENSURE_VALID_NODE_NAME(name, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(namespace_, nullptr);
    RMW_IOX2_ENSURE_VALID_NAMESPACE(namespace_, nullptr);
    RMW_IOX2_ENSURE_INITIALIZED(context, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(context->implementation_identifier, nullptr);

    // ementation -------------------------------------------------------------------------------
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::allocate_copy;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ::rmw::iox2::Node;

    RMW_IOX2_LOG_DEBUG("Creating node '%s' in namespace '%s'", name, namespace_);

    rmw_node_t* node = rmw_node_allocate();
    if (!node) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node handle");
        return nullptr;
    }
    node->context = context;
    node->implementation_identifier = rmw_get_implementation_identifier();

    auto name_ptr = allocate_copy(name);
    if (name_ptr.has_error()) {
        destroy_node_impl(node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node name");
        return nullptr;
    }
    node->name = name_ptr.value();

    auto namespace_ptr = allocate_copy(namespace_);
    if (namespace_ptr.has_error()) {
        destroy_node_impl(node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node namespace");
        return nullptr;
    }
    node->namespace_ = namespace_ptr.value();

    auto ptr = allocate<Node>();
    if (ptr.has_error()) {
        destroy_node_impl(node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for Node");
        return nullptr;
    }

    if (auto construction = create_in_place<Node>(ptr.value(), *context->impl, node->name, node->namespace_);
        construction.has_error()) {
        destruct<Node>(ptr.value());
        deallocate<Node>(ptr.value());
        destroy_node_impl(node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to construct Node");
        return nullptr;
    }
    node->data = ptr.value();

    return node;
}

rmw_ret_t rmw_destroy_node(rmw_node_t* node) {
    // Invariants ----------------------------------------------------------------------------------

    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // ementation -------------------------------------------------------------------------------

    RMW_IOX2_LOG_DEBUG("Destroying node '%s' in namespace '%s'", node->name, node->namespace_);

    destroy_node_impl(node);

    return RMW_RET_OK;
}

const rmw_guard_condition_t* rmw_node_get_graph_guard_condition(const rmw_node_t* node) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(node->context, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(node->context->impl, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, nullptr);

    // ementation -------------------------------------------------------------------------------
    using rmw::iox2::Node;
    using rmw::iox2::unsafe_cast;

    auto* guard_condition = rmw_guard_condition_allocate();
    if (guard_condition == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_guard_condition_t");
        return nullptr;
    }

    guard_condition->implementation_identifier = rmw_get_implementation_identifier();
    guard_condition->context = node->context;

    auto node_impl = unsafe_cast<Node*>(node->data);
    if (node_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Node");
        return nullptr;
    }

    guard_condition->data = static_cast<void*>(&node_impl.value()->graph_guard_condition());

    return guard_condition;
}
}
