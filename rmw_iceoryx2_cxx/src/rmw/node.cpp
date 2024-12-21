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

void inline cleanup_node(rmw_node_t* rmw_node) noexcept {
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;
    using rmw::iox2::Node;

    if (rmw_node) {
        deallocate(rmw_node->name);
        deallocate(rmw_node->namespace_);
        if (rmw_node->data) {
            destruct<Node>(rmw_node->data);
            deallocate(rmw_node->data);
        }
        rmw_node_free(rmw_node);
    }
}

} // namespace

rmw_node_t* rmw_create_node(rmw_context_t* rmw_context, const char* name, const char* namespace_) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_context, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(name, nullptr);
    RMW_IOX2_ENSURE_VALID_NODE_NAME(name, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(namespace_, nullptr);
    RMW_IOX2_ENSURE_VALID_NAMESPACE(namespace_, nullptr);
    RMW_IOX2_ENSURE_INITIALIZED(rmw_context, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_context->implementation_identifier, nullptr);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::allocate_copy;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using NodeImpl = ::rmw::iox2::Node;

    RMW_IOX2_LOG_DEBUG("Creating node '%s' in namespace '%s'", name, namespace_);

    rmw_node_t* rmw_node = rmw_node_allocate();
    if (!rmw_node) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node handle");
        return nullptr;
    }
    rmw_node->context = rmw_context;
    rmw_node->implementation_identifier = rmw_get_implementation_identifier();

    auto name_ptr = allocate_copy(name);
    if (name_ptr.has_error()) {
        cleanup_node(rmw_node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node name");
        return nullptr;
    }
    rmw_node->name = name_ptr.value();

    auto namespace_ptr = allocate_copy(namespace_);
    if (namespace_ptr.has_error()) {
        cleanup_node(rmw_node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node namespace");
        return nullptr;
    }
    rmw_node->namespace_ = namespace_ptr.value();

    auto node_impl = allocate<NodeImpl>();
    if (node_impl.has_error()) {
        cleanup_node(rmw_node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for Node");
        return nullptr;
    }

    if (auto construction =
            create_in_place<NodeImpl>(node_impl.value(), *rmw_context->impl, rmw_node->name, rmw_node->namespace_);
        construction.has_error()) {
        destruct<NodeImpl>(node_impl.value());
        deallocate<NodeImpl>(node_impl.value());
        cleanup_node(rmw_node);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to construct Node");
        return nullptr;
    }
    rmw_node->data = node_impl.value();

    return rmw_node;
}

rmw_ret_t rmw_destroy_node(rmw_node_t* rmw_node) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    RMW_IOX2_LOG_DEBUG("Destroying node '%s' in namespace '%s'", rmw_node->name, rmw_node->namespace_);

    cleanup_node(rmw_node);

    return RMW_RET_OK;
}

const rmw_guard_condition_t* rmw_node_get_graph_guard_condition(const rmw_node_t* rmw_node) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node->context, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node->context->impl, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, nullptr);

    // Implementation -------------------------------------------------------------------------------
    using NodeImpl = rmw::iox2::Node;
    using rmw::iox2::unsafe_cast;

    auto* rmw_guard_condition = rmw_guard_condition_allocate();
    if (rmw_guard_condition == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_guard_condition_t");
        return nullptr;
    }

    rmw_guard_condition->implementation_identifier = rmw_get_implementation_identifier();
    rmw_guard_condition->context = rmw_node->context;

    auto node_impl = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (node_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Node");
        return nullptr;
    }

    rmw_guard_condition->data = static_cast<void*>(&node_impl.value()->graph_guard_condition());

    return rmw_guard_condition;
}
}
