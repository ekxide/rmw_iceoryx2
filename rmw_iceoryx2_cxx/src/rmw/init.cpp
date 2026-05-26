// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/init.h"
#include "rcutils/strdup.h"
#include "rmw/init_options.h"
#include "rmw/ret_types.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/rmw/identifier.hpp"

#include <charconv>
#include <cstdlib>
#include <cstring>
#include <system_error>

constexpr const char* DEFAULT_ENCLAVE = "";

namespace
{

/// Reads `name` from the environment. Empty or unset leaves `out` unchanged.
/// Any non-numeric / out-of-range value is reported via the error chain.
auto parse_size_env(const char* name, ::iox2::bb::Optional<size_t>& out) -> rmw_ret_t {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return RMW_RET_OK;
    }

    const char* end = raw + std::strlen(raw);
    size_t value = 0;
    auto result = std::from_chars(raw, end, value);
    if (result.ec != std::errc{} || result.ptr != end) {
        RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING(
            "%s has invalid value '%s'; expected a non-negative integer", name, raw);
        return RMW_RET_INVALID_ARGUMENT;
    }
    out = value;

    return RMW_RET_OK;
}

/// Reads `RMW_IOX2_QOS_MATCHING`. Empty or unset leaves `out` unchanged.
/// Any value other than `strict` / `adoptive` is reported via the error chain.
auto parse_qos_matching_env(::rmw::iox2::QosMatchingMode& out) -> rmw_ret_t {
    const char* raw = std::getenv("RMW_IOX2_QOS_MATCHING");
    if (raw == nullptr || raw[0] == '\0') {
        return RMW_RET_OK;
    }

    if (std::strcmp(raw, "strict") == 0) {
        out = ::rmw::iox2::QosMatchingMode::STRICT;
        return RMW_RET_OK;
    }
    if (std::strcmp(raw, "adoptive") == 0) {
        out = ::rmw::iox2::QosMatchingMode::ADOPTIVE;
        return RMW_RET_OK;
    }
    RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING(
        "RMW_IOX2_QOS_MATCHING has invalid value '%s'; expected 'strict' or 'adoptive'", raw);

    return RMW_RET_INVALID_ARGUMENT;
}

} // namespace

extern "C" {

rmw_ret_t rmw_init_options_init(rmw_init_options_t* init_options, rcutils_allocator_t allocator) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(&allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_INITIALIZED(init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_INITIALIZED(init_options, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::allocate;
    using rmw::iox2::construct;
    using rmw::iox2::deallocate;

    auto qos_matching_mode = ::rmw::iox2::QosMatchingMode::STRICT;
    if (auto result = parse_qos_matching_env(qos_matching_mode); result != RMW_RET_OK) {
        return result;
    }
    ::iox2::bb::Optional<size_t> max_publishers_per_topic;
    if (auto result = parse_size_env("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC", max_publishers_per_topic);
        result != RMW_RET_OK) {
        return result;
    }
    ::iox2::bb::Optional<size_t> max_subscribers_per_topic;
    if (auto result = parse_size_env("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC", max_subscribers_per_topic);
        result != RMW_RET_OK) {
        return result;
    }
    ::iox2::bb::Optional<size_t> max_nodes_per_service;
    if (auto result = parse_size_env("RMW_IOX2_MAX_NODES_PER_SERVICE", max_nodes_per_service); result != RMW_RET_OK) {
        return result;
    }

    auto impl_ptr = allocate<rmw_init_options_impl_s>();
    if (!impl_ptr.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_init_options_impl_s");
        return RMW_RET_BAD_ALLOC;
    }
    if (!construct<rmw_init_options_impl_s>(impl_ptr.value()).has_value()) {
        deallocate(impl_ptr.value());
        RMW_IOX2_CHAIN_ERROR_MSG("failed to construct rmw_init_options_impl_s");
        return RMW_RET_ERROR;
    }

    impl_ptr.value()->qos_matching_mode = qos_matching_mode;
    impl_ptr.value()->max_publishers_per_topic = max_publishers_per_topic;
    impl_ptr.value()->max_subscribers_per_topic = max_subscribers_per_topic;
    impl_ptr.value()->max_nodes_per_service = max_nodes_per_service;

    init_options->implementation_identifier = rmw_get_implementation_identifier();
    init_options->allocator = allocator;
    init_options->instance_id = 0;
    init_options->enclave = rcutils_strdup(DEFAULT_ENCLAVE, allocator);
    init_options->impl = impl_ptr.value();

    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_copy(const rmw_init_options_t* src, rmw_init_options_t* dst) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(src, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(dst, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_INITIALIZED(src, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_INITIALIZED(dst, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(src->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::allocate;
    using rmw::iox2::construct;
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;

    auto impl_ptr = allocate<rmw_init_options_impl_s>();
    if (!impl_ptr.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_init_options_impl_s");
        return RMW_RET_BAD_ALLOC;
    }
    if (!construct<rmw_init_options_impl_s>(impl_ptr.value(), *src->impl).has_value()) {
        deallocate(impl_ptr.value());
        RMW_IOX2_CHAIN_ERROR_MSG("failed to copy-construct rmw_init_options_impl_s");
        return RMW_RET_ERROR;
    }

    char* enclave = rcutils_strdup(src->enclave, src->allocator);
    if (enclave == nullptr) {
        destruct<rmw_init_options_impl_s>(impl_ptr.value());
        deallocate(impl_ptr.value());
        RMW_IOX2_CHAIN_ERROR_MSG("failed to duplicate enclave string");
        return RMW_RET_BAD_ALLOC;
    }

    *dst = *src;
    dst->impl = impl_ptr.value();
    dst->enclave = enclave;

    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_fini(rmw_init_options_t* rmw_init_options) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(&rmw_init_options->allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(rmw_init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_init_options->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;

    if (rmw_init_options->enclave != nullptr) {
        rmw_init_options->allocator.deallocate(rmw_init_options->enclave, rmw_init_options->allocator.state);
    }
    if (rmw_init_options->impl != nullptr) {
        destruct<rmw_init_options_impl_s>(rmw_init_options->impl);
        deallocate(rmw_init_options->impl);
    }

    *rmw_init_options = rmw_get_zero_initialized_init_options();

    return RMW_RET_OK;
}

rmw_ret_t rmw_init(const rmw_init_options_t* rmw_init_options, rmw_context_t* context) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_init_options->enclave, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(rmw_init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_INITIALIZED(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_INITIALIZED(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_init_options->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::allocate;
    using rmw::iox2::construct;
    using rmw::iox2::create_in_place;
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;

    context->instance_id = rmw_init_options->instance_id;
    context->implementation_identifier = rmw_get_implementation_identifier();
    context->options.enclave = rcutils_strdup(rmw_init_options->enclave, rmw_init_options->allocator);

    auto ptr = allocate<rmw_context_impl_s>();
    if (!ptr.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_context_impl_s");
        return RMW_RET_ERROR;
    }

    if (!create_in_place<rmw_context_impl_s>(ptr.value(), context->instance_id, *rmw_init_options->impl).has_value()) {
        destruct<rmw_context_impl_s>(ptr.value());
        deallocate(ptr.value());
        RMW_IOX2_CHAIN_ERROR_MSG("failed to construct rmw_context_impl_s");
        return RMW_RET_ERROR;
    }
    context->impl = ptr.value();

    return RMW_RET_OK;
}

rmw_ret_t rmw_shutdown(rmw_context_t* rmw_context) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(rmw_context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_context->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;

    if (rmw_context->impl != nullptr) {
        destruct<rmw_context_impl_s>(rmw_context->impl);
        deallocate(rmw_context->impl);
        rmw_context->impl = nullptr;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_context_fini(rmw_context_t* rmw_context) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(rmw_context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_context->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_INITIALIZED(rmw_context, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    *rmw_context = rmw_get_zero_initialized_context();

    return RMW_RET_OK;
}
}
