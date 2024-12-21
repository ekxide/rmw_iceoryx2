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

constexpr const char* DEFAULT_ENCLAVE = "";

extern "C" {

rmw_ret_t rmw_init_options_init(rmw_init_options_t* init_options, rcutils_allocator_t allocator) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(&allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_INITIALIZED(init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_INITIALIZED(init_options, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    init_options->implementation_identifier = rmw_get_implementation_identifier();
    init_options->allocator = allocator;
    init_options->instance_id = 0;
    init_options->enclave = rcutils_strdup(DEFAULT_ENCLAVE, allocator);
    init_options->impl = const_cast<rmw_init_options_impl_t*>(&INITIALIZED_OPTIONS);

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
    *dst = *src;

    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_fini(rmw_init_options_t* init_options) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(&init_options->allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(init_options->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    *init_options = rmw_get_zero_initialized_init_options();

    return RMW_RET_OK;
}

rmw_ret_t rmw_init(const rmw_init_options_t* init_options, rmw_context_t* context) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(init_options->enclave, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(init_options, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_INITIALIZED(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_INITIALIZED(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(init_options->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::allocate;
    using rmw::iox2::construct;
    using rmw::iox2::create_in_place;
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;

    context->instance_id = init_options->instance_id;
    context->implementation_identifier = rmw_get_implementation_identifier();
    context->options.enclave = rcutils_strdup(init_options->enclave, init_options->allocator);

    auto ptr = allocate<rmw_context_impl_s>();
    if (ptr.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_context_impl_s");
        return RMW_RET_ERROR;
    }

    if (create_in_place<rmw_context_impl_s>(ptr.value(), context->instance_id).has_error()) {
        destruct<rmw_context_impl_s>(ptr.value());
        deallocate(ptr.value());
        RMW_IOX2_CHAIN_ERROR_MSG("failed to construct rmw_context_impl_s");
        return RMW_RET_ERROR;
    }
    context->impl = ptr.value();

    return RMW_RET_OK;
}

rmw_ret_t rmw_shutdown(rmw_context_t* context) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(context->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;

    if (context->impl != nullptr) {
        destruct<rmw_context_impl_s>(context->impl);
        deallocate(context->impl);
        context->impl = nullptr;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_context_fini(rmw_context_t* context) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(context, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(context->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_INITIALIZED(context, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    *context = rmw_get_zero_initialized_context();

    return RMW_RET_OK;
}
}
