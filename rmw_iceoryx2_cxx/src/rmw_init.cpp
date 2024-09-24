// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/check_type_identifiers_match.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw/ret_types.h"
#include "rmw_iceoryx2_cxx/rmw_allocator_helpers.hpp"
#include "rmw_iceoryx2_cxx/rmw_context_impl.hpp"
#include "rmw_iceoryx2_cxx/rmw_identifier.hpp"

extern "C" {

rmw_ret_t rmw_init_options_init(rmw_init_options_t* init_options, rcutils_allocator_t allocator)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
    RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
    if (init_options->implementation_identifier != nullptr)
    {
        RMW_SET_ERROR_MSG("expected zero-initialized init_options");
        return RMW_RET_INVALID_ARGUMENT;
    }
    init_options->implementation_identifier = rmw_get_implementation_identifier();
    init_options->allocator = allocator;
    init_options->instance_id = 0;
    init_options->impl = nullptr; // no implementation-specific data
    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_copy(const rmw_init_options_t* src, rmw_init_options_t* dst)
{
    RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
    RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
    RMW_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_init_options_copy: source options",
                                     src->implementation_identifier,
                                     rmw_get_implementation_identifier(),
                                     return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (dst->implementation_identifier != nullptr)
    {
        RMW_SET_ERROR_MSG("expected zero-initialized dst");
        return RMW_RET_INVALID_ARGUMENT;
    }
    *dst = *src;
    return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_fini(rmw_init_options_t* init_options)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
    RCUTILS_CHECK_ALLOCATOR(&(init_options->allocator), return RMW_RET_INVALID_ARGUMENT);
    RMW_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_init_options_fini: options",
                                     init_options->implementation_identifier,
                                     rmw_get_implementation_identifier(),
                                     return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    *init_options = rmw_get_zero_initialized_init_options();
    return RMW_RET_OK;
}

rmw_ret_t rmw_init(const rmw_init_options_t* options, rmw_context_t* context)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
    RMW_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_init: options",
                                     options->implementation_identifier,
                                     rmw_get_implementation_identifier(),
                                     return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    context->instance_id = options->instance_id;
    context->implementation_identifier = rmw_get_implementation_identifier();

    if (context->impl != nullptr && context->impl->data != nullptr)
    {
        auto initialized = context->impl->is_initialized.load(std::memory_order_acquire);
        if (initialized)
        {
            RMW_SET_ERROR_MSG("rmw_init called on already initialized context");
            return RMW_RET_INVALID_ARGUMENT;
        }
    }

    auto impl_ptr = iox2_rmw::allocate<iox2_rmw::ContextImpl>();
    if (!impl_ptr)
    {
        RMW_SET_ERROR_MSG("failed to allocate memory for rmw context implementation");
        return RMW_RET_ERROR;
    }
    RMW_TRY_PLACEMENT_NEW(impl_ptr, impl_ptr, iox2_rmw::deallocate(impl_ptr), iox2_rmw::ContextImpl);

    // Initialize rmw struct with pointer to ContextImpl
    // TODO: ... there must be a better way to do this
    auto context_impl = iox2_rmw::allocate<rmw_context_impl_s>();
    context_impl->data = impl_ptr;

    context->impl = context_impl;
    context->impl->is_initialized.store(true, std::memory_order_release);

    return RMW_RET_OK;
}

rmw_ret_t rmw_shutdown(rmw_context_t* context)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
    RMW_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_shutdown: context",
                                     context->implementation_identifier,
                                     rmw_get_implementation_identifier(),
                                     return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (!context->impl)
    {
        // nothing to do
        return RMW_RET_OK;
    }
    if (!context->impl->is_initialized.load(std::memory_order_acquire))
    {
        RMW_SET_ERROR_MSG("rmw_shutdown called on uninitialized context");
        return RMW_RET_INVALID_ARGUMENT;
    }

    iox2_rmw::destruct<iox2_rmw::ContextImpl>(context->impl->data);
    iox2_rmw::deallocate(context->impl->data);
    iox2_rmw::deallocate(context->impl);
    context->impl = nullptr;

    return RMW_RET_OK;
}

rmw_ret_t rmw_context_fini(rmw_context_t* context)
{
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
    RMW_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_context_fini: context",
                                     context->implementation_identifier,
                                     rmw_get_implementation_identifier(),
                                     return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    *context = rmw_get_zero_initialized_context();
    return RMW_RET_OK;
}
}
