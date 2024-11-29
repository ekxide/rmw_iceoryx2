// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/allocators.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/allocator.hpp"
#include "rmw_iceoryx2_cxx/create.hpp"
#include "rmw_iceoryx2_cxx/ensure.hpp"
#include "rmw_iceoryx2_cxx/error_message.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"

extern "C" {
rmw_guard_condition_t* rmw_create_guard_condition(rmw_context_t* context) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(context, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(context->implementation_identifier, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(context->impl, nullptr);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ::rmw::iox2::GuardConditionImpl;
    using ::rmw::iox2::unsafe_cast;

    auto* guard_condition = rmw_guard_condition_allocate();
    if (guard_condition == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_guard_condition_t");
        return nullptr;
    }

    guard_condition->context = context;
    guard_condition->implementation_identifier = rmw_get_implementation_identifier();

    auto ptr = allocate<GuardConditionImpl>();
    if (ptr.has_error()) {
        rmw_guard_condition_free(guard_condition);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for GuardConditionImpl");
        return nullptr;
    }

    if (create_in_place<GuardConditionImpl>(ptr.value(), *context->impl).has_error()) {
        destruct<GuardConditionImpl>(ptr.value());
        deallocate<GuardConditionImpl>(ptr.value());
        rmw_guard_condition_free(guard_condition);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to construct GuardConditionImpl");
        return nullptr;
    }
    guard_condition->data = ptr.value();

    return guard_condition;
}

rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t* guard_condition) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(guard_condition->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;
    using rmw::iox2::GuardConditionImpl;

    if (guard_condition->data) {
        destruct<GuardConditionImpl>(guard_condition->data);
        deallocate(guard_condition->data);
    }
    rmw_guard_condition_free(guard_condition);

    return RMW_RET_OK;
}

rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t* guard_condition) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(guard_condition->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::GuardConditionImpl;
    using rmw::iox2::unsafe_cast;

    auto result = RMW_RET_OK;
    unsafe_cast<GuardConditionImpl*>(guard_condition->data).and_then([&result](auto impl) {
        if (impl->trigger().has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to trigger guard condition");
            result = RMW_RET_ERROR;
        }
    });

    return result;
}
}
