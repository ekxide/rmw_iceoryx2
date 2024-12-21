// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"
#include "rmw/allocators.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"

extern "C" {
rmw_guard_condition_t* rmw_create_guard_condition(rmw_context_t* rmw_context) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_context, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_context->implementation_identifier, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_context->impl, nullptr);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using GuardConditionImpl = ::rmw::iox2::GuardCondition;
    using ::rmw::iox2::unsafe_cast;

    auto* rmw_guard_condition = rmw_guard_condition_allocate();
    if (rmw_guard_condition == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_guard_condition_t");
        return nullptr;
    }

    rmw_guard_condition->context = rmw_context;
    rmw_guard_condition->implementation_identifier = rmw_get_implementation_identifier();

    auto guard_condition_impl = allocate<GuardConditionImpl>();
    if (guard_condition_impl.has_error()) {
        rmw_guard_condition_free(rmw_guard_condition);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for GuardCondition");
        return nullptr;
    }

    if (create_in_place<GuardConditionImpl>(guard_condition_impl.value(), *rmw_context->impl).has_error()) {
        destruct<GuardConditionImpl>(guard_condition_impl.value());
        deallocate<GuardConditionImpl>(guard_condition_impl.value());
        rmw_guard_condition_free(rmw_guard_condition);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to construct GuardCondition");
        return nullptr;
    }
    rmw_guard_condition->data = guard_condition_impl.value();

    return rmw_guard_condition;
}

rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t* rmw_guard_condition) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_guard_condition, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_guard_condition->implementation_identifier,
                                   RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;
    using GuardConditionImpl = rmw::iox2::GuardCondition;

    if (rmw_guard_condition->data) {
        destruct<GuardConditionImpl>(rmw_guard_condition->data);
        deallocate(rmw_guard_condition->data);
    }
    rmw_guard_condition_free(rmw_guard_condition);

    return RMW_RET_OK;
}

rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t* rmw_guard_condition) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_guard_condition, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_guard_condition->implementation_identifier,
                                   RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using GuardConditionImpl = rmw::iox2::GuardCondition;
    using rmw::iox2::unsafe_cast;

    auto result = RMW_RET_OK;
    unsafe_cast<GuardConditionImpl*>(rmw_guard_condition->data).and_then([&result](auto impl) {
        if (impl->trigger().has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to trigger guard condition");
            result = RMW_RET_ERROR;
        }
    });

    return result;
}
}
