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
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"

extern "C" {
rmw_guard_condition_t* rmw_create_guard_condition(rmw_context_t* context) {
    using rmw::iox2::unsafe_cast;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(context->impl, nullptr);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_create_node: context",
                                          context->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return nullptr);

    auto* guard_condition = rmw_guard_condition_allocate();
    if (guard_condition == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for guard condition");
        return nullptr;
    }

    guard_condition->context = context;
    guard_condition->implementation_identifier = rmw_get_implementation_identifier();

    guard_condition->data = context->impl->create_guard_condition();
    if (guard_condition->data == nullptr) {
        rmw_guard_condition_free(guard_condition);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create guard condition impl");
        return nullptr;
    }

    return guard_condition;
}

rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t* guard_condition) {
    using rmw::iox2::GuardConditionImpl;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_destroy_guard_condition: guard_condition",
                                          guard_condition->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_ERROR);

    if (guard_condition->data) {
        rmw::iox2::destruct<GuardConditionImpl>(guard_condition->data);
        rmw::iox2::deallocate(guard_condition->data);
    }

    rmw_guard_condition_free(guard_condition);

    return RMW_RET_OK;
}

rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t* guard_condition) {
    using rmw::iox2::GuardConditionImpl;
    using rmw::iox2::unsafe_cast;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_trigger_guard_condition: guard_condition",
                                          guard_condition->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_ERROR);

    rmw_ret_t result = RMW_RET_ERROR;
    unsafe_cast<GuardConditionImpl*>(guard_condition->data).and_then([&result](auto impl) {
        impl->trigger();
        result = RMW_RET_OK;
    });

    return result;
}
}
