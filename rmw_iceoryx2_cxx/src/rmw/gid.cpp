// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/publisher.hpp"

rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t* rmw_publisher, rmw_gid_t* rmw_gid) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_gid, RMW_RET_INVALID_ARGUMENT);

    // ementation -------------------------------------------------------------------------------
    using ::rmw::iox2::Publisher;
    using ::rmw::iox2::unsafe_cast;

    auto publisher_impl = unsafe_cast<Publisher*>(rmw_publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Publisher");
        return RMW_RET_ERROR;
    }

    if (auto id = publisher_impl.value()->unique_id(); id.has_value()) {
        rmw_gid->implementation_identifier = rmw_get_implementation_identifier();
        std::copy(id.value().data(), id.value().data() + RMW_GID_STORAGE_SIZE, rmw_gid->data);
        return RMW_RET_OK;
    }

    RMW_IOX2_CHAIN_ERROR_MSG("unable to retrieve UniquePortId for Publisher");
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_gid_for_client(const rmw_client_t* client, rmw_gid_t* gid) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(client, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(client->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(gid, RMW_RET_INVALID_ARGUMENT);

    // ementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_compare_gids_equal(const rmw_gid_t* gid1, const rmw_gid_t* gid2, bool* result) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(result, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(gid1->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_IMPLEMENTATION(gid2->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // ementation -------------------------------------------------------------------------------
    // NOTE: In iceoryx2, GIDs for different entities (Publishers, Notifiers, etc.) have unique types.
    //       Thus, these IDs may have the same value but the type system prevents them from being considered equal.
    //       In C, only the raw value is worked with. This might cause problems.

    *result = std::equal(gid1->data, gid1->data + RMW_GID_STORAGE_SIZE, gid2->data);
    return RMW_RET_OK;
}
