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
#include "rmw_iceoryx2_cxx/allocator.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/publisher_impl.hpp"

rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t* publisher, rmw_gid_t* gid) {
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_get_gid_for_publisher: publisher",
                                          publisher->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    using ::rmw::iox2::PublisherImpl;
    using ::rmw::iox2::unsafe_cast;

    auto publisher_impl = unsafe_cast<PublisherImpl*>(publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve PublisherImpl");
        return RMW_RET_ERROR;
    }

    if (auto id = publisher_impl.value()->unique_id(); id.has_value()) {
        gid->implementation_identifier = rmw_get_implementation_identifier();
        std::copy(id.value().data(), id.value().data() + RMW_GID_STORAGE_SIZE, gid->data);
        return RMW_RET_OK;
    }

    RMW_IOX2_CHAIN_ERROR_MSG("unable to retrieve UniquePortId for PublisherImpl");
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_gid_for_client(const rmw_client_t* client, rmw_gid_t* gid) {
    IOX_TODO();
}

rmw_ret_t rmw_compare_gids_equal(const rmw_gid_t* gid1, const rmw_gid_t* gid2, bool* result) {
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_compare_gids_equal: gid1",
                                          gid1->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_compare_gids_equal: gid2",
                                          gid2->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);


    // NOTE: In iceoryx2, GIDs for different entities (Publishers, Notifiers, etc.) have unique types.
    //       Thus, these IDs may have the same value but the type system prevents them from being considered equal.
    //       In C, only the raw value is worked with. This might cause problems.

    *result = std::equal(gid1->data, gid1->data + RMW_GID_STORAGE_SIZE, gid2->data);
    return RMW_RET_OK;
}
