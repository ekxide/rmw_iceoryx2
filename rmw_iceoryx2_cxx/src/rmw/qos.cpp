// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/qos_profiles.h"
#include "rmw/ret_types.h"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"

extern "C" {
rmw_ret_t rmw_qos_profile_check_compatible(const rmw_qos_profile_t publisher_profile,
                                           const rmw_qos_profile_t subscription_profile,
                                           rmw_qos_compatibility_type_t* compatibility,
                                           char* reason,
                                           size_t reason_size) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(compatibility, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(reason, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    // All QoS assumed compatible - to be determined at gateways
    *compatibility = RMW_QOS_COMPATIBILITY_OK;

    // Un-terminated char array leads to crashes in rqt_graph
    reason[0] = '\0';

    return RMW_RET_OK;
}
}
