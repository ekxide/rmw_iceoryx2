// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/event.h"

namespace
{

TEST(RmwEventTest, no_event_type_is_supported) {
    for (const auto type : {RMW_EVENT_LIVELINESS_CHANGED,
                            RMW_EVENT_REQUESTED_DEADLINE_MISSED,
                            RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE,
                            RMW_EVENT_MESSAGE_LOST,
                            RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE,
                            RMW_EVENT_SUBSCRIPTION_MATCHED,
                            RMW_EVENT_LIVELINESS_LOST,
                            RMW_EVENT_OFFERED_DEADLINE_MISSED,
                            RMW_EVENT_OFFERED_QOS_INCOMPATIBLE,
                            RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE,
                            RMW_EVENT_PUBLICATION_MATCHED,
                            RMW_EVENT_INVALID}) {
        EXPECT_FALSE(rmw_event_type_is_supported(type)) << "event type " << type;
    }
}

} // namespace
