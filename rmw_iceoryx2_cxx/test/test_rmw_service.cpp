// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx_test_msgs/srv/basic_types.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/srv/empty.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{
using namespace rmw::iox2::testing;
using rmw_iceoryx2_cxx_test_msgs::srv::BasicTypes;
using rmw_iceoryx2_cxx_test_msgs::srv::Empty;

class RmwServiceTest : public TestBase
{
protected:
    void SetUp() override {
        initialize();
    }

    void TearDown() override {
        cleanup();
        print_rmw_errors();
    }
};

TEST_F(RmwServiceTest, create_and_destroy) {
    auto* service = create_service<BasicTypes>(create_test_topic());

    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(service->implementation_identifier, nullptr);
    RMW_ASSERT_NE(service->data, nullptr);
    ASSERT_STREQ(service->service_name, create_test_topic().c_str());
}

TEST_F(RmwServiceTest, two_servers_can_serve_the_same_service) {
    RMW_ASSERT_NE(create_service<BasicTypes>(create_test_topic()), nullptr);
    RMW_ASSERT_NE(create_service<BasicTypes>(create_test_topic()), nullptr);
}

TEST_F(RmwServiceTest, rejects_a_different_service_type_under_the_same_name) {
    RMW_ASSERT_NE(create_service<BasicTypes>(create_test_topic()), nullptr);
    EXPECT_EQ(create_service<Empty>(create_test_topic()), nullptr);
    rcutils_reset_error();
}

TEST_F(RmwServiceTest, reports_the_requested_qos) {
    auto profile = rmw_qos_profile_services_default;
    profile.depth = 7;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    auto* service = create_service<BasicTypes>(create_test_topic(), profile);
    RMW_ASSERT_NE(service, nullptr);

    rmw_qos_profile_t request_qos = {};
    ASSERT_RMW_OK(rmw_service_request_subscription_get_actual_qos(service, &request_qos));
    EXPECT_EQ(request_qos.depth, 7U);
    EXPECT_EQ(request_qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    rmw_qos_profile_t response_qos = {};
    ASSERT_RMW_OK(rmw_service_response_publisher_get_actual_qos(service, &response_qos));
    EXPECT_EQ(response_qos.depth, 7U);
    EXPECT_EQ(response_qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

} // namespace
