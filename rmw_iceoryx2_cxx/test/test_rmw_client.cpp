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
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{
using namespace rmw::iox2::testing;
using rmw_iceoryx2_cxx_test_msgs::srv::BasicTypes;

class RmwClientTest : public TestBase
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

TEST_F(RmwClientTest, create_and_destroy) {
    auto* client = create_client<BasicTypes>(create_test_topic());

    RMW_ASSERT_NE(client, nullptr);
    RMW_ASSERT_NE(client->implementation_identifier, nullptr);
    RMW_ASSERT_NE(client->data, nullptr);
    ASSERT_STREQ(client->service_name, create_test_topic().c_str());
}

TEST_F(RmwClientTest, reports_the_requested_qos) {
    auto profile = rmw_qos_profile_services_default;
    profile.depth = 7;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    auto* client = create_client<BasicTypes>(create_test_topic(), profile);
    RMW_ASSERT_NE(client, nullptr);

    rmw_qos_profile_t request_qos = {};
    ASSERT_RMW_OK(rmw_client_request_publisher_get_actual_qos(client, &request_qos));
    EXPECT_EQ(request_qos.depth, 7U);
    EXPECT_EQ(request_qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    rmw_qos_profile_t response_qos = {};
    ASSERT_RMW_OK(rmw_client_response_subscription_get_actual_qos(client, &response_qos));
    EXPECT_EQ(response_qos.depth, 7U);
    EXPECT_EQ(response_qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST_F(RmwClientTest, server_is_available_only_while_a_server_exists) {
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(client, nullptr);

    bool is_available = true;
    ASSERT_RMW_OK(rmw_service_server_is_available(test_node(), client, &is_available));
    EXPECT_FALSE(is_available);

    auto* service = create_service<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    ASSERT_RMW_OK(rmw_service_server_is_available(test_node(), client, &is_available));
    EXPECT_TRUE(is_available);

    destroy_service(service);
    ASSERT_RMW_OK(rmw_service_server_is_available(test_node(), client, &is_available));
    EXPECT_FALSE(is_available);
}

TEST_F(RmwClientTest, each_client_has_its_own_gid) {
    auto* first = create_client<BasicTypes>(create_test_topic());
    auto* second = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(first, nullptr);
    RMW_ASSERT_NE(second, nullptr);

    rmw_gid_t first_gid = {};
    rmw_gid_t second_gid = {};
    ASSERT_RMW_OK(rmw_get_gid_for_client(first, &first_gid));
    ASSERT_RMW_OK(rmw_get_gid_for_client(second, &second_gid));
    EXPECT_STREQ(first_gid.implementation_identifier, rmw_get_implementation_identifier());

    bool equal = true;
    ASSERT_RMW_OK(rmw_compare_gids_equal(&first_gid, &second_gid, &equal));
    EXPECT_FALSE(equal);
}

} // namespace
