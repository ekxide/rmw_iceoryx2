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
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/srv/basic_types.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <algorithm>

namespace
{
using namespace rmw::iox2::testing;
using rmw_iceoryx2_cxx_test_msgs::srv::BasicTypes;

class RmwRequestResponseTest : public TestBase
{
protected:
    void SetUp() override {
        initialize();
    }

    void TearDown() override {
        cleanup();
        print_rmw_errors();
    }

    static BasicTypes::Request make_request(int32_t value) {
        BasicTypes::Request request;
        request.int32_value = value;
        request.string_value = "request " + std::to_string(value);
        return request;
    }
};

TEST_F(RmwRequestResponseTest, a_request_reaches_the_server) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(42);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

    BasicTypes::Request received_request;
    rmw_service_info_t request_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &received_request, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_request, request);
    EXPECT_EQ(request_header.request_id.sequence_number, sequence_id);

    rmw_gid_t client_gid{};
    ASSERT_RMW_OK(rmw_get_gid_for_client(client, &client_gid));
    EXPECT_TRUE(std::equal(client_gid.data,
                           client_gid.data + RMW_GID_STORAGE_SIZE,
                           reinterpret_cast<const uint8_t*>(request_header.request_id.writer_guid)));

    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &received_request, &taken));
    EXPECT_FALSE(taken);
}

TEST_F(RmwRequestResponseTest, requests_are_numbered_in_order) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    int64_t first_id = 0;
    int64_t second_id = 0;
    auto first_request = make_request(1);
    auto second_request = make_request(2);
    ASSERT_RMW_OK(rmw_send_request(client, &first_request, &first_id));
    ASSERT_RMW_OK(rmw_send_request(client, &second_request, &second_id));
    EXPECT_LT(first_id, second_id);

    for (auto expected_id : {first_id, second_id}) {
        BasicTypes::Request request;
        rmw_service_info_t header{};
        bool taken = false;
        ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
        ASSERT_TRUE(taken);
        EXPECT_EQ(header.request_id.sequence_number, expected_id);
    }
}

TEST_F(RmwRequestResponseTest, requests_of_servers_that_are_gone_do_not_occupy_the_client) {
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(client, nullptr);

    for (size_t i = 0; i <= ::rmw::iox2::DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT; ++i) {
        auto* service = create_service<BasicTypes>(create_test_topic());
        RMW_ASSERT_NE(service, nullptr);

        auto request = make_request(static_cast<int32_t>(i));
        int64_t sequence_id = 0;
        ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));
        destroy_service(service);
    }
}

TEST_F(RmwRequestResponseTest, a_request_without_a_server_is_lost) {
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(1);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

    auto* service = create_service<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    rmw_service_info_t header{};
    bool taken = true;
    ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
    EXPECT_FALSE(taken);
}

} // namespace
