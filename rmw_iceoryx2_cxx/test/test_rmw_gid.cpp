// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/get_service_endpoint_info.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/srv/basic_types.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <algorithm>
#include <array>
#include <vector>

namespace
{

using namespace rmw::iox2::testing;

class RmwGidTest : public TestBase
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

TEST_F(RmwGidTest, can_retrieve_publisher_gid) {
    using rmw::iox2::allocate;
    using rmw::iox2::construct;
    using rmw::iox2::deallocate;
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    auto gid_alloc = allocate<rmw_gid_t>();
    ASSERT_TRUE(gid_alloc.has_value()) << "unable to allocate for rmw_gid_t";
    auto gid = gid_alloc.value();
    ASSERT_TRUE(construct<rmw_gid_t>(gid).has_value()) << "unable to construct rmw_gid_t";
    memset(gid->data, 0, RMW_GID_STORAGE_SIZE);

    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher, gid));

    ASSERT_TRUE(memcmp(gid->data, "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", RMW_GID_STORAGE_SIZE) != 0);

    deallocate(gid);
}

TEST_F(RmwGidTest, can_compare_publisher_gids) {
    using rmw::iox2::allocate;
    using rmw::iox2::construct;
    using rmw::iox2::deallocate;
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher_1 = create_default_publisher<Defaults>(create_test_topic());
    auto gid_1_alloc = allocate<rmw_gid_t>();
    ASSERT_TRUE(gid_1_alloc.has_value()) << "unable to allocate for rmw_gid_t";
    auto gid_1 = gid_1_alloc.value();
    ASSERT_TRUE(construct<rmw_gid_t>(gid_1).has_value()) << "unable to construct rmw_gid_t";
    memset(gid_1->data, 0, RMW_GID_STORAGE_SIZE);

    auto* publisher_2 = create_default_publisher<Defaults>(create_test_topic());
    auto gid_2_alloc = allocate<rmw_gid_t>();
    ASSERT_TRUE(gid_2_alloc.has_value()) << "unable to allocate for rmw_gid_t";
    auto gid_2 = gid_2_alloc.value();
    ASSERT_TRUE(construct<rmw_gid_t>(gid_2).has_value()) << "unable to construct rmw_gid_t";
    memset(gid_2->data, 0, RMW_GID_STORAGE_SIZE);

    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher_1, gid_1));
    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher_2, gid_2));

    bool result_equal{false};
    ASSERT_RMW_OK(rmw_compare_gids_equal(gid_1, gid_1, &result_equal));
    ASSERT_TRUE(result_equal);

    bool result_unequal{true};
    ASSERT_RMW_OK(rmw_compare_gids_equal(gid_1, gid_2, &result_unequal));
    ASSERT_FALSE(result_unequal);

    deallocate(gid_1);
    deallocate(gid_2);
}

TEST_F(RmwGidTest, gids_of_different_endpoint_kinds_differ) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    using rmw_iceoryx2_cxx_test_msgs::srv::BasicTypes;

    const auto topic = create_test_topic();
    auto* publisher = create_default_publisher<Defaults>(topic);
    ASSERT_NE(create_default_subscriber<Defaults>(topic), nullptr);
    ASSERT_NE(create_service<BasicTypes>(topic), nullptr);
    auto* client = create_client<BasicTypes>(topic);
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(client, nullptr);

    std::vector<std::array<uint8_t, RMW_GID_STORAGE_SIZE>> gids;
    auto keep = [&gids](const uint8_t* data) {
        std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid{};
        std::copy(data, data + RMW_GID_STORAGE_SIZE, gid.begin());
        gids.push_back(gid);
    };
    rmw_gid_t gid{};
    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher, &gid));
    keep(gid.data);
    ASSERT_RMW_OK(rmw_get_gid_for_client(client, &gid));
    keep(gid.data);
    auto allocator = rcutils_get_default_allocator();
    auto subscriptions = rmw_get_zero_initialized_topic_endpoint_info_array();
    ASSERT_RMW_OK(rmw_get_subscriptions_info_by_topic(test_node(), &allocator, topic.c_str(), false, &subscriptions));
    ASSERT_EQ(subscriptions.size, 1U);
    keep(subscriptions.info_array[0].endpoint_gid);
    auto servers = rmw_get_zero_initialized_service_endpoint_info_array();
    ASSERT_RMW_OK(rmw_get_servers_info_by_service(test_node(), &allocator, topic.c_str(), false, &servers));
    ASSERT_EQ(servers.size, 1U);
    keep(servers.info_array[0].endpoint_gids[0]);

    std::sort(gids.begin(), gids.end());
    EXPECT_EQ(std::adjacent_find(gids.begin(), gids.end()), gids.end());
    EXPECT_RMW_OK(rmw_topic_endpoint_info_array_fini(&subscriptions, &allocator));
    EXPECT_RMW_OK(rmw_service_endpoint_info_array_fini(&servers, &allocator));
}

} // namespace
