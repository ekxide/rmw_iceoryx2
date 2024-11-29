// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw_iceoryx2_cxx/allocator.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

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
    auto gid = allocate<rmw_gid_t>().expect("unable to allocate for rmw_gid_t");
    construct<rmw_gid_t>(gid).expect("unable to construct rmw_gid_t");
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
    auto gid_1 = allocate<rmw_gid_t>().expect("unable to allocate for rmw_gid_t");
    construct<rmw_gid_t>(gid_1).expect("unable to construct rmw_gid_t");
    memset(gid_1->data, 0, RMW_GID_STORAGE_SIZE);

    auto* publisher_2 = create_default_publisher<Defaults>(create_test_topic());
    auto gid_2 = allocate<rmw_gid_t>().expect("unable to allocate for rmw_gid_t");
    construct<rmw_gid_t>(gid_1).expect("unable to construct rmw_gid_t");
    memset(gid_1->data, 0, RMW_GID_STORAGE_SIZE);

    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher_1, gid_1));
    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher_2, gid_2));

    bool result_equal{false};
    ASSERT_RMW_OK(rmw_compare_gids_equal(gid_1, gid_1, &result_equal));
    ASSERT_TRUE(result_equal);

    bool result_unequal{true};
    ASSERT_RMW_OK(rmw_compare_gids_equal(gid_1, gid_2, &result_equal));
    ASSERT_TRUE(result_unequal);

    deallocate(gid_1);
    deallocate(gid_2);
}

} // namespace
