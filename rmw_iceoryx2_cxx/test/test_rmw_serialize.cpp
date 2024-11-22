// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwSerializeTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
        print_rmw_errors();
    }
};

TEST_F(RmwSerializeTest, serialize_pod_type_succeeds) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    Defaults msg{};
    rmw_serialized_message_t serialized_msg{};
    auto allocator = rcutils_get_default_allocator();

    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Defaults), &allocator));
    ASSERT_RMW_OK(rmw_serialize(&msg, test_type_support<Defaults>(), &serialized_msg));
    ASSERT_EQ(msg, *reinterpret_cast<Defaults*>(serialized_msg.buffer));
}

TEST_F(RmwSerializeTest, deserialize_pod_type_succeeds) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    Defaults msg{};
    msg.int64_value = 777;
    rmw_serialized_message_t serialized_msg{};
    auto allocator = rcutils_get_default_allocator();
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Defaults), &allocator));
    new (serialized_msg.buffer) Defaults{};
    serialized_msg.buffer_length = sizeof(Defaults);

    ASSERT_RMW_OK(rmw_deserialize(&serialized_msg, test_type_support<Defaults>(), &msg));

    ASSERT_EQ(msg, *reinterpret_cast<Defaults*>(serialized_msg.buffer));
}

TEST_F(RmwSerializeTest, serialize_non_pod_type_fails) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    Strings msg{};
    rmw_serialized_message_t serialized_msg{};
    auto allocator = rcutils_get_default_allocator();

    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Strings), &allocator));
    ASSERT_RMW_ERR(RMW_RET_UNSUPPORTED, rmw_serialize(&msg, test_type_support<Strings>(), &serialized_msg));
}

TEST_F(RmwSerializeTest, deserialize_non_pod_type_fails) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    Strings msg{};
    rmw_serialized_message_t serialized_msg{};
    auto allocator = rcutils_get_default_allocator();
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Strings), &allocator));
    new (serialized_msg.buffer) Strings{};
    serialized_msg.buffer_length = sizeof(Strings);

    ASSERT_RMW_ERR(RMW_RET_UNSUPPORTED, rmw_deserialize(&serialized_msg, test_type_support<Strings>(), &msg));
}

} // namespace
