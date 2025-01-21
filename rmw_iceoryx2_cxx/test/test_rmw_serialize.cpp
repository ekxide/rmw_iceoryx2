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

TEST_F(RmwSerializeTest, serialize_deserialize_pod_type) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    Defaults input{};
    input.int64_value = 777;

    rmw_serialized_message_t serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Defaults), &test_allocator()));
    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Defaults>(), &serialized_msg));

    Defaults output{};
    ASSERT_RMW_OK(rmw_deserialize(&serialized_msg, test_type_support<Defaults>(), &output));
    ASSERT_EQ(input, output);
}

TEST_F(RmwSerializeTest, serialize_deserialize_non_pod_type) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    Strings input{};
    input.string_value = "GloryToHypnoToad";

    rmw_serialized_message_t serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Strings), &test_allocator()));
    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Strings>(), &serialized_msg));

    Strings output{};
    ASSERT_RMW_OK(rmw_deserialize(&serialized_msg, test_type_support<Strings>(), &output));
    ASSERT_EQ(input, output);
}

} // namespace
