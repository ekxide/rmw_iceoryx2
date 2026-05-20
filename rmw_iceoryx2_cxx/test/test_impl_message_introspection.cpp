// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/arrays.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/bounded_sequences.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/empty.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/multi_nested.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/nested.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/unbounded_sequences.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/w_strings.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class MessageIntrospectionTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
        print_rmw_errors();
    }
};

TEST_F(MessageIntrospectionTest, self_containted_messages_properly_classified) {
    using rmw::iox2::is_self_contained;
    using rmw_iceoryx2_cxx_test_msgs::msg::Arrays;
    using rmw_iceoryx2_cxx_test_msgs::msg::BoundedSequences;
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    using rmw_iceoryx2_cxx_test_msgs::msg::Empty;
    using rmw_iceoryx2_cxx_test_msgs::msg::MultiNested;
    using rmw_iceoryx2_cxx_test_msgs::msg::Nested;
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;
    using rmw_iceoryx2_cxx_test_msgs::msg::UnboundedSequences;
    using rmw_iceoryx2_cxx_test_msgs::msg::WStrings;

    // Self-contained
    ASSERT_TRUE(is_self_contained(test_type_support<Defaults>()));
    ASSERT_TRUE(is_self_contained(test_type_support<Empty>()));
    ASSERT_TRUE(is_self_contained(test_type_support<Nested>())); // BasicTypes inside.

    // Not self-contained
    ASSERT_FALSE(is_self_contained(test_type_support<Arrays>()));
    ASSERT_FALSE(is_self_contained(test_type_support<Strings>()));
    ASSERT_FALSE(is_self_contained(test_type_support<WStrings>()));
    ASSERT_FALSE(is_self_contained(test_type_support<BoundedSequences>()));
    ASSERT_FALSE(is_self_contained(test_type_support<UnboundedSequences>()));
    ASSERT_FALSE(is_self_contained(test_type_support<MultiNested>()));
}

} // namespace
