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
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
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

TEST_F(MessageIntrospectionTest, sizes) {
    using rmw::iox2::message_size;
    using rmw::iox2::serialized_message_size;
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    std::cout << "size(Defaults): " << message_size(test_type_support<Defaults>()) << std::endl;
    std::cout << "size(Strings): " << message_size(test_type_support<Strings>()) << std::endl;

    Defaults defaults_msg{};
    Strings strings_msg{};

    std::cout << "serialized_size(Defaults): " << serialized_message_size(&defaults_msg, test_type_support<Defaults>())
              << std::endl;
    std::cout << "serialized_size(Strings): " << serialized_message_size(&strings_msg, test_type_support<Strings>())
              << std::endl;
}

} // namespace
