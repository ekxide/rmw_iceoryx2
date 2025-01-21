// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox/optional.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/subscriber.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class SubscriberTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(SubscriberTest, construction) {
    using ::rmw::iox2::Context;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::Node;
    using ::rmw::iox2::Subscriber;
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    iox::optional<Context> context_storage;
    create_in_place(context_storage, test_id()).expect("failed to create context for subscriber creation");
    auto& context = context_storage.value();

    iox::optional<Node> node_storage;
    create_in_place(node_storage, context, "Node", "RmwPublisherTest")
        .expect("failed to create node for publisher creation");
    auto& node = node_storage.value();

    iox::optional<Subscriber> subscriber_storage;
    ASSERT_FALSE(create_in_place(subscriber_storage, node, "Topic", test_type_support<Defaults>()).has_error());
}

} // namespace
