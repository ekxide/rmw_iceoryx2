// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox2/bb/optional.hpp"
#include "rmw/qos_profiles.h"
#include "rmw_iceoryx2_cxx/impl/common/attributes.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/publisher.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class PublisherTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(PublisherTest, construction) {
    using ::rmw::iox2::Context;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::Node;
    using ::rmw::iox2::ProfileKind;
    using ::rmw::iox2::Publisher;
    using ::rmw::iox2::Qos;
    using ::rmw::iox2::TryConvert;
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    rmw_init_options_impl_s options;
    ::iox2::bb::Optional<Context> context_storage;
    ASSERT_TRUE(create_in_place(context_storage, test_id(), options).has_value())
        << "failed to create context for publisher creation";
    auto& context = context_storage.value();

    ::iox2::bb::Optional<Node> node_storage;
    ASSERT_TRUE(create_in_place(node_storage, context, "Node", "RmwPublisherTest").has_value())
        << "failed to create node for publisher creation";
    auto& node = node_storage.value();

    auto qos = TryConvert<Qos>::from(rmw_qos_profile_default, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(qos.has_value()) << "failed to resolve default QoS";

    ::iox2::bb::Optional<Publisher> publisher_storage;
    ASSERT_TRUE(
        create_in_place(publisher_storage, node, "Topic", test_type_support<Defaults>(), qos.value()).has_value());
}

} // namespace
