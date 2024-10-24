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
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/allocator_helpers.hpp"
#include "rmw_iceoryx2_cxx/create.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/names.hpp"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwGuardConditionTest : public TestBase
{
    using IceoryxListener = ::iox2::Listener<::iox2::ServiceType::Ipc>;

protected:
    void SetUp() override {
        initialize_test_context();
    }

    void TearDown() override {
        cleanup_test_context();
        print_rmw_errors();
    }

    rmw::iox2::NodeImpl& test_node() {
        if (!m_node) {
            create_in_place(m_node, names::test_node(test_id()).c_str()).expect("failed to create test node");
        }
        return m_node.value();
    }

    template <typename String>
    auto test_listener(String&& name) -> IceoryxListener {
        auto service_name =
            ::iox2::ServiceName::create(name.c_str()).expect("failed to create test listener service name");
        auto service = test_node()
                           .as_iox2()
                           .service_builder(service_name)
                           .event()
                           .open_or_create()
                           .expect("failed to create test listener service");
        auto listener = service.listener_builder().create().expect("failed to create test listener");

        return listener;
    };

private:
    iox::optional<::rmw::iox2::NodeImpl> m_node;
};

TEST_F(RmwGuardConditionTest, create_and_destroy) {
    auto guard_condition = rmw_create_guard_condition(&context);

    RMW_ASSERT_NE(guard_condition, nullptr);
    RMW_ASSERT_NE(guard_condition->context, nullptr);
    RMW_ASSERT_NE(guard_condition->implementation_identifier, nullptr);
    RMW_ASSERT_NE(guard_condition->data, nullptr);

    ASSERT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
}

TEST_F(RmwGuardConditionTest, trigger) {
    using ::rmw::iox2::GuardConditionImpl;
    using ::rmw::iox2::unsafe_cast;
    namespace names = ::rmw::iox2::names;

    auto guard_condition = rmw_create_guard_condition(&context);
    EXPECT_NE(guard_condition, nullptr);
    EXPECT_NE(guard_condition->data, nullptr);

    // TODO: An easier way to access the guard condition ID?
    auto impl = unsafe_cast<GuardConditionImpl*>(guard_condition->data).expect("failed to get guard condition impl");
    auto listener = test_listener(names::guard_condition(guard_condition->context->instance_id));

    EXPECT_RMW_OK(rmw_trigger_guard_condition(guard_condition));
    auto event =
        listener.timed_wait_one(iox::units::Duration::fromMicroseconds(500)).expect("failed to wait for trigger");
    ASSERT_TRUE(event.has_value());
    ASSERT_EQ(event.value().as_value(), impl->id());

    EXPECT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
}

} // namespace
