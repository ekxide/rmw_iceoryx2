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
#include "rmw_iceoryx2_cxx/rmw_allocator_helpers.hpp"
#include "rmw_iceoryx2_cxx/rmw_guard_condition_impl.hpp"
#include "rmw_iceoryx2_cxx/rmw_node_impl.hpp"
#include "rmw_iceoryx2_cxx/service_names.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

class RmwGuardConditionTest : public rmw::iox2::testing::TestBase
{
protected:
    void SetUp() override {
        init_options = rmw_get_zero_initialized_init_options();
        ASSERT_RMW_OK(rmw_init_options_init(&init_options, allocator));
        init_options.instance_id = unique_id;
        context = rmw_get_zero_initialized_context();
        ASSERT_RMW_OK(rmw_init(&init_options, &context));
    }

    void TearDown() override {
        EXPECT_RMW_OK(rmw_shutdown(&context));
        EXPECT_RMW_OK(rmw_context_fini(&context));
        EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
    }

    rmw::iox2::NodeImpl& test_node() {
        if (!m_node) {
            m_node.emplace(test_node_name().c_str());
        }
        return m_node.value();
    }

protected:
    rmw_init_options_t init_options;
    rmw_context_t context;

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

    auto guard_condition = rmw_create_guard_condition(&context);
    EXPECT_NE(guard_condition, nullptr);
    EXPECT_NE(guard_condition->data, nullptr);

    // TODO: An easier way to access the guard condition ID?
    auto impl =
        rmw::iox2::unsafe_cast<GuardConditionImpl*>(guard_condition->data).expect("failed to get guard condition impl");
    auto listener = test_node().create_listener(
        rmw::iox2::guard_condition_service_name(guard_condition->context->instance_id, impl->id()));

    EXPECT_RMW_OK(rmw_trigger_guard_condition(guard_condition));
    auto event =
        listener.timed_wait_one(iox::units::Duration::fromMicroseconds(500)).expect("failed to wait for trigger");
    ASSERT_TRUE(event.has_value());

    EXPECT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
}

} // namespace
