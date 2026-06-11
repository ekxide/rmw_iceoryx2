// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox2/bb/duration.hpp"
#include "iox2/bb/optional.hpp"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwGuardConditionTest : public TestBase
{
    using Listener = ::iox2::Listener<::iox2::ServiceType::Local>;

protected:
    void SetUp() override {
        initialize_test_context();
    }

    void TearDown() override {
        cleanup_test_context();
        print_rmw_errors();
    }

    rmw::iox2::Iceoryx2& iox2() {
        if (!m_iox2.has_value()) {
            auto result = create_in_place(m_iox2, names::test_handle(test_id()));
            EXPECT_TRUE(result.has_value()) << "failed to create test node";
        }
        return m_iox2.value();
    }

    template <typename String>
    auto iox2_listener(String&& name) -> Listener {
        auto service_name = ::iox2::ServiceName::create(name.c_str());
        EXPECT_TRUE(service_name.has_value()) << "failed to create test listener service name";
        auto service = iox2().local().service_builder(service_name.value()).event().open_or_create();
        EXPECT_TRUE(service.has_value()) << "failed to create test listener service";
        auto listener = service.value().listener_builder().create();
        EXPECT_TRUE(listener.has_value()) << "failed to create test listener";

        return std::move(listener.value());
    }

private:
    ::iox2::bb::Optional<::rmw::iox2::Iceoryx2> m_iox2;
};

TEST_F(RmwGuardConditionTest, create_and_destroy) {
    auto guard_condition = rmw_create_guard_condition(test_context());

    RMW_ASSERT_NE(guard_condition, nullptr);
    RMW_ASSERT_NE(guard_condition->context, nullptr);
    RMW_ASSERT_NE(guard_condition->implementation_identifier, nullptr);
    RMW_ASSERT_NE(guard_condition->data, nullptr);

    ASSERT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
}

TEST_F(RmwGuardConditionTest, trigger) {
    using ::rmw::iox2::GuardCondition;
    using ::rmw::iox2::unsafe_cast;
    namespace names = ::rmw::iox2::names;

    auto guard_condition = rmw_create_guard_condition(test_context());
    EXPECT_NE(guard_condition, nullptr);
    EXPECT_NE(guard_condition->data, nullptr);

    // TODO: An easier way to access the guard condition ID?
    auto impl_result = unsafe_cast<GuardCondition*>(guard_condition->data);
    ASSERT_TRUE(impl_result.has_value()) << "failed to get guard condition impl";
    auto impl = impl_result.value();
    auto listener = iox2_listener(names::guard_condition(guard_condition->context->instance_id, impl->trigger_id()));

    EXPECT_RMW_OK(rmw_trigger_guard_condition(guard_condition));
    size_t received_id = 0;
    bool received = false;
    auto wait_result = listener.timed_wait(
        [&](auto activation) {
            received_id = activation.id().as_value();
            received = true;
        },
        ::iox2::bb::Duration::from_micros(500u));

    ASSERT_TRUE(wait_result.has_value()) << "failed to wait for trigger";
    ASSERT_TRUE(received);
    ASSERT_EQ(received_id, impl->trigger_id());

    EXPECT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
}

} // namespace
