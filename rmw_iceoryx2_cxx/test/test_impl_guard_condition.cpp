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
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class GuardConditionTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(GuardConditionTest, construction) {
    using ::rmw::iox2::Context;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::GuardCondition;
    using ::rmw::iox2::GuardConditionError;

    rmw_init_options_impl_s options;
    ::iox2::bb::Optional<Context> context_storage;
    ASSERT_TRUE(create_in_place(context_storage, test_id(), options).has_value())
        << "failed to create context for guard condition creation";
    auto& context = context_storage.value();

    ::iox2::bb::Optional<GuardCondition> guard_condition_storage;
    ASSERT_TRUE(create_in_place(guard_condition_storage, context).has_value());
}

} // namespace
