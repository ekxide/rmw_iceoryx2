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
#include "rmw_iceoryx2_cxx/create.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwGuardConditionImplTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(RmwGuardConditionImplTest, construction) {
    using ::rmw::iox2::ContextImpl;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::GuardConditionError;
    using ::rmw::iox2::GuardConditionImpl;

    iox::optional<ContextImpl> context_storage;
    create_in_place(context_storage, test_id()).expect("failed to create context for guard condition creation");
    auto& context = context_storage.value();

    iox::optional<GuardConditionImpl> guard_condition;
    ASSERT_FALSE(
        create_in_place(guard_condition, context.node(), context.id(), context.next_guard_condition_id()).has_error());
}

} // namespace
