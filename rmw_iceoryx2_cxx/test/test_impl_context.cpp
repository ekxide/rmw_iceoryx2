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
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class ContextTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(ContextTest, construction) {
    using ::rmw::iox2::Context;
    using ::rmw::iox2::create_in_place;

    rmw_init_options_impl_s options;
    ::iox2::bb::Optional<Context> context_storage;
    ASSERT_TRUE(create_in_place(context_storage, test_id(), options).has_value());
}

} // namespace
