// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwSubscriberImplTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(RmwSubscriberImplTest, construction) {
    rmw::iox2::ContextImpl context{test_id()};
    rmw::iox2::SubscriberImpl publsher{context.node(), "Topic", "Type"};
    ASSERT_TRUE(true);
}

} // namespace
