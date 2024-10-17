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
#include "rmw_iceoryx2_cxx/iox2/publisher_impl.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwPublisherImplTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(RmwPublisherImplTest, construction) {
    rmw::iox2::ContextImpl context{test_id()};
    rmw::iox2::PublisherImpl publsher{context.node(), context.id(), "Topic", "Type", 8};
    ASSERT_TRUE(true);
}

// TEST_F(RmwPublisherImplTest, loan_and_return) {
//     rmw::iox2::ContextImpl context{test_id()};
//     rmw::iox2::PublisherImpl publisher{context.node(), "Topic", "Type"};
//
//     auto loaned_memory = publisher.loan();
//     EXPECT_FALSE(loaned_memory.has_error());
//     EXPECT_FALSE(publisher.return_loan(loaned_memory.value()).has_error());
// }

} // namespace
