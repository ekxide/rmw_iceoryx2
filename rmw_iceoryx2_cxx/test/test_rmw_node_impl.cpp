// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw_iceoryx2_cxx/rmw_node_impl.hpp"
#include "testing/base.hpp"

namespace
{

class RmwNodeImplTest : public rmw::iox2::testing::TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(RmwNodeImplTest, construction) {
    rmw::iox2::NodeImpl node{"/Sensors/Camera"};
    ASSERT_TRUE(true);
}

} // namespace
