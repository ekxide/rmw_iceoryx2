// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
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
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/callback_thread.hpp"

#include <memory>

namespace
{

TEST(CallbackThreadTest, starting_a_started_thread_succeeds) {
    ::iox2::bb::Optional<rmw::iox2::Iceoryx2> iox2;
    ASSERT_TRUE(
        rmw::iox2::create_in_place(iox2, "CallbackThreadTest", rmw::iox2::Iceoryx2::Config::global_config().to_owned())
            .has_value());
    auto callback_thread = std::make_shared<rmw::iox2::CallbackThread>();
    ASSERT_TRUE(callback_thread->start(iox2.value()).has_value());
    ASSERT_TRUE(callback_thread->start(iox2.value()).has_value());
}

} // namespace
