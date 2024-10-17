// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox/slice.hpp"
#include "iox2/node.hpp"
#include "iox2/sample_mut_uninit.hpp"
#include "iox2/service_name.hpp"
#include "rmw_iceoryx2_cxx/iox2/sample_registry.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwSampleRegistryTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(RmwSampleRegistryTest, store_and_retrieve_loaned_publisher_sample) {
    using Payload = ::iox::Slice<uint8_t>;
    using Sample = ::iox2::SampleMutUninit<::iox2::ServiceType::Ipc, Payload, void>;
    using ::iox2::NodeBuilder;
    using ::iox2::NodeName;
    using ::iox2::ServiceName;
    using ::iox2::ServiceType;
    using ::rmw::iox2::SampleRegistry;

    SampleRegistry<Sample> sut{};

    auto node =
        NodeBuilder()
            .name(
                NodeName::create("rmw_sample_registry_test::store_loaned_sample").expect("failed to create node name"))
            .create<ServiceType::Ipc>()
            .expect("");

    auto payload_size = 64;
    auto service_name = ServiceName::create("rmw_sample_registry_test::store_loaned_sample::topic")
                            .expect("failed to create service name");
    auto service = node.service_builder(service_name)
                       .publish_subscribe<Payload>()
                       .open_or_create()
                       .expect("failed to create service");
    auto publisher =
        service.publisher_builder().max_slice_len(payload_size).create().expect("failed to create publisher");
    auto sample = publisher.loan_slice_uninit(payload_size).expect("failed to loan");
    auto sample_ptr = sample.payload_slice().data();
    ASSERT_NE(sample_ptr, nullptr);

    auto stored_ptr = sut.store(std::move(sample));
    ASSERT_NE(stored_ptr, nullptr);

    ASSERT_EQ(sample_ptr, stored_ptr);

    auto released_sample = sut.release(stored_ptr);
    ASSERT_TRUE(released_sample.has_value());
    auto released_ptr = released_sample->payload_slice().data();
    ASSERT_EQ(released_ptr, sample_ptr);
}

} // namespace
