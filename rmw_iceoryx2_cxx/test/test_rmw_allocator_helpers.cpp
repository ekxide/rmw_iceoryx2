// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw_iceoryx2_cxx/allocator_helpers.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class DummyClass
{
public:
    DummyClass()
        : value(42) {
    }
    ~DummyClass() {
        value = 0;
    }
    int value;
};

class AllocatorHelpersTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
        print_rmw_errors();
    }
};

TEST_F(AllocatorHelpersTest, allocate_primitive) {
    auto result = rmw::iox2::allocate<int>();
    ASSERT_TRUE(result.has_value());

    auto ptr = result.value();
    *ptr = 42;
    ASSERT_EQ(*ptr, 42);

    rmw::iox2::deallocate(ptr);
}

TEST_F(AllocatorHelpersTest, allocate_many_primitives) {
    const size_t num = 5;
    auto result = rmw::iox2::allocate<int>(num);
    ASSERT_TRUE(result.has_value());

    auto ptr = result.value();
    for (size_t i = 0; i < num; ++i) {
        ptr[i] = i;
    }
    for (size_t i = 0; i < num; ++i) {
        ASSERT_EQ(ptr[i], i);
    }

    rmw::iox2::deallocate(ptr);
}

TEST_F(AllocatorHelpersTest, allocate_struct) {
    struct LargeObject
    {
        char data[1024];
    };
    auto result = rmw::iox2::allocate<LargeObject>();
    ASSERT_TRUE(result.has_value());

    auto ptr = result.value();
    memset(ptr->data, 'A', sizeof(ptr->data));
    ASSERT_EQ(ptr->data[0], 'A');
    ASSERT_EQ(ptr->data[1023], 'A');

    rmw::iox2::deallocate(ptr);
}

TEST_F(AllocatorHelpersTest, allocate_cstr_copy) {
    const char* original = "Hello, World!";
    const auto result = rmw::iox2::allocate_copy(original);
    ASSERT_TRUE(result.has_value());

    auto copy = result.value();
    ASSERT_STREQ(copy, original);
    ASSERT_NE(copy, original); // address should be different

    rmw::iox2::deallocate(copy);
}

TEST_F(AllocatorHelpersTest, allocate_empty_cstr_copy) {
    const char* empty = "";

    const auto result = rmw::iox2::allocate_copy(empty);
    ASSERT_TRUE(result.has_value());

    auto copy = result.value();
    ASSERT_STREQ(copy, empty);
    ASSERT_NE(copy, empty); // address should be different

    rmw::iox2::deallocate(copy);
}

TEST_F(AllocatorHelpersTest, deallocate_nullptr) {
    int* ptr = nullptr;
    ASSERT_NO_THROW(rmw::iox2::deallocate(ptr));
}

TEST_F(AllocatorHelpersTest, construct_and_destruct) {
    auto allocation = rmw::iox2::allocate<DummyClass>();
    ASSERT_TRUE(allocation.has_value());

    auto ptr = allocation.value();
    auto constructed = rmw::iox2::construct(ptr);
    ASSERT_TRUE(constructed.has_value());

    auto obj = constructed.value();
    ASSERT_EQ(obj->value, 42);

    rmw::iox2::destruct<DummyClass>(obj);
    ASSERT_EQ(obj->value, 0);

    rmw::iox2::deallocate(obj);
}

TEST_F(AllocatorHelpersTest, destruct_nullptr) {
    ASSERT_NO_THROW(rmw::iox2::destruct<int>(nullptr));
}

} // namespace
