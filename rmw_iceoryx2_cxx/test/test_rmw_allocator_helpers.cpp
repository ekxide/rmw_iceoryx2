// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw_iceoryx2_cxx/rmw_allocator_helpers.hpp"
#include "testing/base.hpp"

namespace
{

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

class AllocatorHelpersTest : public rmw::iox2::testing::TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(AllocatorHelpersTest, AllocateCStringCopy) {
    const char* original = "Hello, World!";
    const char* copy = rmw::iox2::allocate_copy(original);

    ASSERT_NE(copy, nullptr);
    ASSERT_STREQ(copy, original);
    ASSERT_NE(copy, original); // address should be different

    rmw::iox2::deallocate(copy);
}

TEST_F(AllocatorHelpersTest, AllocateEmptyCStringCopy) {
    const char* empty = "";
    const char* copy = rmw::iox2::allocate_copy(empty);

    ASSERT_NE(copy, nullptr);
    ASSERT_STREQ(copy, empty);
    ASSERT_NE(copy, empty); // address should be different

    rmw::iox2::deallocate(copy);
}

TEST_F(AllocatorHelpersTest, Allocate) {
    int* ptr = rmw::iox2::allocate<int>();
    ASSERT_NE(ptr, nullptr);
    *ptr = 42;
    ASSERT_EQ(*ptr, 42);

    rmw::iox2::deallocate(ptr);
}

TEST_F(AllocatorHelpersTest, AllocateMany) {
    const size_t array_size = 5;
    int* array = rmw::iox2::allocate<int>(array_size);
    ASSERT_NE(array, nullptr);

    for (size_t i = 0; i < array_size; ++i) {
        array[i] = i;
    }
    for (size_t i = 0; i < array_size; ++i) {
        ASSERT_EQ(array[i], i);
    }

    rmw::iox2::deallocate(array);
}

TEST_F(AllocatorHelpersTest, AllocateLargeObject) {
    struct LargeObject
    {
        char data[1024];
    };
    LargeObject* large_obj = rmw::iox2::allocate<LargeObject>();
    ASSERT_NE(large_obj, nullptr);
    memset(large_obj->data, 'A', sizeof(large_obj->data));
    ASSERT_EQ(large_obj->data[0], 'A');
    ASSERT_EQ(large_obj->data[1023], 'A');
    rmw::iox2::deallocate(large_obj);
}

TEST_F(AllocatorHelpersTest, DeallocateNullptr) {
    int* ptr = nullptr;
    ASSERT_NO_THROW(rmw::iox2::deallocate(ptr));
}

TEST_F(AllocatorHelpersTest, Destruct) {
    DummyClass* obj = rmw::iox2::allocate<DummyClass>();
    new (obj) DummyClass();
    ASSERT_EQ(obj->value, 42);

    rmw::iox2::destruct<DummyClass>(obj);
    ASSERT_EQ(obj->value, 0);

    rmw::iox2::deallocate(obj);
}

TEST_F(AllocatorHelpersTest, DestructNullptr) {
    ASSERT_NO_THROW(rmw::iox2::destruct<int>(nullptr));
}

} // namespace
