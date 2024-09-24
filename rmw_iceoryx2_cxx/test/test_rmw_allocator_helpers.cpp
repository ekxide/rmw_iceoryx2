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

class AllocatorHelpersTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
    }

    void TearDown() override
    {
    }
};

TEST_F(AllocatorHelpersTest, AllocateCStringCopy)
{
    const char* original = "Hello, World!";
    const char* copy = iox2_rmw::allocate_copy(original);

    ASSERT_NE(copy, nullptr);
    ASSERT_STREQ(copy, original);
    ASSERT_NE(copy, original); // address should be different

    iox2_rmw::deallocate(const_cast<char*>(copy));
}

TEST_F(AllocatorHelpersTest, AllocateEmptyStringCopy)
{
    const char* empty = "";
    const char* copy = iox2_rmw::allocate_copy(empty);

    ASSERT_NE(copy, nullptr);
    ASSERT_STREQ(copy, empty);
    ASSERT_NE(copy, empty); // address should be different

    iox2_rmw::deallocate(const_cast<char*>(copy));
}

TEST_F(AllocatorHelpersTest, Allocate)
{
    int* ptr = iox2_rmw::allocate<int>();
    ASSERT_NE(ptr, nullptr);
    *ptr = 42;
    ASSERT_EQ(*ptr, 42);

    iox2_rmw::deallocate(ptr);
}

TEST_F(AllocatorHelpersTest, AllocateMany)
{
    const size_t array_size = 5;
    int* array = iox2_rmw::allocate<int>(array_size);
    ASSERT_NE(array, nullptr);

    for (size_t i = 0; i < array_size; ++i)
    {
        array[i] = i;
    }
    for (size_t i = 0; i < array_size; ++i)
    {
        ASSERT_EQ(array[i], i);
    }

    iox2_rmw::deallocate(array);
}

TEST_F(AllocatorHelpersTest, DeallocateNullptr)
{
    int* ptr = nullptr;
    ASSERT_NO_THROW(iox2_rmw::deallocate(ptr));
}

TEST_F(AllocatorHelpersTest, Destruct)
{
    class TestClass
    {
    public:
        TestClass()
            : value(42)
        {
        }
        ~TestClass()
        {
            value = 0;
        }
        int value;
    };

    TestClass* obj = iox2_rmw::allocate<TestClass>();
    new (obj) TestClass();
    ASSERT_EQ(obj->value, 42);

    iox2_rmw::destruct<TestClass>(obj);
    ASSERT_EQ(obj->value, 0);

    iox2_rmw::deallocate(obj);
}

TEST_F(AllocatorHelpersTest, DestructNullptr)
{
    ASSERT_NO_THROW(iox2_rmw::destruct<int>(nullptr));
}
