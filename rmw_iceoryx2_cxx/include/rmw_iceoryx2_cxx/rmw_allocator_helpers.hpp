// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_ALLOCATOR_HELPERS_HPP_
#define RMW_IOX2_ALLOCATOR_HELPERS_HPP_

#include "rmw/allocators.h"
#include "rmw/visibility_control.h"

#include <cstring>
#include <iostream>

namespace rmw::iox2
{

RMW_PUBLIC inline const char* allocate_copy(const char* cstr) {
    auto length = strlen(cstr);
    auto ptr = static_cast<char*>(rmw_allocate(sizeof(char) * length + 1));
    if (!ptr) {
        return nullptr;
    }
    memcpy(ptr, cstr, length + 1);
    return ptr;
}

template <typename T>
RMW_PUBLIC inline T* allocate(size_t num = 1) {
    if (num == 0) {
        return nullptr;
    }
    auto ptr = static_cast<T*>(rmw_allocate(sizeof(T) * num));
    return ptr;
}

template <typename T>
RMW_PUBLIC inline void deallocate(T*& ptr) {
    if (ptr) {
        rmw_free(static_cast<void*>(ptr));
        ptr = nullptr;
    }
}

RMW_PUBLIC inline void deallocate(char*& ptr) {
    if (ptr) {
        rmw_free(static_cast<void*>(ptr));
        ptr = nullptr;
    }
}

RMW_PUBLIC inline void deallocate(const char*& ptr) {
    if (ptr) {
        rmw_free(const_cast<void*>(static_cast<const void*>(ptr)));
        ptr = nullptr;
    }
}

template <typename T>
RMW_PUBLIC inline void destruct(void* ptr) {
    if (ptr != nullptr) {
        T* typed_ptr = static_cast<T*>(ptr);
        typed_ptr->~T();
    }
}

template <typename T, typename... Args>
RMW_PUBLIC inline T* construct(T* ptr, Args&&... args) {
    if (ptr != nullptr) {
        new (ptr) T(std::forward<Args>(args)...);
        return ptr;
    }
    return nullptr;
}

} // namespace rmw::iox2

#endif // RMW_IOX2_ALLOCATOR_HELPERS_HPP_
