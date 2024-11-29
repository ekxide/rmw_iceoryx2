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

#include "iox/expected.hpp"
#include "rmw/allocators.h"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/error_message.hpp"

#include <cstring>

namespace rmw::iox2
{

template <typename T>
RMW_PUBLIC inline auto allocate(size_t num = 1) -> iox::expected<T*, MemoryError> {
    using iox::err;
    using iox::ok;
    using rmw::iox2::MemoryError;

    if (num == 0) {
        RMW_IOX2_CHAIN_ERROR_MSG("attempted to allocate nothing");
        return err(MemoryError::ALLOCATION);
    }

    auto ptr = static_cast<T*>(rmw_allocate(sizeof(T) * num));
    if (!ptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate");
        return err(MemoryError::ALLOCATION);
    }
    return ok(ptr);
}

RMW_PUBLIC inline auto allocate_copy(const char* cstr) -> iox::expected<char*, MemoryError> {
    using iox::err;
    using iox::ok;
    using rmw::iox2::MemoryError;

    auto length = strlen(cstr);
    auto ptr = static_cast<char*>(rmw_allocate(sizeof(char) * length + 1));
    if (!ptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate");
        return err(MemoryError::ALLOCATION);
    }
    memcpy(ptr, cstr, length + 1);
    return ok(ptr);
}

template <typename T>
RMW_PUBLIC inline auto deallocate(T*& ptr) -> void {
    if (ptr) {
        rmw_free(static_cast<void*>(ptr));
        ptr = nullptr;
    }
}

RMW_PUBLIC inline auto deallocate(char*& ptr) -> void {
    if (ptr) {
        rmw_free(static_cast<void*>(ptr));
        ptr = nullptr;
    }
}

RMW_PUBLIC inline auto deallocate(const char*& ptr) -> void {
    if (ptr) {
        rmw_free(const_cast<void*>(static_cast<const void*>(ptr)));
        ptr = nullptr;
    }
}

template <typename T, typename... Args>
RMW_PUBLIC inline auto construct(T* ptr, Args&&... args) -> iox::expected<T*, MemoryError> {
    using iox::err;
    using iox::ok;
    using rmw::iox2::MemoryError;

    if (ptr == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("attempted to construct at nullptr");
        return err(MemoryError::CONSTRUCTION);
    }
    new (ptr) T(std::forward<Args>(args)...);
    return ok(ptr);
}

template <typename T>
RMW_PUBLIC inline auto destruct(void* ptr) -> void {
    if (ptr != nullptr) {
        T* typed_ptr = static_cast<T*>(ptr);
        typed_ptr->~T();
    }
}

template <typename T>
auto unsafe_cast(void* ptr) -> iox::expected<T, MemoryError> {
    using iox::err;
    using iox::ok;
    using rmw::iox2::MemoryError;

    if (!ptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("attempted to cast nullptr");
        return err(MemoryError::CAST);
    }
    return ok(reinterpret_cast<T>(ptr));
};

} // namespace rmw::iox2

#endif // RMW_IOX2_ALLOCATOR_HELPERS_HPP_
