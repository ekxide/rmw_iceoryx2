// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_TESTING_FAILING_ALLOCATOR_HPP_
#define RMW_IOX2_TESTING_FAILING_ALLOCATOR_HPP_

#include "rcutils/allocator.h"

#include <cstddef>
#include <cstdlib>

namespace rmw::iox2::testing
{

struct FailingAllocatorState
{
    size_t remaining;
    size_t live;
};

inline auto failing_allocate(size_t size, void* state) -> void* {
    auto* counts = static_cast<FailingAllocatorState*>(state);
    if (counts->remaining == 0) {
        return nullptr;
    }
    --counts->remaining;
    ++counts->live;
    return std::malloc(size);
}

inline auto failing_zero_allocate(size_t count, size_t size, void* state) -> void* {
    auto* counts = static_cast<FailingAllocatorState*>(state);
    if (counts->remaining == 0) {
        return nullptr;
    }
    --counts->remaining;
    ++counts->live;
    return std::calloc(count, size);
}

inline auto failing_reallocate(void* pointer, size_t size, void* state) -> void* {
    if (pointer == nullptr) {
        return failing_allocate(size, state);
    }
    return std::realloc(pointer, size);
}

inline auto failing_deallocate(void* pointer, void* state) -> void {
    if (pointer != nullptr) {
        --static_cast<FailingAllocatorState*>(state)->live;
        std::free(pointer);
    }
}

inline auto failing_allocator(FailingAllocatorState* state) -> rcutils_allocator_t {
    auto allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = failing_allocate;
    allocator.deallocate = failing_deallocate;
    allocator.reallocate = failing_reallocate;
    allocator.zero_allocate = failing_zero_allocate;
    allocator.state = state;
    return allocator;
}

} // namespace rmw::iox2::testing

#endif // RMW_IOX2_TESTING_FAILING_ALLOCATOR_HPP_
