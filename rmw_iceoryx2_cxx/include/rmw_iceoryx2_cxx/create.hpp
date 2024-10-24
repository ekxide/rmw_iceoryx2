// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_CREATE_HPP_
#define RMW_IOX2_CREATE_HPP_

#include "iox/expected.hpp"
#include "iox/optional.hpp"
#include "iox2/service_builder_publish_subscribe_error.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"

namespace rmw::iox2
{

/**
 * @brief Creates an object of type T at the given memory location.
 *
 * This function constructs an object of type T at the specified memory address,
 * passing any additional arguments to the constructor. It uses placement new
 * and handles potential construction errors.
 *
 * @tparam T The type of object to create.
 * @tparam Args Variadic template for constructor arguments.
 * @param ptr Pointer to the memory location where the object should be constructed.
 * @param args Arguments to be forwarded to the constructor of T.
 * @return An expected object containing a pointer to the constructed object on success,
 *         or a ConstructionError on failure. On failure, the RMW error state is set with the cause.
 */
template <typename T, typename... Args>
RMW_PUBLIC inline auto create_in_place(T* ptr, Args&&... args) -> iox::expected<T*, ConstructionError> {
    using iox::err;
    using iox::ok;

    if (ptr == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("attempted to construct at nullptr");
        return err(ConstructionError::INVALID_DESTINATION);
    }

    iox::optional<typename Error<T>::Type> error{};
    new (ptr) T(error, std::forward<Args>(args)...);

    if (error.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("error in constructor");
        return err(ConstructionError::FAILED_TO_CONSTRUCT);
    }

    return ok(ptr);
}

/**
 * @brief Creates an object of type T within an optional storage.
 *
 * This function constructs an object of type T within the provided optional storage,
 * passing any additional arguments to the constructor. It handles potential construction errors.
 *
 * @tparam T The type of object to create.
 * @tparam Args Variadic template for constructor arguments.
 * @param storage An optional storage where the object should be constructed.
 * @param args Arguments to be forwarded to the constructor of T.
 * @return An expected object containing void on success, or a ConstructionError on failure.
 *         On failure, the RMW error state is set with the cause.
 */
template <typename T, typename... Args>
RMW_PUBLIC inline auto create_in_place(iox::optional<T>& storage,
                                       Args&&... args) -> iox::expected<void, ConstructionError> {
    using iox::err;
    using iox::ok;

    iox::optional<typename Error<T>::Type> error{};
    storage.emplace(error, std::forward<Args>(args)...);

    if (error.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("error in constructor");
        return err(ConstructionError::FAILED_TO_CONSTRUCT);
    }

    return ok();
}

} // namespace rmw::iox2

#endif
