// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_CREATION_LOCK_HPP_
#define RMW_IOX2_CREATION_LOCK_HPP_

#include "iox/expected.hpp"

namespace rmw::iox2
{

/**
 * @brief A utility class for controlling object creation.
 * @details CreationLock is used to restrict and manage the creation of certain objects.
 * It provides a mechanism for controlled instantiation through friend functions.
 * This class cannot be directly instantiated by users, ensuring that object
 * creation follows the intended patterns defined by the system.
 *
 * Structs whose construction should be restricted to the creation pattern should have a definition in the
 * form:
 *
 * @code{.cpp}
 * enum class ErrorType : uint8_t {
 *      SOME_ERROR,
 * };
 *
 * class MyType {
 *      using ErrorType = MyErrorType;
 *      MyType(CreationLock, iox::optional<ErrorType>&, ...);
 * };
 * @endcode
 */
class CreationLock
{
private:
    template <typename T, typename... Args>
    friend auto create(Args&&... args) -> iox::expected<T, typename T::ErrorType>;

    template <typename T, typename... Args>
    friend auto create_in_place(T* ptr, Args&&... args) -> iox::expected<void, typename T::ErrorType>;

    template <typename T, typename... Args>
    friend auto create_in_place(iox::optional<T>& storage,
                                Args&&... args) -> iox::expected<void, typename T::ErrorType>;

    CreationLock() = default;
    static auto unlock() -> CreationLock {
        return CreationLock{};
    }
};

} // namespace rmw::iox2

#endif
