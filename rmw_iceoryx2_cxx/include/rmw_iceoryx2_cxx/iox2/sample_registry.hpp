// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_SAMPLE_REGISTRY_HPP_
#define RMW_IOX2_SAMPLE_REGISTRY_HPP_

#include "iox/expected.hpp"
#include "iox/optional.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"

#include <unordered_map>

namespace rmw::iox2
{

template <typename T>
class SampleRegistry;

template <typename T>
struct Error<SampleRegistry<T>>
{
    using Type = SampleRegistryError;
};


/// @brief Stores samples loaned from iceoryx2 before they are ready for publishing.
/// @details Samples loaned from iceoryx2 must be retained until being published or manually released (e.g. by
///          subscriptions).
///
/// This struct stores samples, organized by the address of their payloads.
/// This is because this is the addresses that will be provided to the upper ROS layers to write the payload,
/// and then provided back to the RMW to execute the publish or release the sample.
///
template <typename SampleType>
class SampleRegistry
{
public:
    using ErrorType = typename Error<SampleRegistry<SampleType>>::Type;

public:
    /// @brief Store a sample in the registry and return a pointer to its payload
    /// @param[in] sample The sample to store
    /// @return Pointer to the payload data that can also be used to retrieve/release the sample later
    auto store(SampleType&& sample) -> uint8_t* {
        // const_cast required to work with Sample and SamplMut
        // Should be adapted to handle both cases without casting (when functional)
        auto payload_ptr = const_cast<uint8_t*>(sample.payload().data());
        m_samples.emplace(payload_ptr, std::move(sample));
        return payload_ptr;
    }

    /// @brief Retrieve a stored sample by its payload pointer without removing it
    /// @param[in] loaned_memory Pointer to the payload data
    /// @return Pointer to the stored sample if found, nullopt otherwise
    auto retrieve(uint8_t* loaned_memory) -> iox::optional<SampleType*> {
        using iox::nullopt;

        auto it = m_samples.find(loaned_memory);
        if (it != m_samples.end()) {
            return &(it->second);
        }
        return nullopt;
    }

    /// @brief Remove and return a stored sample
    /// @param[in] loaned_memory Pointer to the payload data
    /// @return Expected containing the removed sample if found, error otherwise
    auto release(uint8_t* loaned_memory) -> iox::expected<SampleType, ErrorType> {
        using iox::err;
        using iox::ok;
        using iox::optional;

        auto it = m_samples.find(loaned_memory);
        if (it == m_samples.end()) {
            return err(ErrorType::INVALID_PAYLOAD);
        }
        auto sample = std::move(it->second);
        m_samples.erase(it);
        return ok(std::move(sample));
    }

private:
    std::unordered_map<uint8_t*, SampleType> m_samples;
};

} // namespace rmw::iox2

#endif
