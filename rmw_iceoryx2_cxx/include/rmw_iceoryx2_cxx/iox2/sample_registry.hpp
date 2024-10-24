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

/**
 * @brief Storage for loaned samples.
 *
 * Samples loaned from iceoryx2 must be retained until being published or manually released (e.g. by subscriptions).
 * This struct stores samples, organized by the address of their payloads.
 * This is because this is the addresses that will be provided to the upper ROS layers to write the payload,
 * and then provided back to the RMW to execute the publish or release the sample.
 */
template <typename SampleType>
class SampleRegistry
{
public:
    using ErrorType = typename Error<SampleRegistry<SampleType>>::Type;

public:
    auto store(SampleType&& sample) -> uint8_t* {
        // const_cast required to work with Sample and SamplMut
        auto it = m_samples.emplace(const_cast<uint8_t*>(sample.payload_slice().data()), std::move(sample));
        return it.first->first;
    }
    auto retrieve(uint8_t* loaned_memory) -> iox::optional<SampleType*> {
        using iox::nullopt;

        auto it = m_samples.find(loaned_memory);
        if (it != m_samples.end()) {
            return &(it->second);
        }
        return nullopt;
    }
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
