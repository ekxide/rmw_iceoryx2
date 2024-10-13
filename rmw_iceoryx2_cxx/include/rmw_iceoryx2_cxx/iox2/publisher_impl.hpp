// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_PUBLISHER_IMPL_HPP_
#define RMW_IOX2_PUBLISHER_IMPL_HPP_

#include "iox/optional.hpp"
#include "iox2/node.hpp"
#include "iox2/publisher.hpp"
#include "iox2/sample_mut.hpp"
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"

#include <unordered_map>

namespace rmw::iox2
{

/// TODO: Implement and use iox::Slice to enable dynamically-sized types
/**
 * @brief Implementation of an RMW publisher using iceoryx2.
 *
 * @tparam PayloadSize The maximum size of a sample.
 */
class RMW_PUBLIC PublisherImpl
{
    using Payload = ::iox::Slice<uint8_t>;
    using Sample = ::iox2::SampleMut<::iox2::ServiceType::Ipc, Payload, void>;
    using IceoryxPublisher = ::iox2::Publisher<::iox2::ServiceType::Ipc, Payload, void>;

    /**
     * @brief Storage for loaned samples.
     *
     * Samples loaned from iceoryx2 must be retained until being published.
     * This struct stores samples, organized by the address of their payloads. This is because
     * this is the addressed that will be provided to the upper ROS layers to write the payload,
     * and then provided back to the RMW to execute the publish.
     */
    class SampleRegistry
    {
    public:
        auto store(Sample&& sample) -> void {
            m_samples.emplace(sample.payload_mut().begin(), std::move(sample));
        }
        auto retrieve(void* loaned_memory) -> iox::optional<Sample*> {
            using iox::nullopt;

            auto it = m_samples.find(loaned_memory);
            if (it != m_samples.end()) {
                return &(it->second);
            }
            return nullopt;
        }
        auto release(void* loaned_memory) -> iox::expected<void, LoanError> {
            using iox::err;
            using iox::ok;

            auto it = m_samples.find(loaned_memory);
            if (it == m_samples.end()) {
                return err(LoanError::INVALID_PAYLOAD);
            }
            m_samples.erase(it);
            return ok();
        }

    private:
        std::unordered_map<const void*, Sample> m_samples;
    };

public:
    explicit PublisherImpl(NodeImpl& node, const char* topic, const char* type);

    /**
     * @brief Get the topic name.
     */
    auto topic() -> const std::string&;

    /**
     * @brief Get the type name.
     */
    auto type() -> const std::string&;

    /**
     * @brief Loan memory from iceoryx2 to be populated with data to publish.
     *
     * @return Pointer to the beginning of the loaned memory which resides in shared memory.
     */
    auto loan() -> iox::expected<void*, LoanError>;

    /**
     * @brief Returns previously loaned, unpublished memory to iceoryx2.
     * @detail Does nothing if loaned_memory does not point to memory that was previously loaned, or if it was already
     * dropped.
     *
     * @param loaned_memory Pointer to memory loaned from iceoryx.
     *
     * @return Error if the provided loaned_memory was not previously loaned.
     */
    auto return_loan(void* loaned_memory) -> iox::expected<void, LoanError>;

    /**
     * @brief Publish memory previously loaned by this publisher.
     *
     * @param ros_message Pointer to the beginning of the loaned memory
     *
     * @return Error if unable to publish the provided loaned_memory.
     */
    auto publish(void* loaned_memory) -> iox::expected<void, PublishError>;

private:
    std::string m_topic;
    std::string m_type;

    iox::optional<IceoryxPublisher> m_publisher;
    SampleRegistry m_registry;
};

} // namespace rmw::iox2

#endif
