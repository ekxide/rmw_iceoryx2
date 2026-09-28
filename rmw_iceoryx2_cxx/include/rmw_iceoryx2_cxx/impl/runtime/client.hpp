// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_CLIENT_HPP_
#define RMW_IOX2_RUNTIME_CLIENT_HPP_

#include "iox2/bb/optional.hpp"
#include "iox2/bb/slice.hpp"
#include "iox2/marker.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/types.h"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/sample_registry.hpp"
#include "rmw_iceoryx2_interoperability/rmw_iceoryx2_interoperability.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

#include <map>
#include <mutex>

namespace rmw::iox2
{

class Client;

template <>
struct Error<Client>
{
    using Type = ClientError;
};

/// @brief A response taken by a client, valid until the loan is returned
struct ClientResponse
{
    uint8_t* bytes;
    size_t number_of_bytes;
    ::rmw_iceoryx2_interoperability::MessageInfoHeader message_info;
};

/// @brief Implementation of the RMW client for iceoryx2
///
/// @details Requests and responses are exchanged as CDR-serialized payloads of an iceoryx2
///          request-response service. The user header carries the ROS sequence number of the
///          request and the source timestamp. All operations are thread-safe, as the RMW API
///          requires.
class RMW_PUBLIC Client
{
public:
    using Payload = ::iox2::bb::Slice<::iox2::CustomPayloadMarker>;
    using UserHeader = ::rmw_iceoryx2_interoperability::MessageInfoHeader;
    using ErrorType = Error<Client>::Type;

private:
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniqueClientId;
    using IceoryxService = Iceoryx2::InterProcess::RequestResponseService<Payload, UserHeader>;
    using IceoryxClient = Iceoryx2::InterProcess::Client<Payload, UserHeader>;
    using IceoryxRequest = Iceoryx2::InterProcess::RequestMutUninit<Payload, UserHeader>;
    using IceoryxPendingResponse = Iceoryx2::InterProcess::PendingResponse<Payload, UserHeader>;
    using IceoryxResponse = Iceoryx2::InterProcess::Response<Payload, UserHeader>;

public:
    /// @brief Constructor for Client
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] node The node that owns this client
    /// @param[in] service The name of the ROS service to call
    /// @param[in] type_support The service typesupport
    /// @param[in] qos The QoS requested for this client
    Client(CreationLock,
           ::iox2::bb::Optional<ErrorType>& error,
           Node& node,
           const char* service,
           const rosidl_service_type_support_t* type_support,
           const rmw_qos_profile_t& qos);

    /// @brief Get the unique identifier of this client
    /// @return The unique id or empty optional if failing to retrieve it from iceoryx2
    auto unique_id() -> const ::iox2::bb::Optional<RawIdType>&;

    /// @brief Get the name of the ROS service
    /// @return The service name as string
    auto service() const -> const std::string&;

    /// @brief Get the typesupport used by the client
    /// @return Pointer to the typesupport stored in the loaded typesupport library
    auto typesupport() const -> const rosidl_service_type_support_t*;

    /// @brief Get the service name used internally, required for matching via iceoryx2
    /// @return The service name as string
    auto service_name() const -> const std::string&;

    /// @brief Get the QoS requested for this client
    /// @return Reference to the QoS
    auto qos() const -> const rmw_qos_profile_t&;

    /// @brief Check whether a server is available for the service
    /// @return True if at least one server exists
    auto is_server_available() const -> bool;

    /// @brief Loan memory for a request
    /// @param[in] number_of_bytes Required buffer size in bytes
    /// @return Expected containing pointer to loaned memory or error
    auto loan_request(uint64_t number_of_bytes) -> ::iox2::bb::Expected<void*, ErrorType>;

    /// @brief Return previously loaned request memory without sending it
    /// @param[in] loaned_memory Pointer to the loaned memory to return
    /// @return Expected containing void or error if the return failed
    auto return_request_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType>;

    /// @brief Send previously loaned request memory to the servers
    /// @param[in] loaned_memory Pointer to the loaned memory to send
    /// @note The memory must be initialized before sending
    /// @return Expected containing the sequence number of the request or error if sending failed
    auto send_request(void* loaned_memory) -> ::iox2::bb::Expected<uint64_t, ErrorType>;

    /// @brief Take the next response to a request of this client
    /// @return Expected containing the response if one was available
    auto take_response() -> ::iox2::bb::Expected<::iox2::bb::Optional<ClientResponse>, ErrorType>;

    /// @brief Return the memory of a taken response
    /// @param[in] loaned_memory Pointer to the loaned memory to return
    /// @return Expected containing void or error if the return failed
    auto return_response_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType>;

private:
    const std::string m_service;
    const rosidl_service_type_support_t* const m_typesupport;
    const std::string m_service_name;
    const rmw_qos_profile_t m_qos;

    ::iox2::bb::Optional<IdType> m_iox2_unique_id;
    ::iox2::bb::Optional<IceoryxService> m_iox2_service;
    ::iox2::bb::Optional<IceoryxClient> m_iox2_client;
    std::mutex m_mutex;
    SampleRegistry<IceoryxRequest> m_requests;
    std::map<uint64_t, IceoryxPendingResponse> m_pending_responses;
    SampleRegistry<IceoryxResponse> m_responses;
    uint64_t m_sequence_number{0};
};

} // namespace rmw::iox2

#endif
