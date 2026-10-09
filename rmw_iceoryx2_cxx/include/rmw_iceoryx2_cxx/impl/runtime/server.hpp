// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_SERVER_HPP_
#define RMW_IOX2_RUNTIME_SERVER_HPP_

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

#include <array>
#include <map>
#include <mutex>
#include <unordered_map>

namespace rmw::iox2
{

class Server;

template <>
struct Error<Server>
{
    using Type = ServerError;
};

using ClientId = std::array<uint8_t, ::iox2::UNIQUE_PORT_ID_LENGTH>;

/// @brief A request taken by a server, valid until it is responded to or its client disconnects
struct ServerRequest
{
    uint8_t* bytes;
    size_t number_of_bytes;
    ::rmw_iceoryx2_interoperability::MessageInfoHeader message_info;
    ClientId client_id;
};

/// @brief Implementation of the RMW service for iceoryx2
///
/// @details Requests and responses are exchanged as CDR-serialized payloads of an iceoryx2
///          request-response service. The user header carries the ROS sequence number of the
///          request and the source timestamp. All operations are thread-safe, as the RMW API
///          requires.
class RMW_PUBLIC Server
{
public:
    using Payload = ::iox2::bb::Slice<::iox2::CustomPayloadMarker>;
    using UserHeader = ::rmw_iceoryx2_interoperability::MessageInfoHeader;
    using ErrorType = Error<Server>::Type;

private:
    using RequestId = std::pair<ClientId, uint64_t>;
    using IceoryxServer = Iceoryx2::InterProcess::Server<Payload, UserHeader>;
    using IceoryxActiveRequest = Iceoryx2::InterProcess::ActiveRequest<Payload, UserHeader>;
    using IceoryxResponse = Iceoryx2::InterProcess::ResponseMutUninit<Payload, UserHeader>;

public:
    /// @brief Constructor for Server
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] node The node that owns this server
    /// @param[in] service The name of the ROS service to serve
    /// @param[in] type_support The service typesupport
    /// @param[in] qos The QoS requested for this server
    Server(CreationLock,
           ::iox2::bb::Optional<ErrorType>& error,
           Node& node,
           const char* service,
           const rosidl_service_type_support_t* type_support,
           const rmw_qos_profile_t& qos);

    /// @brief Get the name of the ROS service
    /// @return The service name as string
    auto service() const -> const std::string&;

    /// @brief Get the typesupport used by the server
    /// @return Pointer to the typesupport stored in the loaded typesupport library
    auto typesupport() const -> const rosidl_service_type_support_t*;

    /// @brief Get the service name used internally, required for matching via iceoryx2
    /// @return The service name as string
    auto service_name() const -> const std::string&;

    /// @brief Get the QoS requested for this server
    /// @return Reference to the QoS
    auto qos() const -> const rmw_qos_profile_t&;

    /// @brief Take the next request sent by a client
    /// @return Expected containing the request if one was available
    auto take_request() -> ::iox2::bb::Expected<::iox2::bb::Optional<ServerRequest>, ErrorType>;

    /// @brief Drop a taken request without responding to it
    /// @param[in] client_id The id of the client that sent the request
    /// @param[in] sequence_number The sequence number of the request
    auto discard_request(const ClientId& client_id, uint64_t sequence_number) -> void;

    /// @brief Loan memory for the response to a taken request
    /// @param[in] client_id The id of the client that sent the request
    /// @param[in] sequence_number The sequence number of the request
    /// @param[in] number_of_bytes Required buffer size in bytes
    /// @return Expected containing pointer to the loaned memory, or empty if the client is gone
    auto loan_response(const ClientId& client_id, uint64_t sequence_number, uint64_t number_of_bytes)
        -> ::iox2::bb::Expected<::iox2::bb::Optional<void*>, ErrorType>;

    /// @brief Return previously loaned response memory without sending it
    /// @param[in] loaned_memory Pointer to the loaned memory to return
    /// @return Expected containing void or error if the return failed
    auto return_response_loan(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType>;

    /// @brief Send previously loaned response memory to the client of the request
    /// @param[in] loaned_memory Pointer to the loaned memory to send
    /// @note The memory must be initialized before sending
    /// @return Expected containing void or error if sending failed
    auto send_response(void* loaned_memory) -> ::iox2::bb::Expected<void, ErrorType>;

private:
    const std::string m_service;
    const rosidl_service_type_support_t* const m_typesupport;
    const std::string m_service_name;
    const rmw_qos_profile_t m_qos;

    std::mutex m_mutex;
    ::iox2::bb::Optional<IceoryxServer> m_iox2_server;
    std::map<RequestId, IceoryxActiveRequest> m_active_requests;
    SampleRegistry<IceoryxResponse> m_responses;
    std::unordered_map<const uint8_t*, RequestId> m_response_requests;
};

} // namespace rmw::iox2

#endif
