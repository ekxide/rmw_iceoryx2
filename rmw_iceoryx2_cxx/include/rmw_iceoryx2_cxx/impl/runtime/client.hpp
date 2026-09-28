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
#include "rmw_iceoryx2_interoperability/rmw_iceoryx2_interoperability.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

namespace rmw::iox2
{

class Client;

template <>
struct Error<Client>
{
    using Type = ClientError;
};

/// @brief Implementation of the RMW client for iceoryx2
///
/// @details Requests and responses are exchanged as CDR-serialized payloads of an iceoryx2
///          request-response service.
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

private:
    const std::string m_service;
    const rosidl_service_type_support_t* const m_typesupport;
    const std::string m_service_name;
    const rmw_qos_profile_t m_qos;

    ::iox2::bb::Optional<IdType> m_iox2_unique_id;
    ::iox2::bb::Optional<IceoryxService> m_iox2_service;
    ::iox2::bb::Optional<IceoryxClient> m_iox2_client;
};

} // namespace rmw::iox2

#endif
