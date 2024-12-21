// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_COMMON_ENSURE_HPP_
#define RMW_IOX2_COMMON_ENSURE_HPP_

#include "rmw_iceoryx2_cxx/impl/common/defaults.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/message/typesupport.hpp"

#define RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, return_value) RCUTILS_CHECK_ALLOCATOR(allocator, return return_value)

#define RMW_IOX2_ENSURE_IMPLEMENTATION(implementation_identifier, return_value)                                        \
    if (implementation_identifier != rmw_get_implementation_identifier()) {                                            \
        RMW_IOX2_CHAIN_ERROR_MSG(#implementation_identifier " invalid for rmw_iceoryx2_cxx");                          \
        return return_value;                                                                                           \
    }

// Reimplemented to use RMW_IOX2_CHAIN_ERROR_MSG
#define RMW_IOX2_CHECK_FOR_NULL_WITH_MSG(value, msg, error_statement)                                                  \
    do {                                                                                                               \
        if (NULL == value) {                                                                                           \
            RMW_IOX2_CHAIN_ERROR_MSG(msg);                                                                             \
            error_statement;                                                                                           \
        }                                                                                                              \
    } while (0)

#define RMW_IOX2_CHECK_IS_NULL_WITH_MSG(value, msg, error_statement)                                                   \
    do {                                                                                                               \
        if (NULL != value) {                                                                                           \
            RMW_IOX2_CHAIN_ERROR_MSG(msg);                                                                             \
            error_statement;                                                                                           \
        }                                                                                                              \
    } while (0)

// Single macro for checking that argument IS null
#define RMW_IOX2_ENSURE_NULL(argument, return_value)                                                                   \
    RMW_IOX2_CHECK_IS_NULL_WITH_MSG(argument, #argument " argument is not null", return return_value)


#define RMW_IOX2_ENSURE_NOT_NULL(argument, return_value)                                                               \
    RMW_IOX2_CHECK_FOR_NULL_WITH_MSG(argument, #argument " argument is null", return return_value)


#define RMW_IOX2_ENSURE_ZERO_INITIALIZED(element, return_value)                                                        \
    if (element->implementation_identifier != nullptr || element->impl != nullptr) {                                   \
        RMW_IOX2_CHAIN_ERROR_MSG(#element " must be zero initialized");                                                \
        return return_value;                                                                                           \
    }

#define RMW_IOX2_ENSURE_NOT_ZERO_INITIALIZED(element, return_value)                                                    \
    if (element->implementation_identifier == nullptr && element->impl == nullptr) {                                   \
        RMW_IOX2_CHAIN_ERROR_MSG(#element " must not be zero initialized");                                            \
        return return_value;                                                                                           \
    }

#define RMW_IOX2_ENSURE_INITIALIZED(element, return_value)                                                             \
    if (element->impl == nullptr) {                                                                                    \
        RMW_IOX2_CHAIN_ERROR_MSG(#element " must be initialized");                                                     \
        return return_value;                                                                                           \
    }

#define RMW_IOX2_ENSURE_NOT_INITIALIZED(element, return_value)                                                         \
    if (element->impl != nullptr) {                                                                                    \
        RMW_IOX2_CHAIN_ERROR_MSG(#element " must not be initialized");                                                 \
        return return_value;                                                                                           \
    }

#define RMW_IOX2_ENSURE_VALID_NODE_NAME(name, return_value)                                                            \
    {                                                                                                                  \
        int validation_result = RMW_NODE_NAME_VALID;                                                                   \
        rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);                                     \
        if (RMW_RET_OK != ret) {                                                                                       \
            return return_value;                                                                                       \
        }                                                                                                              \
        if (RMW_NODE_NAME_VALID != validation_result) {                                                                \
            const char* reason = rmw_node_name_validation_result_string(validation_result);                            \
            RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node name: %s", reason);                                     \
            return return_value;                                                                                       \
        }                                                                                                              \
    }

#define RMW_IOX2_ENSURE_VALID_NAMESPACE(namespace_, return_value)                                                      \
    {                                                                                                                  \
        int validation_result = RMW_NAMESPACE_VALID;                                                                   \
        rmw_ret_t ret = rmw_validate_namespace(namespace_, &validation_result, nullptr);                               \
        if (RMW_RET_OK != ret) {                                                                                       \
            return return_value;                                                                                       \
        }                                                                                                              \
        if (RMW_NAMESPACE_VALID != validation_result) {                                                                \
            const char* reason = rmw_namespace_validation_result_string(validation_result);                            \
            RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node namespace: %s", reason);                                \
            return return_value;                                                                                       \
        }                                                                                                              \
    }

#define RMW_IOX2_ENSURE_VALID_TOPIC_NAME(topic_name, return_value)                                                     \
    {                                                                                                                  \
        int validation_result = RMW_TOPIC_VALID;                                                                       \
        rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);                         \
        if (RMW_RET_OK != ret) {                                                                                       \
            return return_value;                                                                                       \
        }                                                                                                              \
        if (RMW_TOPIC_VALID != validation_result) {                                                                    \
            const char* reason = rmw_full_topic_name_validation_result_string(validation_result);                      \
            RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);                                    \
            return return_value;                                                                                       \
        }                                                                                                              \
    }

#define RMW_IOX2_ENSURE_VALID_SERVICE_NAME(service_name, return_value)                                                 \
    RMW_IOX2_ENSURE_VALID_TOPIC_NAME(service_name, return_value)

#define RMW_IOX2_ENSURE_VALID_QOS(qos, return_value)                                                                   \
    if (qos->history == RMW_QOS_POLICY_HISTORY_UNKNOWN || qos->reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN       \
        || qos->durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN                                                        \
        || qos->liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN) {                                                     \
        return return_value;                                                                                           \
    }

#define RMW_IOX2_ENSURE_VALID_TYPESUPPORT(type_support, return_value)                                                  \
    auto handle = get_message_typesupport_handle(type_support, RMW_ICEORYX2_CXX_TYPESUPPORT_C);                        \
    if (!handle) {                                                                                                     \
        auto prev_error_string = rcutils_get_error_string();                                                           \
        rcutils_reset_error();                                                                                         \
        handle = get_message_typesupport_handle(type_support, RMW_ICEORYX2_CXX_TYPESUPPORT_CPP);                       \
        if (!handle) {                                                                                                 \
            auto error_string = rcutils_get_error_string();                                                            \
            rcutils_reset_error();                                                                                     \
            RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING(                                                               \
                "Invalid typesupport: %s\n%s", prev_error_string.str, error_string.str);                               \
            return return_value;                                                                                       \
        }                                                                                                              \
    }

#define RMW_IOX2_ENSURE_VALID_SERVICE_TYPESUPPORT(type_support, return_value)                                          \
    auto handle = get_service_typesupport_handle(type_support, RMW_ICEORYX2_CXX_TYPESUPPORT_C);                        \
    if (!handle) {                                                                                                     \
        auto prev_error_string = rcutils_get_error_string();                                                           \
        rcutils_reset_error();                                                                                         \
        handle = get_service_typesupport_handle(type_support, RMW_ICEORYX2_CXX_TYPESUPPORT_CPP);                       \
        if (!handle) {                                                                                                 \
            auto error_string = rcutils_get_error_string();                                                            \
            rcutils_reset_error();                                                                                     \
            RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING(                                                               \
                "Invalid typesupport: %s\n%s", prev_error_string.str, error_string.str);                               \
            return return_value;                                                                                       \
        }                                                                                                              \
    }

#define RMW_IOX2_ENSURE_CAN_LOAN(endpoint, return_value)                                                               \
    if (!endpoint->can_loan_messages) {                                                                                \
        return return_value;                                                                                           \
    }

#define RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(arr, ret)                                                                    \
    if ((arr).size != 0 || (arr).data != NULL || (arr).allocator.allocate != NULL                                      \
        || (arr).allocator.deallocate != NULL || (arr).allocator.reallocate != NULL                                    \
        || (arr).allocator.zero_allocate != NULL || (arr).allocator.state != NULL) {                                   \
        return ret;                                                                                                    \
    }

#define RMW_IOX2_ENSURE_OK(result)                                                                                     \
    if (result != RMW_RET_OK) {                                                                                        \
        return result;                                                                                                 \
    }

#endif
