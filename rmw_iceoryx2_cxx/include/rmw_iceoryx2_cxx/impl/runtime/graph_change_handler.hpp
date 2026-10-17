// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_GRAPH_CHANGE_HANDLER_HPP_
#define RMW_IOX2_RUNTIME_GRAPH_CHANGE_HANDLER_HPP_

#include "iox2/file_descriptor.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>

namespace rmw::iox2
{

/// @brief Runs the callbacks registered with it when the graph changed
class RMW_PUBLIC GraphChangeHandler : public std::enable_shared_from_this<GraphChangeHandler>
{
public:
    using CallbackId = uint64_t;
    using OnGraphChange = std::function<void()>;

    /// @brief A registered callback, removed when the registration is destroyed
    class RMW_PUBLIC Registration
    {
    public:
        Registration(const Registration&) = delete;
        Registration(Registration&& other) noexcept;
        auto operator=(const Registration&) -> Registration& = delete;
        auto operator=(Registration&& other) noexcept -> Registration&;
        /// @brief Removes the callback, it is not called anymore after this returns
        ~Registration();

    private:
        friend class GraphChangeHandler;
        explicit Registration(std::shared_ptr<GraphChangeHandler> handler);

        static inline std::atomic<CallbackId> s_next_id{0};
        std::shared_ptr<GraphChangeHandler> m_handler;
        CallbackId m_id;
    };

    /// @brief Creates the handler, woken by the graph listener when the graph changed
    /// @param[in] graph_listener The graph listener of the handler
    explicit GraphChangeHandler(GraphListener graph_listener);
    GraphChangeHandler(const GraphChangeHandler&) = delete;
    GraphChangeHandler(GraphChangeHandler&&) = delete;
    auto operator=(const GraphChangeHandler&) -> GraphChangeHandler& = delete;
    auto operator=(GraphChangeHandler&&) -> GraphChangeHandler& = delete;
    ~GraphChangeHandler() = default;

    /// @brief Get the file descriptor that is readable when the graph changed
    /// @return The file descriptor of the graph listener
    auto file_descriptor() const -> ::iox2::FileDescriptorView;

    /// @brief Registers a callback
    /// @param[in] callback The callback, called on the callback thread
    /// @return The registration of the callback
    auto register_callback(OnGraphChange callback) -> Registration;

    /// @brief Calls every registered callback, if the graph changed since the last call
    void run_callbacks();

private:
    void remove(CallbackId id);

    std::mutex m_mutex;
    GraphListener m_graph_listener;
    std::map<CallbackId, OnGraphChange> m_callbacks;
};

} // namespace rmw::iox2

#endif
