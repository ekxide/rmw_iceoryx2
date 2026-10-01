// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Shared wire-contract definitions for interoperating with `rmw_iceoryx2`.
//!
//! Used natively by Rust peers and, via the cbindgen-generated C header, by the C++ rmw.

use std::ffi::{c_char, c_void};

mod constants;
pub use constants::MESSAGE_INFO_HEADER_TYPE_NAME;

/// Per-sample message info that the publisher propagates to the subscriber, mirroring the
/// sender-originated fields of `rmw_message_info_t`. Receiver-side fields (received timestamp,
/// reception sequence number) and the publisher GID (derived from the iceoryx2 publisher id)
/// are not carried here.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct MessageInfoHeader {
    /// System-time nanoseconds at which the sample was published.
    pub source_timestamp: i64,
    /// Per-publisher monotonically increasing sample counter.
    pub publication_sequence_number: u64,
}

// Lets the header be used as an iceoryx2 user header. The type name must match the one the C++
// rmw stamps (MESSAGE_INFO_HEADER_TYPE_NAME) so the service's header type is compatible across
// peers.
unsafe impl iceoryx2::prelude::ZeroCopySend for MessageInfoHeader {
    unsafe fn type_name() -> &'static str {
        MESSAGE_INFO_HEADER_TYPE_NAME
    }
}

#[repr(C)]
struct MessageTypeSupport {
    typesupport_identifier: *const c_char,
    data: *const c_void,
    func: *const c_void,
    get_type_hash_func: Option<unsafe extern "C" fn(*const MessageTypeSupport) -> *const TypeHash>,
}

#[repr(C)]
struct TypeHash {
    version: u8,
    value: [u8; 32],
}

/// Type hash of a message, as stored in the `ros.type_hash` service attribute.
pub unsafe fn type_hash(type_support: *const c_void) -> String {
    use std::fmt::Write;

    let type_support = type_support as *const MessageTypeSupport;
    let get_type_hash = (*type_support)
        .get_type_hash_func
        .expect("rosidl typesupport provides a type hash function");
    let type_hash = get_type_hash(type_support);

    let mut rihs = String::from("RIHS01_");
    for byte in (*type_hash).value {
        write!(rihs, "{byte:02x}").expect("writing to a String cannot fail");
    }
    rihs
}
