//! Shared wire-contract definitions for interoperating with `rmw_iceoryx2`.
//!
//! Used natively by Rust peers and, via the cbindgen-generated C header, by the C++ rmw.

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
