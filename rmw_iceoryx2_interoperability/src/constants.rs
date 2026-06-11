// Canonical string constants shared by Rust consumers and (via build.rs → cbindgen
// `after_includes`) the generated C++ header. cbindgen cannot emit string constants itself,
// so this file is included by both `lib.rs` and `build.rs` to keep a single source of truth.

/// iceoryx2 user-header type name for `MessageInfoHeader`.
pub const MESSAGE_INFO_HEADER_TYPE_NAME: &str = "rmw_iceoryx2/MessageInfoHeader";
