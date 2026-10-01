// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

// Canonical string constants shared by Rust consumers and (via build.rs → cbindgen
// `after_includes`) the generated C++ header. cbindgen cannot emit string constants itself,
// so this file is included by both `lib.rs` and `build.rs` to keep a single source of truth.

/// iceoryx2 user-header type name for `MessageInfoHeader`.
pub const MESSAGE_INFO_HEADER_TYPE_NAME: &str = "rmw_iceoryx2/MessageInfoHeader";
