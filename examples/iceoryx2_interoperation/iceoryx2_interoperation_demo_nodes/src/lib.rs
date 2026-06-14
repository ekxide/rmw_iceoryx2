use std::ffi::{c_char, c_void};
use std::time::{SystemTime, UNIX_EPOCH};

use iceoryx2::prelude::*;
use rmw_iceoryx2_interoperation_demo_msgs::msg::rmw::TransmissionData;
use rosidl_runtime_rs::RmwMessage;

pub use rmw_iceoryx2_interoperability::MessageInfoHeader;

pub mod pretty;

pub const SERVICE_NAME: &str = "ros2://topics/transmission_data";

/// System-time nanoseconds since the Unix epoch, matching the clock `rmw_iceoryx2` stamps into
/// `MessageInfoHeader::source_timestamp` (`rcutils_system_time_now`).
pub fn system_time_nanos() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|elapsed| elapsed.as_nanos() as i64)
        .unwrap_or(0)
}

// Same memory layout as the rosidl-generated struct, carrying the rosidl type name so it
// matches the identity rmw_iceoryx2 stamps on the iceoryx2 service.
#[repr(transparent)]
#[derive(Debug)]
pub struct Payload(pub TransmissionData);

unsafe impl ZeroCopySend for Payload {
    unsafe fn type_name() -> &'static str {
        <TransmissionData as RmwMessage>::TYPE_NAME
    }
}

// Layout-compatible head of `rosidl_message_type_support_t`; only the fields up
// to `get_type_hash_func` are declared, which is all that is read.
#[repr(C)]
struct MessageTypeSupport {
    typesupport_identifier: *const c_char,
    data: *const c_void,
    func: *const c_void,
    get_type_hash_func: Option<unsafe extern "C" fn(*const MessageTypeSupport) -> *const TypeHash>,
}

// Mirror of `rosidl_type_hash_t`.
#[repr(C)]
struct TypeHash {
    version: u8,
    value: [u8; 32],
}

/// The REP-2011 type hash of `TransmissionData` as a RIHS string
/// (`RIHS01_<hex>`), read from the message's rosidl typesupport. This is the
/// same value `rmw_iceoryx2` stamps on the service as the `ros.type_hash`
/// attribute, so it must be mirrored for a compatible open.
pub fn type_hash() -> String {
    use std::fmt::Write;

    // SAFETY: `get_type_support()` returns a valid 'static rosidl type support
    // handle whose `get_type_hash_func` yields a pointer to a 'static type hash.
    unsafe {
        let type_support = TransmissionData::get_type_support() as *const MessageTypeSupport;
        let get_type_hash = (*type_support)
            .get_type_hash_func
            .expect("rosidl typesupport provides a type hash function");
        let type_hash = get_type_hash(type_support);

        // Matches `rosidl_stringify_type_hash`: "RIHS01_" + lowercase hex bytes.
        let mut rihs = String::from("RIHS01_");
        for byte in (*type_hash).value {
            write!(rihs, "{byte:02x}").expect("writing to a String cannot fail");
        }
        rihs
    }
}
