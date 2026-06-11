use std::time::{SystemTime, UNIX_EPOCH};

use iceoryx2::prelude::*;
use rmw_iceoryx2_benchmark_msgs::msg::rmw::Sample;
use rosidl_runtime_rs::RmwMessage;

pub use rmw_iceoryx2_interoperability::MessageInfoHeader;

pub mod options;
pub mod stats;

pub const SERVICE_NAME: &str = "ros2://topics/benchmark";

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
pub struct Payload(pub Sample);

unsafe impl ZeroCopySend for Payload {
    unsafe fn type_name() -> &'static str {
        <Sample as RmwMessage>::TYPE_NAME
    }
}
