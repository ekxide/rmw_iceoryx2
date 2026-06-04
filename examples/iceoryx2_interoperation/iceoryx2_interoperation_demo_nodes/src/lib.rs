use iceoryx2::prelude::*;
use iceoryx2_interoperation_demo_msgs::msg::rmw::TransmissionData;
use rosidl_runtime_rs::RmwMessage;

pub const SERVICE_NAME: &str = "ros2://topics/transmission_data";

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
