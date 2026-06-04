use core::time::Duration;

use iceoryx2::prelude::*;
use iceoryx2_interoperation_demo_msgs::msg::rmw::TransmissionData;
use iceoryx2_interoperation_demo_nodes::{Payload, SERVICE_NAME};

const CYCLE_TIME: Duration = Duration::from_secs(1);

// Must match the values rmw_iceoryx2 uses (DEFAULT_MAX_* and rclcpp's default QoS depth).
const PAYLOAD_ALIGNMENT: usize = 8;
const MAX_PUBLISHERS: usize = 32;
const MAX_SUBSCRIBERS: usize = 32;
const MAX_NODES: usize = 32;
const HISTORY_SIZE: usize = 10;
const SUBSCRIBER_MAX_BUFFER_SIZE: usize = 10;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    set_log_level_from_env_or(LogLevel::Info);

    let node = NodeBuilder::new().create::<ipc::Service>()?;

    // The static config and QoS attributes must match what rmw_iceoryx2 derives from rclcpp's
    // default profile (KeepLast 10, reliable) so either peer may create the service first.
    let qos = AttributeVerifier::new()
        .require(
            &"rmw.qos.local.history".try_into()?,
            &"keep_last:10".try_into()?,
        )?
        .require(
            &"rmw.qos.local.reliability".try_into()?,
            &"reliable".try_into()?,
        )?
        .require(
            &"rmw.qos.local.durability".try_into()?,
            &"volatile".try_into()?,
        )?
        .require(
            &"rmw.qos.local.deadline".try_into()?,
            &"duration:0:0".try_into()?,
        )?
        .require(
            &"rmw.qos.local.lifespan".try_into()?,
            &"duration:0:0".try_into()?,
        )?
        .require(
            &"rmw.qos.local.liveliness".try_into()?,
            &"automatic:0:0".try_into()?,
        )?;

    let service = node
        .service_builder(&SERVICE_NAME.try_into()?)
        .publish_subscribe::<Payload>()
        .payload_alignment(Alignment::new(PAYLOAD_ALIGNMENT).unwrap())
        .max_publishers(MAX_PUBLISHERS)
        .max_subscribers(MAX_SUBSCRIBERS)
        .max_nodes(MAX_NODES)
        .history_size(HISTORY_SIZE)
        .subscriber_max_buffer_size(SUBSCRIBER_MAX_BUFFER_SIZE)
        .enable_safe_overflow(false)
        .open_or_create_with_attributes(&qos)?;
    let publisher = service.publisher_builder().create()?;

    // rmw subscribers wake on an event notification, not by polling.
    let event = node
        .service_builder(&SERVICE_NAME.try_into()?)
        .event()
        .open_or_create()?;
    let notifier = event.notifier_builder().create()?;

    let mut counter: i32 = 0;
    while node.wait(CYCLE_TIME).is_ok() {
        counter += 1;
        let data = TransmissionData {
            x: counter,
            y: counter * 3,
            funky: f64::from(counter) * 812.12,
        };

        let sample = publisher.loan_uninit()?;
        sample.write_payload(Payload(data.clone())).send()?;
        notifier.notify()?;

        println!("sent: {data:?}");
    }

    Ok(())
}
