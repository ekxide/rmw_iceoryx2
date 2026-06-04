use iceoryx2::prelude::*;
use iceoryx2_interoperation_demo_nodes::{system_time_nanos, MessageInfoHeader, Payload, SERVICE_NAME};

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
        .user_header::<MessageInfoHeader>()
        .payload_alignment(Alignment::new(PAYLOAD_ALIGNMENT).unwrap())
        .max_publishers(MAX_PUBLISHERS)
        .max_subscribers(MAX_SUBSCRIBERS)
        .max_nodes(MAX_NODES)
        .history_size(HISTORY_SIZE)
        .subscriber_max_buffer_size(SUBSCRIBER_MAX_BUFFER_SIZE)
        .enable_safe_overflow(false)
        .open_or_create_with_attributes(&qos)?;
    let subscriber = service.subscriber_builder().create()?;

    // rmw_iceoryx2 signals the topic's event service on every send; wake on it instead of polling.
    let event = node
        .service_builder(&SERVICE_NAME.try_into()?)
        .event()
        .open_or_create()?;
    let listener = event.listener_builder().create()?;

    let waitset = WaitSetBuilder::new().create::<ipc::Service>()?;
    let _guard = waitset.attach_notification(&listener)?;

    let on_event = |_id: WaitSetAttachmentId<ipc::Service>| -> CallbackProgression {
        // Drain all notifications, otherwise the WaitSet wakes us again immediately (busy loop).
        listener.try_wait_all(|_| {}).unwrap();
        while let Some(sample) = subscriber.receive().unwrap() {
            let info = sample.user_header();
            // source_timestamp is on the publisher's system clock; the difference is the one-way
            // latency only if both peers share a clock (e.g. same host).
            let latency_us = (system_time_nanos() - info.source_timestamp) as f64 / 1000.0;
            println!(
                "received: {:?} (seq={}, latency={:.1}us)",
                sample.payload().0,
                info.publication_sequence_number,
                latency_us,
            );
        }
        CallbackProgression::Continue
    };

    println!("subscriber ready");
    waitset.wait_and_process(on_event)?;

    Ok(())
}
