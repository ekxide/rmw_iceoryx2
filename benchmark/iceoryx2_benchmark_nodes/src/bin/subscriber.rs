use core::time::Duration;
use std::time::Instant;

use iceoryx2::prelude::*;
use iceoryx2_benchmark_nodes::{
    options, stats::LatencyRecorder, system_time_nanos, MessageInfoHeader, Payload, SERVICE_NAME,
};

// Must match the values rmw_iceoryx2 uses (DEFAULT_MAX_* and rclcpp's default QoS depth).
const PAYLOAD_ALIGNMENT: usize = 8;
const MAX_PUBLISHERS: usize = 32;
const MAX_SUBSCRIBERS: usize = 32;
const MAX_NODES: usize = 32;
const HISTORY_SIZE: usize = 10;
const SUBSCRIBER_MAX_BUFFER_SIZE: usize = 10;

const IDLE_CHECK_INTERVAL: Duration = Duration::from_millis(100);
const IDLE_TIMEOUT: Duration = Duration::from_secs(2);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    set_log_level_from_env_or(LogLevel::Warn);
    let options = options::parse();

    let node = NodeBuilder::new().create::<ipc::Service>()?;

    // The static config and QoS attributes must match what rmw_iceoryx2 derives from rclcpp's
    // default profile (KeepLast 10, reliable) so either peer may create the service first.
    let qos = AttributeVerifier::new()
        .require(
            &"ros.qos.history".try_into()?,
            &"keep_last:10".try_into()?,
        )?
        .require(
            &"ros.qos.reliability".try_into()?,
            &"reliable".try_into()?,
        )?
        .require(
            &"ros.qos.durability".try_into()?,
            &"volatile".try_into()?,
        )?
        .require(
            &"ros.qos.deadline".try_into()?,
            &"duration:0:0".try_into()?,
        )?
        .require(
            &"ros.qos.lifespan".try_into()?,
            &"duration:0:0".try_into()?,
        )?
        .require(
            &"ros.qos.liveliness".try_into()?,
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
        .enable_safe_overflow(true)
        .open_or_create_with_attributes(&qos)?;
    let subscriber = service.subscriber_builder().create()?;

    // rmw_iceoryx2 signals the topic's event service on every send; wake on it instead of polling.
    let event = node
        .service_builder(&SERVICE_NAME.try_into()?)
        .event()
        .open_or_create()?;
    let listener = event.listener_builder().create()?;

    let waitset = WaitSetBuilder::new().create::<ipc::Service>()?;
    let listener_guard = waitset.attach_notification(&listener)?;
    let _tick_guard = waitset.attach_interval(IDLE_CHECK_INTERVAL)?;

    let mut recorder = LatencyRecorder::new(options.warmup, options.count);
    let mut last_receive = Instant::now();

    let on_event = |id: WaitSetAttachmentId<ipc::Service>| -> CallbackProgression {
        // Drain the listener when it fired, otherwise the WaitSet wakes us again immediately.
        if id.has_event_from(&listener_guard) {
            listener.try_wait(|_| {}).unwrap();
        }
        while let Some(sample) = subscriber.receive().unwrap() {
            let info = sample.user_header();
            // source_timestamp is on the publisher's system clock; the difference is the one-way
            // latency only if both peers share a clock.
            recorder.record(system_time_nanos() - info.source_timestamp);
            last_receive = Instant::now();

            if sample.payload().0.sequence + 1 >= options.count {
                return CallbackProgression::Stop;
            }
        }
        if recorder.received() > 0 && last_receive.elapsed() > IDLE_TIMEOUT {
            eprintln!("no samples for 2s - reporting incomplete run");
            return CallbackProgression::Stop;
        }
        CallbackProgression::Continue
    };

    println!("expecting {} samples", options.count);
    waitset.wait_and_process(on_event)?;

    println!("{}", recorder.report("iceoryx2 subscriber", options.count));

    Ok(())
}
