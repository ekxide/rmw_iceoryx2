# benchmark

Applications measuring the one-way latency of `rmw_iceoryx2` at a configurable
publish rate, between ROS 2 nodes and native iceoryx2 nodes.

Complements [`performance_test`](../performance_test), which sweeps payload
sizes with the standardized tool: these applications instead isolate where time
is spent on the receive path (rclcpp executor vs. native iceoryx2 WaitSet) and
how wake-up latency behaves across publish rates — at low rates the numbers are
dominated by cold-start effects (CPU idle states, frequency scaling, cold
caches).

## Pairings

| recipe | publisher | subscriber |
|---|---|---|
| `ros2-to-ros2` | rclcpp | rclcpp |
| `ros2-to-iceoryx2` | rclcpp | native iceoryx2 |
| `iceoryx2-to-ros2` | native iceoryx2 | rclcpp |
| `iceoryx2-to-iceoryx2` | native iceoryx2 | native iceoryx2 (baseline without rclcpp) |

The native nodes speak the `rmw_iceoryx2` wire contract directly (service name,
QoS attributes, static config, message-info user header, event notification),
so each side can be swapped independently.

## Usage

From the workspace root:

```console
just -f src/rmw_iceoryx2/justfile build-benchmark
just -f src/rmw_iceoryx2/justfile run-benchmark ros2-to-iceoryx2
```

Each pairing takes named parameters `rate` (Hz), `count` (samples) and
`warmup` (received samples excluded from the statistics), in any order:

```console
just -f src/rmw_iceoryx2/justfile run-benchmark ros2-to-iceoryx2 rate=100 count=1000 warmup=50
```

Defaults: 10000 samples at 1000 Hz, 100 warmup.

The subscriber prints a report when the final sample arrives (or after 2 s of
silence, should it be lost):

```
REPORT · iceoryx2 subscriber
samples 10000/10000 · lost 0 · warmup 100

  min    902 ns ▏
  p50    7.0 µs █▎
  p90   19.2 µs ███▋
  p99   32.3 µs ██████▏
  max  198.9 µs ██████████████████████████████████████

  mean  10.0 µs
```

The bars are scaled to `max`, so the spread between `p50` and `p99` — the
latency jitter — is visible at a glance.

## Methodology & caveats

- Latency is `receive time − source_timestamp`. The ROS 2 subscriber measures
  in the message callback against the rmw-stamped `source_timestamp`; the
  native subscriber reads the same stamp from the message-info user header.
  Both ends use the system clock.
- The sequence number travels in the payload, so loss detection and run
  completion work with any rmw implementation.
- The topic uses rclcpp's default QoS (RELIABLE, KEEP_LAST 10), which maps to
  a drop-oldest queue of depth 10. A subscriber that cannot keep up with the
  publish rate loses samples; the report counts them.
- At low rates (e.g. the default examples' 1 Hz) every wake-up pays cold-start
  costs.
