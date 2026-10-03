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

## rmw api benchmarks

`rmw_iceoryx2_api_benchmarks` measures what single rmw operations cost,
independent of message traffic:

| recipe | measures |
|---|---|
| `idle-spin` | one `spin_some` of a `SingleThreadedExecutor` whose node has `subscriptions` idle subscriptions, which is the cost of `rmw_wait` with nothing ready |
| `guard-condition` | creating, triggering and destroying a guard condition, the time until a thread waiting on a guard condition wakes up after it is triggered, and the file descriptors per guard condition |
| `graph-change` | the time until a node's `wait_for_graph_change` returns after another context starts creating or destroying a publisher. Both contexts live in one process but use the same inter-process path as separate processes. Changes not seen within 1 s count as lost, and the run stops after three in a row |
| `graph-query` | `get_topic_names_and_types`, `get_node_names`, `count_publishers` and `get_publishers_info_by_topic` with `topics` topics published by another context |
| `node` | creating and destroying a node with the default node options, and the private memory and file descriptors per node with `nodes` nodes alive |
| `node-scaling` | the private memory and file descriptors per node as the nodes of one process grow to each count in `scaling`, and the node that fails to start |
| `endpoint` | creating and destroying a publisher and a subscription, and the private memory and file descriptors per publisher and per subscription with `endpoints` of each alive |
| `service` | creating and destroying a service, the round trip of a request and its response between two contexts, and the private memory and file descriptors per service with `services` alive |

```console
just -f src/rmw_iceoryx2/justfile run-benchmark idle-spin subscriptions=100
just -f src/rmw_iceoryx2/justfile run-benchmark guard-condition count=5000
just -f src/rmw_iceoryx2/justfile run-benchmark graph-change
just -f src/rmw_iceoryx2/justfile run-benchmark graph-query topics=500
just -f src/rmw_iceoryx2/justfile run-benchmark node nodes=16
just -f src/rmw_iceoryx2/justfile run-benchmark node-scaling scaling=1,10,30,100
just -f src/rmw_iceoryx2/justfile run-benchmark endpoint endpoints=100
just -f src/rmw_iceoryx2/justfile run-benchmark service rmw=rmw_fastrtps_cpp
```

`idle-spin` and `guard-condition` take `count` and `warmup` like the pairings.
`graph-change`, `graph-query`, `node`, `endpoint` and `service` take `rounds`
instead, since each round takes milliseconds. All of them take `rmw` to run the same measurement with
another rmw implementation, e.g. `rmw=rmw_fastrtps_cpp`, and so does the
`ros2-to-ros2` pairing. Every recipe takes `repeat` to run
the benchmark several times; with more than one run, a summary with the median
p50, its range and the median p99 of every report follows the runs. To see the
effect of a change, build and run them once on `rolling` and once on the branch,
on an otherwise idle machine, e.g. with `repeat=3`.

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
