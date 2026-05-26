# Quality of Service

This page describes how `rmw_iceoryx2` interprets ROS 2 Quality of Service
(QoS) settings, how endpoints on the same topic agree on QoS, and the
environment variables you can use to tune behavior at process start.

## Overview

- **First endpoint to a topic locks in its QoS.** Whether it's a publisher
  or a subscriber, the first one to come up establishes the QoS for that
  topic. All endpoints that join later must agree, as per `iceoryx2`
  constraints.
- **Mismatching QoS is a hard error by default.** A second endpoint with
  different QoS settings fails to create with a descriptive error
  message naming the policy that differs.
- **Some QoS policies are not honored.** iceoryx2 is a shared-memory
  middleware focused on intra-host communication. Policies that imply
  cross-host coordination are recorded but not enforced. Recorded
  QoS will be applied at the network boundary once `iceoryx2` tunnels
  are integrated.

The defaults are designed with the goal that well-behaved applications using
`rmw_qos_profile_default` "just work." 

## Mapping

| Policy | Supported | Notes |
|---|---|---|
| `history` | `KEEP_LAST` only | `KEEP_ALL` is rejected at endpoint creation — iceoryx2's per-subscriber queues are fixed-size at service creation, so the unbounded queueing `KEEP_ALL` implies is not available. Use `KEEP_LAST` with a sufficient `depth`. |
| `depth` | yes | Drives the underlying buffer size. |
| `reliability` | yes | `RELIABLE` and `BEST_EFFORT` are both honored. |
| `durability` | yes | `VOLATILE` and `TRANSIENT_LOCAL` are both honored. |
| `liveliness` | `AUTOMATIC` only | `MANUAL_BY_TOPIC` is accepted but not enforced. A warning is logged. |
| `liveliness_lease_duration` | no | Recorded but not enforced. |
| `deadline` | no | Recorded but not enforced. |
| `lifespan` | no | Recorded but not enforced. |
| `avoid_ros_namespace_conventions` | yes | Affects topic-name resolution only. |

> [!NOTE]
> "Recorded but not enforced" means the policy value is preserved (a
> subscriber requesting `deadline=1s` will be told that the publisher
> also has `deadline=1s` if that's what was set), but `rmw_iceoryx2`
> itself does not detect or report deadline misses. These policies
> will be applied at the network boundary once `iceoryx2` tunnels
> are integrated.

> [!NOTE]
> Support for `deadline` and `liveliness` may be added in later iterations if
> demand exists.

## Matching Mode

The matching mode controls what happens when an endpoint joins a topic
that already has endpoints with different QoS. Set via the
`RMW_IOX2_QOS_MATCHING` environment variable:

| Value | Behavior |
|---|---|
| `strict` (default) | The new endpoint's QoS must match the existing endpoints' QoS exactly. Mismatch causes endpoint creation to fail. |
| `adoptive` | The new endpoint substitutes its own QoS with that of the existing endpoints and joins successfully. If no endpoints exist yet, the new endpoint creates the topic with its own QoS (same as `strict`). |

```sh
# Production / default — fail fast on configuration drift
unset RMW_IOX2_QOS_MATCHING

# Introspection tooling — auto-match whatever's already there
RMW_IOX2_QOS_MATCHING=adoptive ros2 topic echo /my_topic
```

`adoptive` is intended for CLI and debugging tooling such as
`ros2 topic echo`, `ros2 topic pub`, and `rqt_graph`. These tools use
`rmw_qos_profile_default` (which is `RELIABLE`), so they fail to attach
to publishers using e.g. `rmw_qos_profile_sensor_data` (which is
`BEST_EFFORT`) under the default `strict` matching. Setting
`RMW_IOX2_QOS_MATCHING=adoptive` in the tool's environment lets the tool
inherit the publisher's actual QoS instead.

`ros2 topic info -v <topic>` shows the actual QoS in use on a topic; it
can be used to introspect what an existing endpoint resolved.

> [!NOTE]
> `RMW_IOX2_QOS_MATCHING` is process-scoped: the mode applies to every
> topic in the process. If you need adoptive behavior on one
> topic only, structure your program so that the relevant endpoint
> creation happens in a separate process.

## Configuration

Environment variables can be used to apply further configuration to `iceoryx2`
resource usage.

All variables are read once during `rmw_init_options_init`, i.e., when
the rmw context is created. Changes during a running process have no
effect.

| Variable | Default | Effect |
|---|---|---|
| `RMW_IOX2_QOS_MATCHING` | `strict` | `strict` or `adoptive`. See above. |
| `RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC` | `32` | Upper bound on concurrent publishers per topic. |
| `RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC` | `32` | Upper bound on concurrent subscribers per topic. |
| `RMW_IOX2_MAX_NODES_PER_SERVICE` | `32` | Upper bound on distinct processes participating in a topic. |

> [!IMPORTANT]
> The `MAX_*` settings size the service's per-port tracking slots in
> shared memory — one slot per possible publisher / subscriber / node,
> reserved at service creation regardless of whether it's occupied.
> They are *not* part of the topic's QoS, and they do not size message
> payload memory (which is driven by `depth` and message size).
>
> The first endpoint on a topic fixes the slot count; endpoints joining
> later may request the same value or lower, but never higher — the
> slot tables cannot grow at runtime.
>
> When a late joiner requests a higher cap than the service was sized
> for, endpoint creation fails. The variable must be raised in **all**
> processes — bumping it in one process does not enlarge a service that
> another process already created with the lower value.

## Common errors

### `QoS mismatch on '<topic>': <key> [attribute=<existing>, requested=<requested>]`

A new endpoint tried to join a topic that already exists, but its QoS
differs. The message names every policy that differs, with the value
already on the service and the value requested by the new endpoint. To
resolve:

- Update the new endpoint's QoS profile to match the existing one.
- Set `RMW_IOX2_QOS_MATCHING=adoptive` in the new endpoint's
  environment (recommended only for CLI/debug tools).
- Determine the actual QoS of the existing endpoints with
  `ros2 topic info -v <topic>`.

### `ExceedsMaxSupportedPublishers` / `OpenDoesNotSupportRequestedAmountOfPublishers`

A publisher tried to come up on a topic whose service was sized for
fewer publishers than the running count plus this one — for example,
the 33rd publisher on a service created with the default cap of 32.
iceoryx2 pre-allocates the slot table at service creation and cannot
grow it at runtime.

`ExceedsMaxSupportedPublishers` is emitted when the cap is reached
under matching settings; `OpenDoesNotSupportRequestedAmountOfPublishers`
is emitted when this endpoint requested a *higher* cap than the
existing service was sized for.

Raise the limit at the offending process's init time. Note that **every
process on the topic must use the same limits or lower** — the first
process to create the service fixes the upper bound:

```sh
RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC=128 ros2 launch my_pkg my_launch.py
```

Analogous errors and overrides exist for subscribers
(`ExceedsMaxSupportedSubscribers` /
`OpenDoesNotSupportRequestedAmountOfSubscribers`,
`RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC`) and nodes
(`OpenExceedsMaxNumberOfNodes` /
`OpenDoesNotSupportRequestedAmountOfNodes`,
`RMW_IOX2_MAX_NODES_PER_SERVICE`).

### Warnings about unsupported policies

When you create an endpoint with `deadline`, `lifespan`,
`liveliness_lease_duration`, or `liveliness == MANUAL_BY_TOPIC` set to
non-default values, a warning is logged:

```
QoS policy 'deadline' (=5:0) on topic '/my_topic' is not honored by the iceoryx2 transport
```

The endpoint is created successfully — these are informational warnings,
not failures. The QoS values are visible through `rmw_*_get_actual_qos`
so downstream code can implement these policies in user space if needed.
