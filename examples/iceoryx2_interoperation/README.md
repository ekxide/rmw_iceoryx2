# iceoryx2 interoperation demo

A ROS 2 application (running on `rmw_iceoryx2_cxx`) and a vanilla iceoryx2 Rust
application exchanging `TransmissionData` over shared memory — no DDS, no CDR, raw
zero-copy. Any publisher interoperates with any subscriber.

> [!WARNING]
> Work in progress. This example only demonstrates that ROS 2 ↔ iceoryx2
> interoperation is feasible. The iceoryx2-side setup shown here (manually
> mirroring the rmw's QoS attributes, static config, and type identity) is
> deliberately low-level and will be made ergonomic before it is recommended for
> general use. As a ROS 2 user you do not need any of this — run the ROS nodes as
> usual; only the native iceoryx2 peer carries the boilerplate for now.

## Packages

| Package                                  | Build  | Contents                                  |
| ---------------------------------------- | ------ | ----------------------------------------- |
| `iceoryx2_interoperation_demo_msgs`      | colcon | `TransmissionData` interface (shared)     |
| `rmw_iceoryx2_interoperation_demo_nodes` | colcon | ROS 2 nodes (rclcpp)                      |
| `iceoryx2_interoperation_demo_nodes`     | cargo  | native iceoryx2 apps (no `package.xml`)   |

## Message

`iceoryx2_interoperation_demo_msgs/msg/TransmissionData`: `int32 x`, `int32 y`,
`float64 funky`. Fixed-size POD (no strings/sequences).

## Binaries

| Binary            | Package                                  | Role                                     |
| ----------------- | ---------------------------------------- | ---------------------------------------- |
| `ros2_publisher`  | `rmw_iceoryx2_interoperation_demo_nodes` | ROS 2 publisher on `/transmission_data`  |
| `ros2_subscriber` | `rmw_iceoryx2_interoperation_demo_nodes` | ROS 2 subscriber on `/transmission_data` |
| `publisher`       | `iceoryx2_interoperation_demo_nodes`     | iceoryx2 publisher                       |
| `subscriber`      | `iceoryx2_interoperation_demo_nodes`     | iceoryx2 subscriber                      |

## Wire contract

The ROS topic maps to the iceoryx2 service the vanilla app uses:

- Service name: `ros2://topics/transmission_data` (ROS topic `/transmission_data`)
- Payload: slice of `u8` (`Slice<uint8_t>` ⇄ `[u8]`), no user header
- Bytes: the `#[repr(C)]` `TransmissionData` struct, unserialized
- The iceoryx2 publisher signals the topic's event service on every send (the rmw
  subscriber wakes on notification, not by polling)
- Service static config and QoS attributes mirror rclcpp's **default** profile
  (`KeepLast` depth 10, reliable). Using a non-default ROS QoS requires updating
  `open_or_create_service` in `iceoryx2_interoperation_demo_nodes`

## Prerequisites

- Workspace built (ROS 2 from source, `rmw_iceoryx2_cxx`, `iceoryx2`).
- Network access on the first Rust build (fetches `rosidl_runtime_rs` from crates.io).
- All processes run on the same host with the same iceoryx2 configuration.

## Build

```sh
# workspace root
colcon build --packages-up-to rmw_iceoryx2_interoperation_demo_nodes
source install/setup.bash

# vanilla iceoryx2 Rust app — the environment MUST be sourced
cargo build --release --manifest-path \
  src/rmw_iceoryx2/examples/iceoryx2_interoperation/iceoryx2_interoperation_demo_nodes/Cargo.toml
```

## Run

One process per terminal. In every terminal:

```sh
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
```

ROS 2 nodes:

```sh
ros2 run rmw_iceoryx2_interoperation_demo_nodes ros2_publisher
ros2 run rmw_iceoryx2_interoperation_demo_nodes ros2_subscriber
```

Vanilla iceoryx2 app:

```sh
IOX2_NODES=src/rmw_iceoryx2/examples/iceoryx2_interoperation/iceoryx2_interoperation_demo_nodes/target/release
$IOX2_NODES/publisher
$IOX2_NODES/subscriber
```

Run any one publisher with any one subscriber:

| Publisher              | Subscriber                |
| ---------------------- | ------------------------- |
| `ros2_publisher`       | `subscriber` (iceoryx2)   |
| `publisher` (iceoryx2) | `ros2_subscriber`         |
| `ros2_publisher`       | `ros2_subscriber`         |
| `publisher` (iceoryx2) | `subscriber` (iceoryx2)   |

