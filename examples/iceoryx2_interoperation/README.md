# iceoryx2 interoperation demo

> [!WARNING]
> Work in progress. This example only demonstrates that ROS 2 ↔ iceoryx2
> interoperation is feasible. The iceoryx2-side setup shown here (manually
> mirroring the rmw's QoS attributes, static config, and type identity) is
> deliberately low-level and will be made ergonomic before it is recommended for
> general use. As a ROS 2 user you do not need any of this — run the ROS nodes as
> usual; only the native iceoryx2 peer carries the boilerplate for now.

A ROS 2 application (running on `rmw_iceoryx2_cxx`) and a vanilla iceoryx2 Rust
application exchanging `TransmissionData` over shared memory.

## Topology

```text
   ROS 2 application                                           iceoryx2 application
   (rclcpp -> rmw_iceoryx2_cxx -> iceoryx2)                    (iceoryx2)

   ros2_publisher  --+                                         +--  publisher
   ros2_subscriber --+                                         +--  subscriber
                     |                                         |
                     v                                         v
   +----------------------------------------------------------------+
   |               iceoryx2 shared memory                           |
   |               service: ros2://topics/transmission_data         |
   +----------------------------------------------------------------+
```

## Packages

| Package                                      | Build  | Contents                                |
| -------------------------------------------- | ------ | --------------------------------------- |
| `rmw_iceoryx2_interoperation_demo_msgs`      | colcon | `TransmissionData` interface (shared)   |
| `rmw_iceoryx2_interoperation_demo_nodes`     | colcon | ROS 2 nodes (rclcpp)                    |
| `iceoryx2_interoperation_demo_nodes`         | colcon | native iceoryx2 apps (`ament_cargo`)    |

## Binaries

All operate on the iceoryx2 service name `ros2://topics/transmission_data` (ROS topic
`/transmission_data`): a publish-subscribe service for the payload and an event service
for send notifications.

| Binary            | Package                                  | iceoryx2 services                              |
| ----------------- | ---------------------------------------- | --------------------------------------------- |
| `ros2_publisher`  | `rmw_iceoryx2_interoperation_demo_nodes` | publish-subscribe publisher, event notifier   |
| `ros2_subscriber` | `rmw_iceoryx2_interoperation_demo_nodes` | publish-subscribe subscriber, event listener  |
| `publisher`       | `iceoryx2_interoperation_demo_nodes`     | publish-subscribe publisher, event notifier   |
| `subscriber`      | `iceoryx2_interoperation_demo_nodes`     | publish-subscribe subscriber, event listener  |

## Prerequisites

- Workspace built (ROS 2 from source, `rmw_iceoryx2_cxx`, `iceoryx2`).
- All processes run on the same host with the same iceoryx2 configuration.
- `just` and `tmux` for the scripted run below (the manual steps need neither).

## Build and run with `just`

The `justfile` at `src/rmw_iceoryx2/justfile` wraps the build and run steps. Run all
commands from the **workspace root**, passing the justfile with `-f`.

```sh
# list the available examples and their configurations
just -f src/rmw_iceoryx2/justfile list-examples

# build the ROS 2 packages and native iceoryx2 nodes for this example
just -f src/rmw_iceoryx2/justfile build-example iceoryx2_interoperation

# run a configuration (opens a tmux session: publisher left, subscriber right)
just -f src/rmw_iceoryx2/justfile run-example iceoryx2_interoperation ros2_to_iceoryx2
just -f src/rmw_iceoryx2/justfile run-example iceoryx2_interoperation iceoryx2_to_ros2
```

Two configurations are available, covering both cross-language directions
(`ros2_to_iceoryx2` = ROS 2 publisher → iceoryx2 subscriber, and the reverse). To mix
any other publisher/subscriber combination, use the manual steps below.

## Build and run manually

Build — from the workspace root:

```sh
# Disable colcon-cargo's Cargo-workspace discovery. It enumerates every member of
# the vendored iceoryx2 cargo workspace as a colcon package and folds their
# dev-dependencies into the build order, creating `*-tests-common` cycles that make
# `colcon build` fail to order packages topologically. 
# Fix in colcon-cargo required.
export COLCON_EXTENSION_BLOCKLIST="colcon_core.package_discovery.cargo_workspace:colcon_core.package_identification.cargo_workspace"

colcon build --packages-up-to \
  rmw_iceoryx2_interoperation_demo_nodes iceoryx2_interoperation_demo_nodes
source install/setup.bash
```

Run — one process per terminal. In every terminal:

```sh
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
```

ROS 2 nodes:

```sh
ros2 run rmw_iceoryx2_interoperation_demo_nodes ros2_publisher
ros2 run rmw_iceoryx2_interoperation_demo_nodes ros2_subscriber
```

Native iceoryx2 app:

```sh
ros2 run iceoryx2_interoperation_demo_nodes publisher
ros2 run iceoryx2_interoperation_demo_nodes subscriber
```

Run any one publisher with any one subscriber:

| Publisher              | Subscriber                |
| ---------------------- | ------------------------- |
| `ros2_publisher`       | `subscriber` (iceoryx2)   |
| `publisher` (iceoryx2) | `ros2_subscriber`         |
| `ros2_publisher`       | `ros2_subscriber`         |
| `publisher` (iceoryx2) | `subscriber` (iceoryx2)   |
