# rmw_iceoryx2

1. [Introduction](#introduction)
1. [Feature Completeness](#feature-completeness)
1. [Performance](#performance)
1. [Setup](#setup)
1. [FAQ](#faq)
1. [Commercial Support](#commercial-support)
1. [Maintainers](#maintainers)
1. [Contributors](#contributors)

## Introduction

> [!IMPORTANT]
> The implementation is still in an "alpha" stage.
> Not all functionality is implemented/stable so surprises are to be expected.
>
> If encountering problems, please create an issue so we can converge to
> stability :).

> [!IMPORTANT]
> Currently only supports self-contained messages as message serialization is
> not yet implemented.

ROS 2 [`rmw`](https://github.com/ros2/rmw) implementation for [`iceoryx2`](https://github.com/eclipse-iceoryx/iceoryx2).

`iceoryx2` is a shared memory IPC middleware written in Rust for improved memory
safety and easier safety certifiability. The implementation leverages the C++
bindings to the Rust core.

## Feature Completeness

| Feature                          | Status             |
|----------------------------------|--------------------|
| Node                             | :white_check_mark: |
| Guard Condition                  | :white_check_mark: |
| Event                            | :construction:     |
| Publish-Subscribe (Copy)         | :white_check_mark: |
| Publish-Subscribe (Loan)         | :white_check_mark: |
| Publish-Subscribe (Serialized)   | :construction:     |
| Server-Client                    | :construction:     |
| Waitset                          | :white_check_mark: |
| Graph                            | :construction:     |
| QoS                              | :construction:     |
| Logging                          | :white_check_mark: |

## Setup

1. Set up [your environment](https://docs.ros.org/en/rolling/Installation/Alternatives/Latest-Development-Setup.html) for building ROS 2 from source
1. Create a ROS 2 workspace:

   ```console
   mkdir -p ~/workspace/src && cd ~/workspace
   ```

1. Clone the ROS 2 source:

    ```console
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/rolling/ros2.repos src
    ```

1. Check out the latest version of `iceoryx`:

    ```console
    cd src/eclipse-iceoryx/iceoryx/ && git checkout main && cd -
    ```

1. Clone `iceoryx2`:

    ```console
    git clone git@github.com:eclipse-iceoryx/iceoryx2.git src/iceoryx2
    ```

1. Clone `rmw_iceoryx2`:

    ```console
    git clone git@github.com:ekxide/rmw_iceoryx2.git src/rmw_iceoryx2
    ```

1. Build ROS 2 with `rmw_iceoryx2` and the demo nodes:

    ```console
    RMW_IMPLEMENTATION=rmw_iceoryx2_cxx colcon build --symlink-install --packages-up-to ros2cli_common_extensions rmw_iceoryx2_cxx rmw_iceoryx2_cxx_demo_nodes
    ```

1. Verify the build:

    ```console
    source ~/workspace/install/setup.zsh # or setup.bash
    RMW_IMPLEMENTATION=rmw_iceoryx2_cxx ros2 doctor --report
    ```

    The middleware should be properly set:

    ```
    RMW MIDDLEWARE
      middleware name    : rmw_iceoryx2_cxx
    ```

1. Verify functionality by running the demo nodes:
    1. Terminal 1

        ```console
        source ~/workspace/install/setup.zsh # or setup.bash
        ROS_DISABLE_LOANED_MESSAGES=0 ros2 run rmw_iceoryx2_cxx_demo_nodes listener
        ```

    1. Terminal 2

        ```console
        source ~/workspace/install/setup.zsh # or setup.bash
        ROS_DISABLE_LOANED_MESSAGES=0 ros2 run rmw_iceoryx2_cxx_demo_nodes talker
        ```

## FAQ

### Why another RMW implementation?

The goals of `rmw_iceoryx2` are to:

1. Enable ROS 2 applications to leverage the high performance shared-memory communication offered by `iceoryx2`
1. Enable interoperability between ROS 2 and `iceoryx2` applications

### Is `rmw_iceoryx2` targeting ASIL certification?

ASIL certification of `rmw_iceoryx2` is currently not a priority. Reason being that even if it were to be certified,
a certified flavour of ROS 2 would additionally be required to produce a fully certified application, which is
a large undertaking and not in scope for us.

The `iceoryx2` implementation, however, is prepared for and targeting ASIL-D certification. With interoperability between `iceoryx2` 
and `rmw_iceoryx2`, safety-critical components built on `iceoryx2` (which may be certified up to ASIL-D) can communicate 
with ROS 2 components which may not be certified, or certified at a lower rating (e.g. Quality Management (QM)), thus taking
advantage of the vast development ecosystem offered by ROS 2.

If you have a need for some level of certification for `rmw_iceoryx2`, feel free to get in touch.
We would be happy to discuss your use-case and explore the options together.

### Is `rmw_iceoryx2` capable of host-to-host communication?

In its current form, `rmw_iceoryx2` only supports communication within a single host. However, `iceoryx2` has so-called `Gateways`
and `Tunnels` on the roadmap which will support this use-case and should be available in the coming months.

A `Gateway` bridges between hosts using a host-to-host-capable middleware with a defined on-wire protocol, such as 
[`zenoh`](https://github.com/eclipse-zenoh/zenoh), which runs in an isolated process and exchanges payloads via `iceoryx2` 
shared-memory communication. This keeps network communication isolated from safety-critical software.

A `Tunnel` provides a more direct approach to host-to-host communication while maintaining the same process isolation model.
However, instead of using an intermediary middleware like  `zenoh`, a `Tunnel` writes message payloads directly to the transport layer.
For example, a `Tunnel` using [`smoltcp`](https://github.com/smoltcp-rs/smoltcp) would handle TCP/IP communication directly,
offering lower latency but requiring more careful handling of network communication details.

### What is a self-contained message?

A message definition that does not contain any pointers or references to addresses in a process's virtual 
address space i.e. satisfy [`TriviallyCopyable` named requirement](https://en.cppreference.com/w/cpp/named_req/TriviallyCopyable).

Self-contained messages can be stored in shared memory without any serialization and subsequently read by
any other process on the host system. Binaries should be compiled with the same compiler flags to ensure
consistent memory representation.

### How can I verify that `iceoryx2` is being used by my ROS 2 application?

The [`iox2`](https://github.com/eclipse-iceoryx/iceoryx2/tree/main/iceoryx2-cli) CLI can be used to verify services are created for ROS 2 endpoints:

```console
$ iox2 service list
[
    PublishSubscribe("ros2://topics/basic_types"),
    Event("ros2://topics/basic_types"),
    PublishSubscribe("ros2://topics/parameter_events"),
    Event("ros2://topics/parameter_events"),
]
```

## Commercial Support

<!-- markdownlint-disable -->

<table width="100%">
  <tbody>
    <tr>
      <td align="center" valign="top" width="33%">
        <a href="https://ekxide.io">
        <img src="https://github.com/eclipse-iceoryx/iceoryx2/assets/56729169/c3ce8370-6cef-4c31-8259-93ddaa61c43e" alt="ekxide IO GmbH"/><br />
        </a>
        <a href="mailto:info@ekxide.io">info@ekxide.io</a>
      </td>
      <td>
        <ul>
          <li>accelerated development</li>
          <li>safety certification</li>
          <li>priority bug-fixing</li>
          <li>training and consulting</li>
        </ul>
      </td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-enable -->

## Maintainers

<!-- markdownlint-disable -->

<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%">
          <a href="https://github.com/orecham">
          <img src="https://avatars.githubusercontent.com/u/8487595" width="120px;" alt="»orecham«"/><br />
          <sub><b>»orecham«</b></sub></a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-enable -->

## Contributors

It could be you!

This project is and will always remain fully open source. Looking to use
`iceoryx2` in your ROS 2 application but finding the implementation lacking
in some way? Your contributions can help improve it more quickly, and we'll
provide full support and guidance along the way.
