# rmw_iceoryx2

> [!IMPORTANT]
> The implementation is still in an "alpha" stage.
> Not all functionality is implemented/stable so suprises are to be expected.
>
> If encountering problems, please create an issue so we can converge to
> stability :).

> [!IMPORTANT]
> Currently only supports self-contained messages as message serialiation is
> not yet implemented.

ROS2 RMW implementation for [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2).

`iceoryx2` is a shared memory IPC middleware written in Rust for improved memory
safety and easier safety certifiability. The RMW implementation leverages the C++
bindings to the Rust core.

## Feature Completeness

| Feature                          | Status             |
|----------------------------------|--------------------|
| Node                             | :white_check_mark: |
| Guard Condition                  | :white_check_mark: |
| Event                            | :construction:     |
| Publish-Subscribe (Intra)        | :construction:     |
| Publish-Subscribe (Copy)         | :white_check_mark: |
| Publish-Subscribe (Loan)         | :white_check_mark: |
| Publish-Subscribe (Serialized)   | :construction:     |
| Server-Client                    | :construction:     |
| Waitset                          | :white_check_mark: |
| Graph                            | :construction:     |
| QoS                              | :construction:     |
| Logging                          | :white_check_mark: |

## Setup

1. Set up [your environment](https://docs.ros.org/en/rolling/Installation/Alternatives/Latest-Development-Setup.html) for building ROS2 from source
1. Create a ROS2 workspace:

   ```console
   mkdir -p ~/workspace/src && cd ~/workspace
   ```

1. Clone the ROS2 source:

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

1. Build ROS2 with `rmw_iceoryx2` and the demo nodes:

    ```console
    RMW_IMPLEMENTATION=rmw_iceoryx2_cxx colcon build --symlink-install --packages-up-to ros2cli_common_extensions rmw_iceoryx2_cxx rmw_iceoryx2_cxx_demo_nodes
    ```

1. Verify the build:

    ```console
    source ~/workspace/install/setup.zsh
    ros2 doctor --report
    ```

    The middleware should be properly set:

    ```
    RMW MIDDLEWARE
      middleware name    : rmw_iceoryx2_cxx
    ```

1. Verify functionality by running the demo nodes:
    1. Terminal 1

        ```console
        source ~/workspace/install/setup.zsh
        ROS_DISABLE_LOANED_MESSAGES=0 ros2 run rmw_iceoryx2_cxx_demo_nodes listener
        ```

    1. Terminal 2

        ```console
        source ~/workspace/install/setup.zsh
        ROS_DISABLE_LOANED_MESSAGES=0 ros2 run rmw_iceoryx2_cxx_demo_nodes talker
        ```

## FAQ

### What is a self-contained message?

A message definition that does not contain any pointers or references to addresses in a process's virtual 
address space.

Self-contained messages can be stored in shared memory without any serialization and subsequently read by
any other process on the host system. Binaries should be compiled with the same compiler flags to ensure
consistent memory representation.

### How can I verify that iceoryx2 is being used by my ROS application?

The `iox2` CLI can be used to verify services are created for ROS endpoints:

```console
iox2 service list
```

### Does iceoryx2 include optimizations for intra-process endpoints?

While `iceoryx2` does include a mode for intra-process communication (passing pointers), `rmw_iceoryx2_cxx` 
currently uses the inter-process mode for all communication.

This may be revisited in the future.

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
`iceoryx2` in your ROS2 application but finding the implementation lacking
in some way? Your contributions can help improve it more quickly, and we'll
provide full support and guidance along the way.
