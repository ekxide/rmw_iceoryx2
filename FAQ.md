# Frequently Asked Questions

## Why another RMW implementation?

The goals of `rmw_iceoryx2` are to:

1. Enable ROS 2 applications to leverage the high performance shared-memory
   communication offered by `iceoryx2`
1. Enable interoperability between ROS 2 and `iceoryx2` applications

## Is `rmw_iceoryx2` targeting ASIL certification?

ASIL certification of `rmw_iceoryx2` is currently not a priority. Reason being
that even if it were to be certified, a certified flavour of ROS 2 would
additionally be required to produce a fully certified application, which is a
large undertaking and not in scope for us.

The `iceoryx2` implementation, however, is prepared for and targeting ASIL-D
certification. With interoperability between `iceoryx2` and `rmw_iceoryx2`,
safety-critical components built on `iceoryx2` (which may be certified up to
ASIL-D) can communicate with ROS 2 components which may not be certified, or
certified at a lower rating (e.g. Quality Management (QM)), thus taking
advantage of the vast development ecosystem offered by ROS 2.

If you have a need for some level of certification for `rmw_iceoryx2`, feel free
to get in touch. We would be happy to discuss your use-case and explore the
options together.

## Is `rmw_iceoryx2` capable of host-to-host communication?

`rmw_iceoryx2` communicates over shared memory, which is limited to a single
host. Communication across hosts is possible with the
[`iceoryx2-link`](https://github.com/eclipse-iceoryx/iceoryx2/tree/main/iceoryx2-link),
which extends `iceoryx2` services across the boundary of a shared-memory domain.

A
[tunnel](https://ekxide.github.io/iceoryx2-book/main/fundamentals/links.html#tunnels)
is one flavor of the `iceoryx2-link` that connects `iceoryx2` systems on
different hosts to each other. With a tunnel running on each host, ROS 2 topics
are made available across all of them.

A tunnel runs in its own process, while ROS 2 applications keep communicating
over shared memory. The applications therefore do not link any network code, and
a fault in the network stack stays contained in the tunnel process.

Tunnels are installed as extensions of the `iox2` CLI. The available
implementations and how to install them are listed in the
[`iceoryx2-cli` documentation](https://github.com/eclipse-iceoryx/iceoryx2/tree/main/iceoryx2-cli#implementations).

> [!IMPORTANT]
> The link is currently a prototype and only recommended for experimentation in
> development deployments.

## What is a self-contained message?

A message definition that does not contain any pointers or references to
addresses in a process's virtual address space i.e. satisfy
[`TriviallyCopyable` named requirement](https://en.cppreference.com/w/cpp/named_req/TriviallyCopyable).

Self-contained messages can be stored in shared memory without any serialization
and subsequently read by any other process on the host system. Binaries should
be compiled with the same compiler flags to ensure consistent memory
representation.

## How can I verify that `iceoryx2` is being used by my ROS 2 application?

The [`iox2`](https://github.com/eclipse-iceoryx/iceoryx2/tree/main/iceoryx2-cli)
CLI can be used to verify services are created for ROS 2 endpoints:

```console
$ iox2 service list
[
    PublishSubscribe("ros2://topics/basic_types"),
    Event("ros2://topics/basic_types"),
    PublishSubscribe("ros2://topics/parameter_events"),
    Event("ros2://topics/parameter_events"),
]
```
