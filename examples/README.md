# Examples

Each example is a self-contained subdirectory built and run via the root
`justfile` (`src/rmw_iceoryx2/justfile`). Commands run from the **workspace root**:

```sh
just -f src/rmw_iceoryx2/justfile list-examples              # examples + their configurations
just -f src/rmw_iceoryx2/justfile build-example <example>    # build the example's packages
just -f src/rmw_iceoryx2/justfile run-example <example> <config>
```

`build-example`/`run-example`/`list-examples` discover examples by directory — adding
one requires **no edits to the root justfile**.

## Layout

```
examples/
  <example>/
    justfile          # `build` recipe — builds everything the example needs
    tmux/
      <config>.tmux   # one runnable configuration (a tmux session)
    ...               # the example's packages / sources
```

- `<example>` is the directory name, passed to `build-example`/`run-example`.
- `<config>` is a tmux script's filename without `.tmux`, passed to `run-example`.
- An example is listed by `list-examples` only once it has a `tmux/` directory.

## Adding an example

1. Create `examples/<example>/` with its packages (colcon and/or cargo).
2. Add `examples/<example>/justfile` with a `build` recipe (below).
3. Add one or more `examples/<example>/tmux/<config>.tmux` scripts (below).

Use `examples/iceoryx2_interoperation/` as the reference.

## The example `justfile`

`build-example` runs `just -f examples/<example>/justfile build`, then records a build
stamp that `run-example` checks. The `build` recipe must build everything needed to run.

- Reuse the root `build` recipe for ROS 2 packages instead of re-invoking `colcon`:
  `root := justfile_directory() / "../../justfile"`, then `just -f "{{root}}" build <pkgs>`.
- Build in the workspace root: `ws := invocation_directory()`, `cd "{{ws}}"`.
- Use a `#!/usr/bin/env bash` recipe when steps must share one shell (e.g.
  `source install/setup.bash` before `cargo build`).

## tmux configurations

Each `tmux/<config>.tmux` is a bash script that opens a tmux session running the
processes for one scenario. `run-example` invokes it with `WORKSPACE_ROOT` exported.

Conventions (see the reference example):

- Use a dedicated socket (`tmux -L <name>`) so the host's tmux server is untouched.
- Force bash panes (host shell may be fish/zsh); send a `source install/setup.bash; …`
  prelude into each pane.
- Name the session/window after the config (`basename "${BASH_SOURCE[0]}" .tmux`).
- Verify required binaries exist; otherwise print the build command and exit non-zero.

## Multiple configurations

A configuration is one runnable scenario over the same example packages — e.g. a
different topology, direction, node set, or QoS. List them per example with
`list-examples`.

- One configuration: `run-example <example>` selects it automatically.
- Several: `run-example <example>` lists them and exits; pass `<config>` to choose one.

Example: `iceoryx2_interoperation` has `ros2_to_iceoryx2` and `iceoryx2_to_ros2` —
the same nodes, opposite publisher/subscriber direction.
