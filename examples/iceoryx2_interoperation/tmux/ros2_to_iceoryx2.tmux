#!/usr/bin/env bash
# ROS 2 publisher -> native iceoryx2 subscriber.
set -eo pipefail

WS="${WORKSPACE_ROOT:?run via: just -f src/rmw_iceoryx2/justfile run-example}"
EXAMPLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUBSCRIBER="$EXAMPLE_DIR/iceoryx2_interoperation_demo_nodes/target/release/subscriber"
SESSION_NAME="$(basename "${BASH_SOURCE[0]}" .tmux)"

# Dedicated socket
tmux() { command tmux -L rmw_iceoryx2 "$@"; }

if [[ ! -x "$SUBSCRIBER" ]]; then
    echo "missing native iceoryx2 binary: $SUBSCRIBER" >&2
    echo "build it with: source $WS/install/setup.bash && cargo build --release --manifest-path $EXAMPLE_DIR/iceoryx2_interoperation_demo_nodes/Cargo.toml" >&2
    exit 1
fi

prelude="source '$WS/install/setup.bash'; export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx; export ROS_DISABLE_LOANED_MESSAGES=0"

# Run bash in each pane regardless of the user's $SHELL.
bash_bin="$(command -v bash)"
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
tmux new-session -d -s "$SESSION_NAME" -n "$SESSION_NAME" "$bash_bin"
tmux send-keys -t "$SESSION_NAME" "$prelude; '$SUBSCRIBER'" C-m
tmux split-window -h -b -t "$SESSION_NAME" "$bash_bin"
tmux send-keys -t "$SESSION_NAME" "$prelude; ros2 run rmw_iceoryx2_interoperation_demo_nodes ros2_publisher" C-m
tmux select-layout -t "$SESSION_NAME" even-horizontal
tmux attach-session -t "$SESSION_NAME"
