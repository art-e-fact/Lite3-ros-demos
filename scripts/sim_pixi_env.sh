#!/bin/sh
# Single source of truth for which Newton pixi environment to run in, used by
# both the pixi sim-* tasks (via command substitution) and sim_control_harness.py.
set -eu

if [ -n "${SIM_PIXI_ENV:-}" ]; then
    echo "$SIM_PIXI_ENV"
elif [ "$(uname)" = "Linux" ] && nvidia-smi -L >/dev/null 2>&1; then
    echo "sim-gpu"
else
    echo "sim"
fi
