#!/usr/bin/env bash
# Run pytest in the given repository checkout (worktree) with the project's interpreter.
# Usage: run_tests.sh <checkout-root> <pytest args...>
# Check `free -g` first; the run is capped at 16 GiB virtual memory (8 GiB stops ROS
# from creating threads).
set -e
ROOT="$1"; shift
source /home/nasim/ros_ws/install/setup.bash
cd "$ROOT"
SRC=""
for d in "$ROOT"/*/src; do SRC="$SRC:$d"; done
export PYTHONPATH="$ROOT${SRC}:${PYTHONPATH}"
export CRAM_ORM_BUILD=never OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen
ulimit -v 16777216
exec /home/nasim/.virtualenvs/cram-env/bin/python -m pytest "$@"
