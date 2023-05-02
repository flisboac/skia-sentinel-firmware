#!/bin/bash
if [ "${CODESPACES}" = "true" ]; then
  export WORKSPACE_DIR="$HOME/workspace/skia"
fi
if [ -f "$WORKSPACE_DIR/zephyr/zephyr-env.sh" ]; then
  source "$WORKSPACE_DIR/zephyr/zephyr-env.sh"
fi
