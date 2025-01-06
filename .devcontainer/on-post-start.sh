#!/bin/sh

set -e

WORKSPACE_ROOT="${WORKSPACE_ROOT:-"/workspace"}"
ZEPHYR_APP_DIRNAME="${ZEPHYR_APP_DIRNAME:-"./app"}"  # relative to $WORKSPACE_ROOT
WEST_EXTERNAL_BASE_DIRNAME="${WEST_EXTERNAL_BASE_DIRNAME:-"./ext"}"  # relative to $WORKSPACE_ROOT
SYSTEMD_UDEVD_CMD="${SYSTEMD_UDEVD_CMD:-"/lib/systemd/systemd-udevd"}"

printf  "INFO: Running script: $(basename "$0")\n" >&2

if [ ! -z "${SYSTEMD_UDEVD_CMD}" ]; then
  __UDEV_STAT="$(ps -Ao command)"
  if ! printf '%s' "${__UDEV_STAT}" | grep -F "${SYSTEMD_UDEVD_CMD}" >/dev/null 2>/dev/null; then
    printf 'INFO: Initializing udev daemon...\n' >&2
    sudo "${SYSTEMD_UDEVD_CMD}" --daemon
  fi
fi

if command -v v11vnc >/dev/null 2>/dev/null; then
  printf 'INFO: Initializing VNC server...\n' >&2
  v11vnc --create -forever
fi

printf  "INFO: Finished.\n" >&2
