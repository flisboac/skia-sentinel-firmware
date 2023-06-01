#!/bin/sh
set -e
export DEBUG
PROJECT_ROOT="$(cd "$(dirname "$0")"; pwd)"
source "${PROJECT_ROOT}/.env.sh"
if [ ! -e "${PROJECT_ROOT}/app/build" ]; then
  "${PROJECT_ROOT}/dev-build.sh"
fi
cd "${PROJECT_ROOT}"
west -v flash \
  -d app/build \
  --esp-device="${ESPTOOL_PORT:-"/dev/ttyACM0"}" \
  --esp-baud-rate="${ESPTOOL_BAUD:-"460800"}" \
  --esp-flash-freq="${ESPTOOL_FF:-"80m"}"
