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
  --esp-device="${ESPTOOL_PORT:?Missing ESPTOOL_PORT}" \
  --esp-baud-rate="${ESPTOOL_BAUD:?Missing ESPTOOL_BAUD}" \
  --esp-flash-freq="${ESPTOOL_FF:?Missing ESPTOOL_FF}"
