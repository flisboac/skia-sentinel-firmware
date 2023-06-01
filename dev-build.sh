#!/bin/sh
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")"; pwd)"
export DEBUG
source "${PROJECT_ROOT}/.env.sh"
cd "${PROJECT_ROOT}/app"
if [ ! -z "${OPENOCD}" ]; then
  west build -- \
    -DOPENOCD="${OPENOCD}" \
    -DOPENOCD_DEFAULT_PATH="${OPENOCD_DEFAULT_PATH}" \
    -DZEPHYR_EXTRA_MODULES="${PROJECT_ROOT}"
else
  west build -- \
    -DZEPHYR_EXTRA_MODULES="${PROJECT_ROOT}"
fi
