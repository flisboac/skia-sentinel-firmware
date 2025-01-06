#!/bin/sh
set -e
export DEBUG
PROJECT_ROOT="$(cd "$(dirname "$0")/.."; pwd)"
. "${PROJECT_ROOT}/.env.sh"
cd "${PROJECT_ROOT}"
if [ ! -e .west/config ]; then
  west -v init -l app
fi
west update
# TODO: Add `if` to restrict blob installation depending on env-vars
python3 -m pip install -r "${WEST_EXTERNAL_BASE_DIR}/zephyr/scripts/requirements.txt"
if [ ! -e ~/.cmake/packages/Zephyr ]; then
  west zephyr-export
fi

# NOTE: NOT WORKING.
#west espressif update

west blobs fetch hal_espressif
