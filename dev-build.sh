#!/bin/sh
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")"; pwd)"
source "${PROJECT_ROOT}/.env.sh"
cd "${PROJECT_ROOT}/app"
# FIXME Find out why fixing BOARD_ROOT in the app/CMakeLists.txt doesn't work.
west build -- -DBOARD_ROOT="${PROJECT_ROOT}/app"
