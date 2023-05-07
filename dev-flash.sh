#!/bin/sh
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")"; pwd)"
source "${PROJECT_ROOT}/.env.sh"
if [ ! -e "${PROJECT_ROOT}/app/build" ]; then
  "${PROJECT_ROOT}/dev-build.sh"
fi
cd "${PROJECT_ROOT}"
west -v flash -d app/build
