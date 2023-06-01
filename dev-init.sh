#!/bin/sh
set -e
export DEBUG
PROJECT_ROOT="$(cd "$(dirname "$0")"; pwd)"
source "${PROJECT_ROOT}/.env.sh"
cd "${PROJECT_ROOT}"
west -v init -l app
west update
west blobs fetch hal_espressif
