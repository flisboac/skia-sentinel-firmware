#!/bin/sh

set -e

printf  "INFO: Running script: $(basename "$0")\n" >&2
printf  "INFO: Running as user \"$(id -un)\", ID: $(id -u)\n" >&2

if ! nrfutil list | grep device >/dev/null 2>/dev/null; then
  printf 'INFO: Installing nrfutil device command...\n' >&2
  nrfutil install device
fi

if ! nrfutil list | grep toolchain-manager >/dev/null 2>/dev/null; then
  printf 'INFO: Installing nrfutil toolchain-manager command...\n' >&2
  nrfutil install toolchain-manager
fi

printf  "INFO: Finished.\n" >&2
