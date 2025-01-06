#!/bin/sh

set -e
OWN_DIR="$(cd "$(dirname "$0")"; pwd)"
ENV_VAR_FILE="${OWN_DIR}/.devcontainer/devcontainer.env"
ENV_VAR_SH="${OWN_DIR}/.devcontainer/devcontainer.env.sh"
DEVCONTAINER_FILE="${OWN_DIR}/.devcontainer/devcontainer.json"
DEVCONTAINER_SH="${OWN_DIR}/.devcontainer/devcontainer.json.sh"
DOCKER_COMPOSE_FILE="${OWN_DIR}/.devcontainer/docker-compose.yml"
DOCKER_COMPOSE_SH="${OWN_DIR}/.devcontainer/docker-compose.yml.sh"
REGENERATE="${REGENERATE:-0}"

while [ "$#" -gt 0 ]; then
  case "$1" in
  -p) REGENERATE="1"; shift ;;
  --pristine) REGENERATE="1"; shift ;;
  --) shift; break ;;
  *) printf 'ERROR: Unknown option/flag: %s\n' "$1" >&2; exit 1 ;;
  esac
fi

if [ ! -e "${ENV_VAR_FILE}" ] || [ "${REGENERATE}" -eq 1 ]; then
  printf 'INFO: Initializing .devcontainer'\''s host env-vars file at "%s"...\n' "${ENV_VAR_FILE}" >&2
  sh "${ENV_VAR_SH}"
  printf 'INFO: .devcontainer'\''s host env-vars file generated. You can regenerate it by running \n\t%s\n' "${ENV_VAR_SH}" >&2
fi

if [ ! -e "${DEVCONTAINER_FILE}" ] || [ "${REGENERATE}" -eq 1 ]; then
  printf 'INFO: Initializing devcontainer.json file at "%s"...\n' "${DEVCONTAINER_FILE}" >&2
  sh "${DEVCONTAINER_SH}"
  printf 'INFO: devcontainer.json file generated. You can regenerate it by running \n\t%s\n' "${DEVCONTAINER_SH}" >&2
fi

if [ ! -e "${DOCKER_COMPOSE_FILE}" ] || [ "${REGENERATE}" -eq 1 ]; then
  printf 'INFO: Initializing docker-compose.yml file at "%s"...\n' "${DOCKER_COMPOSE_FILE}" >&2
  sh "${DOCKER_COMPOSE_SH}"
  printf 'INFO: docker-compose.yml file generated. You can regenerate it by running \n\t%s\n' "${DOCKER_COMPOSE_SH}" >&2
fi
