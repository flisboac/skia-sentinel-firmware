#!/bin/sh
: "${PROJECT_ROOT?Missing PROJECT_ROOT.}"
if [ ! -e "${PROJECT_ROOT}/.env" ]; then
  printf 'Copying sample env-var file: %s -> %s\n' "${PROJECT_ROOT}/.env.sample" "${PROJECT_ROOT}/.env" >&2
  cp "${PROJECT_ROOT}/.env.sample" "${PROJECT_ROOT}/.env"
fi
printf 'Sourcing env-var file: %s\n' "${PROJECT_ROOT}/.env" >&2
. "${PROJECT_ROOT}/.env"
for __VAR_NAME__ in $(grep -E '[_[:alnum:]]=.*' "${PROJECT_ROOT}/.env" | cut -d= -f1); do
  export "${__VAR_NAME__}"
done
unset __VAR_NAME__
