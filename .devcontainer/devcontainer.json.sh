#!/bin/sh

set -e
OWN_DIR="$(cd "$(dirname "$0")"; pwd)"
[ ! -e "${OWN_DIR}/devcontainer.env" ] && "${OWN_DIR}/devcontainer.env.sh"
. "${OWN_DIR}/devcontainer.env"
cat >"${OWN_DIR}/devcontainer.json" <<ENVSUBST_EOF
{
  "name": "skia-sentinel-firmware",
  "dockerFile": "Dockerfile",

  "build": {
    "args": {
      "HOST_IP": "${HOST_IP}",
      "USERNAME": "${USERNAME}",
      "USER_UID": "${USER_UID}",
      "USER_GID": "${USER_GID}",
      "TTY_GID": "${TTY_GID}",
      "DIALOUT_GID": "${DIALOUT_GID}",
      "INPUT_GID": "${INPUT_GID}",
      "USERS_GID": "${USERS_GID}",
      "JLINK_GID": "${JLINK_GID}",
      "JLINK_GNAME": "${JLINK_GNAME}",
      "PLUGDEV_GID": "${PLUGDEV_GID}",
      "PLUGDEV_GNAME": "${PLUGDEV_GNAME}"
    }
  },

  "remoteEnv": {
    "DISPLAY": "${DISPLAY}"
  },

  // Set *default* container specific settings.json values on container create.
  "customizations": {
    "vscode": {
      "settings": {
        "terminal.integrated.shell.linux": "/bin/bash"
      },
      "extensions": [
        "ms-vscode.cpptools",
        "platformio.platformio-ide",
        "marus25.cortex-debug",
        "mcu-debug.debug-tracker-vscode",
        "ms-vscode.vscode-serial-monitor",
        "mylonics.zephyr-ide",
        // "ms-vscode.cmake-tools",
        "EditorConfig.EditorConfig",
        "ms-azuretools.vscode-docker",
        "ms-python.python",
        "ms-vscode.vscode-embedded-tools",
        "mhutchie.git-graph",
        "eamodio.gitlens",
        "trond-snekvik.gnu-mapfiles",
        "wayou.vscode-todo-highlight",
        "stkb.rewrap",
        "CS128.cs128-clang-tidy",
        "xaver.clang-format",
        "ms-vscode.cpptools-themes",
        "nordic-semiconductor.nrf-kconfig",
        "nordic-semiconductor.nrf-connect",
        "nordic-semiconductor.nrf-devicetree",
        "nordic-semiconductor.nrf-connect-extension-pack"
      ]
    }
  },

  "workspaceMount": "source=\${localWorkspaceFolder},target=/workspace,type=bind,consistency=consistent",
  "workspaceFolder": "/workspace",

  "forwardPorts": [
    8008
  ],

  "runArgs": [
		"--env-file=\${localWorkspaceFolder}/.devcontainer/devcontainer.env",

    // For access to X socket
    "--security-opt",
    "label=disable",

    // --privileged grants access to /dev on host. Arduino will most likely be /dev/ttysN
    "--privileged"

    // NOTE: Valid only for Podman, AND when not mapping IDs with --uidmap and --gidmap
    // "--userns=keep-id:uid=${USER_UID},gid=${USER_GID}",
    // "--annotation=run.oci.keep_original_groups=1",
    // "--group-add=keep-groups",

    // NOTE: Valid only for Podman, AND when not using --userns
    // "--group-add=keep-groups",
    // "--uidmap=+g${USER_UID}:@${USER_UID}",
    // "--gidmap=+g${USER_GID}:@${USER_GID}",
    // "--gidmap=+g${TTY_GID}:@${TTY_GID}",
    // "--gidmap=+g${DIALOUT_GID}:@${DIALOUT_GID}"
  ],

	"postStartCommand": "bash /workspace/.devcontainer/on-post-start.sh",
  "postCreateCommand": "bash /workspace/.devcontainer/on-post-create.sh",

  "remoteUser": "${USERNAME}",

  "mounts": [
    "source=/dev,target=/dev,type=bind,consistency=consistent",
    "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=consistent"
  ]
}
ENVSUBST_EOF
