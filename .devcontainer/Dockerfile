# https://github.com/zephyrproject-rtos/docker-image/tree/main

FROM ghcr.io/zephyrproject-rtos/ci-base:main

# Common args
ARG \
  USERNAME \
  USER_UID \
  USER_GID \
  TTY_GID \
  DIALOUT_GID \
  INPUT_GID \
  USERS_GID \
  JLINK_GID \
  JLINK_GNAME \
  PLUGDEV_GID \
  PLUGDEV_GNAME \
  HOSTTYPE=""

# Args from devcontainer.json (local build) or GitHub workflow (CI) (or default)
ARG \
  ZSDK_VERSION="0.16.8" \
  ZEPHYR_SDK_BASE_DIR="/opt" \
  NRF_UDEV_VERSION="1.0.1" \
  DEFAULT_HOST_ARCH="x86_64" \
  OPENOCD_ESP32_VERSION="0.12.0-esp32-20241016" \
  SEGGER_JLINK_VERSION="V810" \
  SVD_DATA_DIR="data/STMicro data/Espressif data/Nordic data/RaspberryPi" \
  WORKSPACE_ROOT="/workspace" \
  ZEPHYR_SDK_TOOLCHAIN="arm-zephyr-eabi \
    aarch64-zephyr-elf \
    riscv64-zephyr-elf \
    x86_64-zephyr-elf \
    xtensa-espressif_esp32s3_zephyr-elf \
    xtensa-espressif_esp32_zephyr-elf"

ENV \
  PKG_CONFIG_PATH="/usr/lib/i386-linux-gnu/pkgconfig" \
  OVMF_FD_PATH="/usr/share/ovmf/OVMF.fd" \
  LC_ALL="C.UTF-8" \
  LANG="C.UTF-8" \
  HOSTTYPE="${HOSTTYPE:-${DEFAULT_HOST_ARCH}}" \
  ZEPHYR_SDK_VERSION="${ZSDK_VERSION}" \
  ZEPHYR_SDK="${ZEPHYR_SDK_BASE_DIR}/zephyr-sdk-${ZSDK_VERSION}"

# Install udev and git
RUN apt-get -y update \
  && apt-get -y install \
    # Common stuff
    apt-utils \
    udev \
    ca-certificates \
    apt-transport-https \
    software-properties-common \
    curl \
    dialog \
    git \
    sudo \
    unzip \
    wget \
    file \
    vim \
    srecord \
    xz-utils \
    iputils-ping \
    usbutils \
    # C/C++ Builds
    build-essential \
    make \
    gcc \
    gcc-multilib \
    g++-multilib \
    libsdl2-dev \
    libmagic1 \
    ccache \
    gperf \
    cmake \
    ninja-build \
    device-tree-compiler \
    # C/C++ Tools
    clang-tidy \
    clang-format \
    # Python3
    python3 \
    python3-dev \
    python3-venv \
    python3-distutils \
    python3-setuptools \
    python3-tk \
    python3-wheel \
    python3-pip \
    pipx \
    # General device/firmware tools
    dfu-util \
    linux-tools-common \
    # Debugging tools: stlink
    # https://manpages.ubuntu.com/manpages/focal/en/man1/st-util.1.html
    stlink-tools \
    # Debugging tools: JLink
    # https://github.com/ScoopInstaller/Scoop/issues/4336#issue-864466149
    libxkbcommon-x11-0 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-icccm4 \
    libxcb-shape0 \
    libxcb-render-util0 \
    libjaylink-dev \
    libusb-1.0.0 \
    libfuse2 \
    # X11
    # TODO: Entrypoint for configuring X11 dynamically
    x11-xserver-utils \
    python3-xdg \
    libpython3-dev \
    x11vnc \
    xvfb \
    xterm

# Create non-root user (if not running as root by default)
RUN true \
  && printf 'INFO: Recreating default (base image) user...\n' \
  && userdel user \
  && { groupdel user || true; } \
  && { rm -f /etc/sudoers.d/user || true; } \
  && printf "INFO: Creating non-root user group \"$USERNAME\" (ID: \"${USER_GID}\")...\n" \
  && groupadd --gid "$USER_GID" "$USERNAME" \
  && printf 'INFO: Creating non-root user \"$USERNAME\" (ID: \"${USER_UID}\")...\n' \
  && useradd -s /bin/bash --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME" \
  && printf 'INFO: Configuring sudo for non-root user...\n' \
  && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
  && chmod 0440 "/etc/sudoers.d/$USERNAME" \
  && printf 'INFO: Configuring group "tty"...\n' \
  && groupmod --gid "$TTY_GID" tty \
  && printf 'INFO: Configuring group "dialout"...\n' \
  && groupmod --gid "$DIALOUT_GID" dialout \
  && printf 'INFO: Configuring group "input"...\n' \
  && groupmod --gid "$INPUT_GID" input \
  && printf 'INFO: Configuring group "users"...\n' \
  && groupmod --gid "$USERS_GID" users \
  && printf 'INFO: Adding user to "dialout" group...\n' \
  && usermod -aG dialout "$USERNAME" \
  && printf 'INFO: Adding user to "tty" group...\n' \
  && usermod -aG tty "$USERNAME" \
  && printf 'INFO: Adding user to "input" group...\n' \
  && usermod -aG input "$USERNAME" \
  && printf 'INFO: Adding user to "users" group...\n' \
  && usermod -aG users "$USERNAME" \
  && { if [ ! -z "${JLINK_GID}" ] && [ ! -z "${JLINK_GNAME}" ]; then true \
    && printf 'INFO: Configuring group "${JLINK_GNAME}"...\n' \
    && groupadd --gid "${JLINK_GID}" "${JLINK_GNAME}" \
    && printf 'INFO: Adding user to "${JLINK_GNAME}" group...\n' \
    && usermod -aG "${JLINK_GNAME}" "$USERNAME" \
    ; else true; fi; \
  } \
  && { if [ ! -z "${PLUGDEV_GID}" ]; then true \
    && printf 'INFO: Configuring group "plugdev"...\n' \
    && groupmod --gid "${PLUGDEV_GID}" plugdev \
    && printf 'INFO: Adding user to "plugdev" group...\n' \
    && usermod -aG plugdev "$USERNAME" \
    ; else true; fi; \
  }

# Hack udev to run inside a container
# https://stackoverflow.com/questions/62060604/why-udev-init-script-default-disable-container-support-while-in-fact-it-works
RUN sed -i.bak -e '/if \[ ! -w \/sys \]/,+3 s/^/#/' /etc/init.d/udev

RUN true \
    && printf 'INFO: Downloading Segger...\n' \
    && wget -q \
      --post-data "accept_license_agreement=accepted" \
      https://www.segger.com/downloads/jlink/JLink_Linux_${SEGGER_JLINK_VERSION}_x86_64.deb \
      -O JLink_Linux_x86_64.deb \
    && printf 'INFO: Installing Segger...\n' \
    && dpkg -i ./JLink_Linux_x86_64.deb \
    && rm ./JLink_Linux_x86_64.deb

# needed for Nordic boards
RUN true \
    && printf "INFO: Downloading nrf-udev (Version: v${NRF_UDEV_VERSION})...\n" \
    && wget -q \
      "https://github.com/NordicSemiconductor/nrf-udev/releases/download/v${NRF_UDEV_VERSION}/nrf-udev_${NRF_UDEV_VERSION}-all.deb" \
      -O ./nrf-udev.deb \
    && printf 'INFO: Installing nrf-udev...\n' \
    && dpkg -i ./nrf-udev.deb \
    && rm ./nrf-udev.deb

RUN true \
    && printf 'INFO: Downloading nrf-tools...\n' \
    && wget -q \
      "https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-24-2/nrf-command-line-tools_10.24.2_amd64.deb" \
      -O ./nrf-tools.deb \
    && printf 'INFO: Installing nrf-tools...\n' \
    && dpkg -i ./nrf-tools.deb \
    && rm ./nrf-tools.deb

RUN true \
    && printf 'INFO: Downloading nrf-util...\n' \
    && wget -q \
      "https://files.nordicsemi.com/ui/api/v1/download?repoKey=swtools&path=external/nrfutil/executables/x86_64-unknown-linux-gnu/nrfutil&isNativeBrowsing=false" \
      -O /usr/local/bin/nrfutil \
    && printf 'INFO: Installing nrf-util...\n' \
    && chmod a+x /usr/local/bin/nrfutil \
    && ln -s /usr/local/bin/nrfutil /usr/local/bin/nrf-util \
    && printf 'INFO: Applying suggested fixes for nrf-udev...\n' \
    && bash -c 'true # apt install -y /opt/nrf-command-line-tools/share/JLink_Linux*.deb --fix-broken --allow-downgrades'

# install minimal Zephyr SDK
WORKDIR "${ZEPHYR_SDK_BASE_DIR}"
RUN true \
  && printf "INFO: Downloading Zephyr SDK (URL: https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz)...\n" \
  && wget -q \
    "https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz" \
    -O "zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz" \
  && printf 'INFO: Extracting Zephyr SDK...\n' \
  && tar xf "zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz" \
  && printf 'INFO: Deleting downloaded Zephyr SDK package...\n' \
  && rm "zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz"

# install toolchain and host tools
# https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html
RUN if [ ! -z "${ZEPHYR_SDK_TOOLCHAIN}" ]; then \
    printf '%s' "${ZEPHYR_SDK_TOOLCHAIN}" | xargs -n 1 sh -c 'true \
      && printf "INFO: Initializing Zephyr SDK for toolchain \"${0}\"...\n" \
      && "${ZEPHYR_SDK}/setup.sh" -t "${0}" -h \
      && printf "INFO: Setting PATH env-var for toolchain \"${0}\"...\n" \
      && echo "PATH=\"${ZEPHYR_SDK}/${ZEPHYR_SDK_TOOLCHAIN}/bin:${PATH}\"" >>/etc/environment \
    ' \
  ; fi

# Install OpenOCD-ESP32
RUN true \
  && printf 'INFO: Downloading "openocd-esp32"...\n' \
  && wget -q \
    "https://github.com/espressif/openocd-esp32/releases/download/v${OPENOCD_ESP32_VERSION}/openocd-esp32-linux-amd64-${OPENOCD_ESP32_VERSION}.tar.gz" \
    -O openocd-esp32-linux-amd64.tar.gz \
  && printf 'INFO: Extracting "openocd-esp32"...\n' \
  && tar -C /opt -xzvf openocd-esp32-linux-amd64.tar.gz \
  && ls -laR /opt/openocd-esp32 \
  && printf 'INFO: Configuring "openocd-esp32"...\n' \
  && chmod a+x /opt/openocd-esp32/bin/openocd

# COPY ./guest/devcontainer/openocd-esp32/bin/openocd.sh /opt/openocd-esp32/bin
# RUN chmod a+x /opt/openocd-esp32/bin/openocd.sh

# Starting user-centric configuration.
#
COPY --chown=${USERNAME}:${USERNAME} ./.bashrc /home/${USERNAME}/

# Run the Zephyr SDK setup script as 'user' in order to ensure that the
# `Zephyr-sdk` CMake package is located in the package registry under the
# user's home directory.
USER "$USERNAME"

# Python utilities in general
RUN true \
  && printf "INFO: Configuring pipx...\n" \
  && pipx ensurepath \
  && printf "INFO: Configuring pipx globally...\n" \
  && { sudo pipx ensurepath --global || printf "ERROR: FAILED to configure pipx globally!\n" >&2; } \
  && printf "INFO: Python utilities: Installing 'ruff'...\n" \
  && { curl -LsSf https://astral.sh/ruff/install.sh | sh; } \
  && printf "INFO: Python utilities: Installing 'uv'...\n" \
  && { curl -LsSf https://astral.sh/uv/install.sh | sh; } \
  && printf "INFO: Python utilities: Installing 'poetry'...\n" \
  && ( bash -l -c 'pipx install poetry' )

RUN sudo -E -- \
  sh -c 'true \
    && cd "/home/$USERNAME" \
    && printf "INFO: Setting up Zephyr (script: ${ZEPHYR_SDK}/setup.sh)\n" \
    && "${ZEPHYR_SDK}/setup.sh" -c' \
  && printf "INFO: Installing Zephyr's default UDEV Rules...\n" \
  && sudo cp "${ZEPHYR_SDK}/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules" /lib/udev/rules.d/60-openocd.rules \
  && printf "INFO: Setting ownership of user's CMake dir to \"${USERNAME}:${USERNAME}\"\n" \
  && sudo chown -R "${USERNAME}:${USERNAME}" "/home/${USERNAME}/.cmake"

RUN true \
  && printf "INFO: Downloading cmsis-svd-data...\n" \
  && cd "/opt" \
  && sudo git clone -n --depth=1 --filter=tree:0 https://github.com/cmsis-svd/cmsis-svd-data.git cmsis-svd-data \
  && cd cmsis-svd-data \
  && sudo git sparse-checkout set --no-cone $SVD_DATA_DIR \
  && sudo git checkout \
  && sudo chmod a+rw -R /opt/cmsis-svd-data

RUN true \
  && printf "INFO: Cleaning up image...\n" \
  && sudo apt-get clean -y \
  && sudo apt-get autoremove --purge -y \
  && sudo rm -rf /var/lib/apt/lists/*

WORKDIR "${WORKSPACE_ROOT}"

ENV XDG_CACHE_HOME="/home/${USERNAME}/.cache" \
  WORKSPACE_ROOT="${WORKSPACE_ROOT}" \
  ZEPHYR_SDK_INSTALL_DIR="${ZEPHYR_SDK}" \
  ZEPHYR_TOOLCHAIN_VARIANT="zephyr" \
  ZEPHYR_BASE_DIRNAME="ext/zephyr" \
  WEST_EXTERNAL_BASE_DIRNAME="ext" \
  ZEPHYR_APP_DIRNAME="app"

ENV \
  WEST_EXTERNAL_BASE_DIR="${WORKSPACE_ROOT}/${WEST_EXTERNAL_BASE_DIRNAME}" \
  ZEPHYR_APP_DIR="${WORKSPACE_ROOT}/${ZEPHYR_APP_DIRNAME}" \
  ZEPHYR_BASE="${WORKSPACE_ROOT}/${ZEPHYR_BASE_DIRNAME}" \
  OPENOCD_ESP32="/opt/openocd-esp32/bin/openocd" \
  OPENOCD_ESP32_DEFAULT_PATH="/opt/openocd-esp32/share/openocd/scripts/"

ENV PATH="${ZEPHYR_BASE}/scripts:${ZEPHYR_SDK}/sysroots/x86_64-pokysdk-linux/usr/bin:${PATH}"
  # SVD_DIR="/home/${USERNAME}/cmsis-svd-data/${SVD_DATA_DIR}"

#USER root
