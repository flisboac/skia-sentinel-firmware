cmake_minimum_required(VERSION 3.20.0)

set(ZephyrBuildConfiguration_ROOT ${CMAKE_SOURCE_DIR}/cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
message("app: ZEPHYR_SKIA_FIRMWARE_CMAKE_DIR = ${ZEPHYR_SKIA_FIRMWARE_CMAKE_DIR}")

project(skia-sentinel-firmware VERSION 0.1.0)

target_sources(app PRIVATE src/main.c)
