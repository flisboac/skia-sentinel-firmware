cmake_minimum_required(VERSION 3.15)

get_filename_component(
  PROJECT_ROOT
  ${CMAKE_CURRENT_LIST_DIR}/../..
  ABSOLUTE
)

get_filename_component(
  APPLICATION_PROJECT_DIR
  ${CMAKE_CURRENT_LIST_DIR}/..
  ABSOLUTE
)

# Here is where the (West) magic happens!
list(APPEND ZEPHYR_EXTRA_MODULES ${PROJECT_ROOT})

message("ZephyrBuildConfig: ZEPHYR_BASE = ${ZEPHYR_BASE}")
message("ZephyrBuildConfig: PROJECT_ROOT = ${PROJECT_ROOT}")
message("ZephyrBuildConfig: APPLICATION_PROJECT_DIR = ${APPLICATION_PROJECT_DIR}")
message("ZephyrBuildConfig: BOARD_ROOT = ${BOARD_ROOT}")
message("ZephyrBuildConfig: DTS_ROOT = ${DTS_ROOT}")
message("ZephyrBuildConfig: ZEPHYR_SKIA_FIRMWARE_CMAKE_DIR = ${ZEPHYR_SKIA_FIRMWARE_CMAKE_DIR}")
