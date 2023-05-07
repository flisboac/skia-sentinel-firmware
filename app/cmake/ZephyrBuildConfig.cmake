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

list(APPEND BOARD_ROOT ${APPLICATION_PROJECT_DIR})
list(APPEND DTS_ROOT ${APPLICATION_PROJECT_DIR})

message("ZephyrBuildConfig: ZEPHYR_BASE = ${ZEPHYR_BASE}")
message("ZephyrBuildConfig: PROJECT_ROOT = ${PROJECT_ROOT}")
message("ZephyrBuildConfig: APPLICATION_PROJECT_DIR = ${APPLICATION_PROJECT_DIR}")
message("ZephyrBuildConfig: BOARD_ROOT = ${BOARD_ROOT}")
message("ZephyrBuildConfig: DTS_ROOT = ${DTS_ROOT}")

# if(NOT ENV{ZEPHYR_TOOLCHAIN_VARIANT})
#     set(ZEPHYR_TOOLCHAIN_VARIANT gnuarmemb)
#     find_program(GNU_ARM_GCC arm-none-eabi-gcc)
#     if(NOT ${GNU_ARM_GCC} STREQUAL GNU_ARM_GCC-NOTFOUND)
#         # The toolchain root is located above the path to the compiler.
#         get_filename_component(GNUARMEMB_TOOLCHAIN_PATH ${GNU_ARM_GCC}/../.. ABSOLUTE)
#     endif()
# endif()
