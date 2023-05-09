# list(APPEND BOARD_ROOT ${CMAKE_SOURCE_DIR})
# list(APPEND DTS_ROOT ${CMAKE_SOURCE_DIR})

zephyr_include_directories(include)

add_subdirectory(drivers)

message("*************** TEST zephyr/module: ZEPHYR_SKIA_FIRMWARE_CMAKE_DIR = ${ZEPHYR_SKIA_FIRMWARE_CMAKE_DIR}")
