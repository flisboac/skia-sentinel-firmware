# This is needed to make Zephyr/West/CMake generate system calls,
# in `build/zephyr/include/generated/syscalls`
# list(APPEND SYSCALL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
# set(SYSCALL_INCLUDE_DIRS ${SYSCALL_INCLUDE_DIRS} PARENT_SCOPE)
zephyr_syscall_include_directories(include)

zephyr_include_directories(include)

add_subdirectory(drivers)
