cmake_minimum_required(VERSION 3.8)
project(bluez_inc CXX C)

#set(CMAKE_C_STANDARD 99)
if ("$ENV{CROSS_COMPILE}" STREQUAL "")
    set(ENV{SYSROOT} $ENV{OECORE_TARGET_SYSROOT})
endif()

# Show all the warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -Wall -Wextra -Wno-unused-function -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include(FindPkgConfig)
pkg_check_modules(GLIB glib-2.0 gio-2.0 REQUIRED)
include_directories(${GLIB_INCLUDE_DIRS})
if ("$ENV{CROSS_COMPILE}" STREQUAL "")
    include_directories($ENV{SYSROOT}/usr/include $ENV{SYSROOT}/usr/include/${TOOLCHAIN_MACHINE})
endif()

add_subdirectory(binc)
add_subdirectory(ble_reader)
add_subdirectory(ArduinoJson)
add_subdirectory(common)
include_directories(ArduinoJson)
include_directories(common)
include_directories(ble_reader)

# This is a simple application for message Q testing
add_subdirectory(test_app)

add_custom_target(my_clean
    COMMAND git clean -d -f -x
)