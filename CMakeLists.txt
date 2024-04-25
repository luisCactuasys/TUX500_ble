cmake_minimum_required(VERSION 3.16)
project(bluez_inc CXX C)

#set(CMAKE_C_STANDARD 99)

# Show all the warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -Wall -Werror -Wextra -Wno-unused-function -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include(FindPkgConfig)
pkg_check_modules(GLIB glib-2.0 gio-2.0 REQUIRED)
include_directories(${GLIB_INCLUDE_DIRS})

add_subdirectory(binc)
add_subdirectory(ble_reader)
add_subdirectory(ArduinoJson)
add_subdirectory(common)
include_directories(ArduinoJson)
include_directories(common)
# include_directories(ArduinoJson/ArduinoJson)
# include_directories(ArduinoJson/ArduinoJson/Data)
# include_directories(ArduinoJson/ArduinoJson/Deserialization)
# include_directories(ArduinoJson/ArduinoJson/Serialization)
# include_directories(ArduinoJson/ArduinoJson/Serialization)
# include_directories(ArduinoJson/ArduinoJson/StringTraits)
# include_directories(ArduinoJson/ArduinoJson/TypeTraits)


add_custom_target(my_clean
    COMMAND git clean -d -f -x
)