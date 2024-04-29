add_library(M3ble
            aes.cpp
    )

target_include_directories(M3ble INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(ble_reader.elf ble_reader.cpp)
target_link_libraries(ble_reader.elf Binc)
target_link_libraries(ble_reader.elf Common)
target_link_libraries(ble_reader.elf M3ble)