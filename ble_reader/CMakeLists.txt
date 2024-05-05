add_subdirectory(m3src)
include_directories(m3src)


add_executable(ble_reader.elf ble_reader.cpp)
target_link_libraries(ble_reader.elf Binc)
target_link_libraries(ble_reader.elf Common)
target_link_libraries(ble_reader.elf M3ble)

