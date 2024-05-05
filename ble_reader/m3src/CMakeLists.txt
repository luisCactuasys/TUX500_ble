add_library(M3ble
    aes.cpp
	M3Crc32.cpp
    )
    
target_include_directories(M3ble INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
