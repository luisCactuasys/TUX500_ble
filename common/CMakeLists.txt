
add_library(Common
	tools.cpp
	base64.cpp
	json.cpp
    )

target_include_directories(Common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
