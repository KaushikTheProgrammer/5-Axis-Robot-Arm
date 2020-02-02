add_library (
	vmath
	src/vmath/vmath.cpp
	src/vmath/vmath.hpp
)

find_library(wiringPi_LIB wiringPi)
target_link_libraries(vmath ${wiringPi_LIB})

target_include_directories(vmath PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
