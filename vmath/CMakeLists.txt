add_library (
	vmath
	src/vmath/vmath.cpp
	src/vmath/vmath.h
)

target_include_directories(vmath PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
