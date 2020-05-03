add_library (
	RobotAxis
	RobotAxis.cpp
	RobotAxis.hpp
)

target_link_libraries(RobotAxis Stepper)

target_include_directories(RobotAxis PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")