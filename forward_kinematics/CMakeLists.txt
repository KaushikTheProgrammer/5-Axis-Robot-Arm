include_directories(${CURSES_INCLUDE_DIR})

find_library(wiringPi_LIB wiringPi)
find_package(Curses REQUIRED)

add_executable(forward_kinematics forward_kinematics.cpp)
add_executable(keyboard_control keyboard_control.cpp)

target_link_libraries(forward_kinematics PRIVATE ${wiringPi_LIB} Stepper pthread)
target_link_libraries(keyboard_control PRIVATE RobotAxis ${CURSES_LIBRARIES} -pthread)