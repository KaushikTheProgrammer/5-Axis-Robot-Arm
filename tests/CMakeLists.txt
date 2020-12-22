find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

add_executable(stepper_test stepper_test.cpp)
#add_executable(axis_test axis_test.cpp)

target_link_libraries(stepper_test PRIVATE RobotAxis Stepper -lrt)
#target_link_libraries(axis_test PRIVATE RobotAxis ${CURSES_LIBRARIES} -pthread)

