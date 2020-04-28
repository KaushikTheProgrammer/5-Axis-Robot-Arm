add_executable(limitSwitchTest limitSwitches.cpp)
add_executable(axis_test axis_test.cpp)

target_link_libraries(limitSwitchTest PRIVATE wiringPi)
target_link_libraries(axis_test PRIVATE Stepper pthread)