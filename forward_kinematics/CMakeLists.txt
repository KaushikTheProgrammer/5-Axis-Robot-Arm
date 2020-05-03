add_executable(forward_kinematics forward_kinematics.cpp)

find_library(wiringPi_LIB wiringPi)
target_link_libraries(forward_kinematics PRIVATE ${wiringPi_LIB} Stepper pthread)
