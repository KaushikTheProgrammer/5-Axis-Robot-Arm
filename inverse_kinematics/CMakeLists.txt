add_executable(inverse_kinematics inverse_kinematics.cpp)

find_library(wiringPi_LIB wiringPi)
target_link_libraries(inverse_kinematics PRIVATE ${wiringPi_LIB} RobotAxis vmath pthread)
