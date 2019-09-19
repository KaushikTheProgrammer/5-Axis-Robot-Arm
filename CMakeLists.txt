cmake_minimum_required(VERSION 3.0)
project(RobotArm VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(Stepper)
add_subdirectory(forward_kinematics)
