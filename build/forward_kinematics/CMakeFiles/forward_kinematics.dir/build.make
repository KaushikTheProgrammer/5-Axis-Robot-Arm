# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kaushik/Documents/RobotArm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaushik/Documents/RobotArm/build

# Include any dependencies generated for this target.
include forward_kinematics/CMakeFiles/forward_kinematics.dir/depend.make

# Include the progress variables for this target.
include forward_kinematics/CMakeFiles/forward_kinematics.dir/progress.make

# Include the compile flags for this target's objects.
include forward_kinematics/CMakeFiles/forward_kinematics.dir/flags.make

forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o: forward_kinematics/CMakeFiles/forward_kinematics.dir/flags.make
forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o: ../forward_kinematics/forward_kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaushik/Documents/RobotArm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o"
	cd /home/kaushik/Documents/RobotArm/build/forward_kinematics && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o -c /home/kaushik/Documents/RobotArm/forward_kinematics/forward_kinematics.cpp

forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.i"
	cd /home/kaushik/Documents/RobotArm/build/forward_kinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaushik/Documents/RobotArm/forward_kinematics/forward_kinematics.cpp > CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.i

forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.s"
	cd /home/kaushik/Documents/RobotArm/build/forward_kinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaushik/Documents/RobotArm/forward_kinematics/forward_kinematics.cpp -o CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.s

forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o.requires:

.PHONY : forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o.requires

forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o.provides: forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o.requires
	$(MAKE) -f forward_kinematics/CMakeFiles/forward_kinematics.dir/build.make forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o.provides.build
.PHONY : forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o.provides

forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o.provides.build: forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o


# Object files for target forward_kinematics
forward_kinematics_OBJECTS = \
"CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o"

# External object files for target forward_kinematics
forward_kinematics_EXTERNAL_OBJECTS =

forward_kinematics/forward_kinematics: forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o
forward_kinematics/forward_kinematics: forward_kinematics/CMakeFiles/forward_kinematics.dir/build.make
forward_kinematics/forward_kinematics: Stepper/libStepper.a
forward_kinematics/forward_kinematics: /usr/lib/libwiringPi.so
forward_kinematics/forward_kinematics: forward_kinematics/CMakeFiles/forward_kinematics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaushik/Documents/RobotArm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable forward_kinematics"
	cd /home/kaushik/Documents/RobotArm/build/forward_kinematics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/forward_kinematics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
forward_kinematics/CMakeFiles/forward_kinematics.dir/build: forward_kinematics/forward_kinematics

.PHONY : forward_kinematics/CMakeFiles/forward_kinematics.dir/build

forward_kinematics/CMakeFiles/forward_kinematics.dir/requires: forward_kinematics/CMakeFiles/forward_kinematics.dir/forward_kinematics.cpp.o.requires

.PHONY : forward_kinematics/CMakeFiles/forward_kinematics.dir/requires

forward_kinematics/CMakeFiles/forward_kinematics.dir/clean:
	cd /home/kaushik/Documents/RobotArm/build/forward_kinematics && $(CMAKE_COMMAND) -P CMakeFiles/forward_kinematics.dir/cmake_clean.cmake
.PHONY : forward_kinematics/CMakeFiles/forward_kinematics.dir/clean

forward_kinematics/CMakeFiles/forward_kinematics.dir/depend:
	cd /home/kaushik/Documents/RobotArm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaushik/Documents/RobotArm /home/kaushik/Documents/RobotArm/forward_kinematics /home/kaushik/Documents/RobotArm/build /home/kaushik/Documents/RobotArm/build/forward_kinematics /home/kaushik/Documents/RobotArm/build/forward_kinematics/CMakeFiles/forward_kinematics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : forward_kinematics/CMakeFiles/forward_kinematics.dir/depend

