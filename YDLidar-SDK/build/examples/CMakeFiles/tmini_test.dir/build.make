# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ubuntu/ros2_ws/src/YDLidar-SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/ros2_ws/src/YDLidar-SDK/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/tmini_test.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/tmini_test.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/tmini_test.dir/flags.make

examples/CMakeFiles/tmini_test.dir/tmini_test.cpp.o: examples/CMakeFiles/tmini_test.dir/flags.make
examples/CMakeFiles/tmini_test.dir/tmini_test.cpp.o: ../examples/tmini_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/ros2_ws/src/YDLidar-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/tmini_test.dir/tmini_test.cpp.o"
	cd /home/ubuntu/ros2_ws/src/YDLidar-SDK/build/examples && /bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tmini_test.dir/tmini_test.cpp.o -c /home/ubuntu/ros2_ws/src/YDLidar-SDK/examples/tmini_test.cpp

examples/CMakeFiles/tmini_test.dir/tmini_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tmini_test.dir/tmini_test.cpp.i"
	cd /home/ubuntu/ros2_ws/src/YDLidar-SDK/build/examples && /bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/ros2_ws/src/YDLidar-SDK/examples/tmini_test.cpp > CMakeFiles/tmini_test.dir/tmini_test.cpp.i

examples/CMakeFiles/tmini_test.dir/tmini_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tmini_test.dir/tmini_test.cpp.s"
	cd /home/ubuntu/ros2_ws/src/YDLidar-SDK/build/examples && /bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/ros2_ws/src/YDLidar-SDK/examples/tmini_test.cpp -o CMakeFiles/tmini_test.dir/tmini_test.cpp.s

# Object files for target tmini_test
tmini_test_OBJECTS = \
"CMakeFiles/tmini_test.dir/tmini_test.cpp.o"

# External object files for target tmini_test
tmini_test_EXTERNAL_OBJECTS =

tmini_test: examples/CMakeFiles/tmini_test.dir/tmini_test.cpp.o
tmini_test: examples/CMakeFiles/tmini_test.dir/build.make
tmini_test: libydlidar_sdk.a
tmini_test: examples/CMakeFiles/tmini_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/ros2_ws/src/YDLidar-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../tmini_test"
	cd /home/ubuntu/ros2_ws/src/YDLidar-SDK/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tmini_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/tmini_test.dir/build: tmini_test

.PHONY : examples/CMakeFiles/tmini_test.dir/build

examples/CMakeFiles/tmini_test.dir/clean:
	cd /home/ubuntu/ros2_ws/src/YDLidar-SDK/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/tmini_test.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/tmini_test.dir/clean

examples/CMakeFiles/tmini_test.dir/depend:
	cd /home/ubuntu/ros2_ws/src/YDLidar-SDK/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/ros2_ws/src/YDLidar-SDK /home/ubuntu/ros2_ws/src/YDLidar-SDK/examples /home/ubuntu/ros2_ws/src/YDLidar-SDK/build /home/ubuntu/ros2_ws/src/YDLidar-SDK/build/examples /home/ubuntu/ros2_ws/src/YDLidar-SDK/build/examples/CMakeFiles/tmini_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/tmini_test.dir/depend

