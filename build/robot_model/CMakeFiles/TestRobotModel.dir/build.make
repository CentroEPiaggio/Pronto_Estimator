# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ros/docker_pronto_ws/src/robot/robot_model

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/docker_pronto_ws/build/robot_model

# Include any dependencies generated for this target.
include CMakeFiles/TestRobotModel.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/TestRobotModel.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/TestRobotModel.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TestRobotModel.dir/flags.make

CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o: CMakeFiles/TestRobotModel.dir/flags.make
CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o: /home/ros/docker_pronto_ws/src/robot/robot_model/test/test_robot_model.cpp
CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o: CMakeFiles/TestRobotModel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/docker_pronto_ws/build/robot_model/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o -MF CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o.d -o CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o -c /home/ros/docker_pronto_ws/src/robot/robot_model/test/test_robot_model.cpp

CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/docker_pronto_ws/src/robot/robot_model/test/test_robot_model.cpp > CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.i

CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/docker_pronto_ws/src/robot/robot_model/test/test_robot_model.cpp -o CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.s

# Object files for target TestRobotModel
TestRobotModel_OBJECTS = \
"CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o"

# External object files for target TestRobotModel
TestRobotModel_EXTERNAL_OBJECTS =

TestRobotModel: CMakeFiles/TestRobotModel.dir/test/test_robot_model.cpp.o
TestRobotModel: CMakeFiles/TestRobotModel.dir/build.make
TestRobotModel: librobot_model.so
TestRobotModel: /opt/ros/humble/lib/x86_64-linux-gnu/libpinocchio.so
TestRobotModel: /lib/x86_64-linux-gnu/libboost_filesystem.so
TestRobotModel: /lib/x86_64-linux-gnu/libboost_serialization.so
TestRobotModel: /lib/x86_64-linux-gnu/libboost_system.so
TestRobotModel: /home/ros/docker_pronto_ws/install/ryml/lib/libryml.so
TestRobotModel: /opt/ros/humble/lib/libament_index_cpp.so
TestRobotModel: CMakeFiles/TestRobotModel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/docker_pronto_ws/build/robot_model/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable TestRobotModel"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TestRobotModel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TestRobotModel.dir/build: TestRobotModel
.PHONY : CMakeFiles/TestRobotModel.dir/build

CMakeFiles/TestRobotModel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TestRobotModel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TestRobotModel.dir/clean

CMakeFiles/TestRobotModel.dir/depend:
	cd /home/ros/docker_pronto_ws/build/robot_model && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/docker_pronto_ws/src/robot/robot_model /home/ros/docker_pronto_ws/src/robot/robot_model /home/ros/docker_pronto_ws/build/robot_model /home/ros/docker_pronto_ws/build/robot_model /home/ros/docker_pronto_ws/build/robot_model/CMakeFiles/TestRobotModel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TestRobotModel.dir/depend
