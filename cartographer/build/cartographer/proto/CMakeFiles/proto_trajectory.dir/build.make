# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/fan/ros_slam/cartographer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fan/ros_slam/cartographer/build

# Include any dependencies generated for this target.
include cartographer/proto/CMakeFiles/proto_trajectory.dir/depend.make

# Include the progress variables for this target.
include cartographer/proto/CMakeFiles/proto_trajectory.dir/progress.make

# Include the compile flags for this target's objects.
include cartographer/proto/CMakeFiles/proto_trajectory.dir/flags.make

cartographer/proto/trajectory.pb.cc: ../cartographer/proto/trajectory.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running C++ protocol buffer compiler on trajectory.proto"
	cd /home/fan/ros_slam/cartographer/build/cartographer/proto && /usr/bin/protoc --cpp_out /home/fan/ros_slam/cartographer/build -I /home/fan/ros_slam/cartographer /home/fan/ros_slam/cartographer/cartographer/proto/trajectory.proto

cartographer/proto/trajectory.pb.h: cartographer/proto/trajectory.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate cartographer/proto/trajectory.pb.h

cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o: cartographer/proto/CMakeFiles/proto_trajectory.dir/flags.make
cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o: cartographer/proto/trajectory.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o"
	cd /home/fan/ros_slam/cartographer/build/cartographer/proto && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o -c /home/fan/ros_slam/cartographer/build/cartographer/proto/trajectory.pb.cc

cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.i"
	cd /home/fan/ros_slam/cartographer/build/cartographer/proto && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fan/ros_slam/cartographer/build/cartographer/proto/trajectory.pb.cc > CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.i

cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.s"
	cd /home/fan/ros_slam/cartographer/build/cartographer/proto && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fan/ros_slam/cartographer/build/cartographer/proto/trajectory.pb.cc -o CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.s

cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o.requires:

.PHONY : cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o.requires

cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o.provides: cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o.requires
	$(MAKE) -f cartographer/proto/CMakeFiles/proto_trajectory.dir/build.make cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o.provides.build
.PHONY : cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o.provides

cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o.provides.build: cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o


# Object files for target proto_trajectory
proto_trajectory_OBJECTS = \
"CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o"

# External object files for target proto_trajectory
proto_trajectory_EXTERNAL_OBJECTS =

cartographer/proto/libproto_trajectory.a: cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o
cartographer/proto/libproto_trajectory.a: cartographer/proto/CMakeFiles/proto_trajectory.dir/build.make
cartographer/proto/libproto_trajectory.a: cartographer/proto/CMakeFiles/proto_trajectory.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libproto_trajectory.a"
	cd /home/fan/ros_slam/cartographer/build/cartographer/proto && $(CMAKE_COMMAND) -P CMakeFiles/proto_trajectory.dir/cmake_clean_target.cmake
	cd /home/fan/ros_slam/cartographer/build/cartographer/proto && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/proto_trajectory.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cartographer/proto/CMakeFiles/proto_trajectory.dir/build: cartographer/proto/libproto_trajectory.a

.PHONY : cartographer/proto/CMakeFiles/proto_trajectory.dir/build

cartographer/proto/CMakeFiles/proto_trajectory.dir/requires: cartographer/proto/CMakeFiles/proto_trajectory.dir/trajectory.pb.cc.o.requires

.PHONY : cartographer/proto/CMakeFiles/proto_trajectory.dir/requires

cartographer/proto/CMakeFiles/proto_trajectory.dir/clean:
	cd /home/fan/ros_slam/cartographer/build/cartographer/proto && $(CMAKE_COMMAND) -P CMakeFiles/proto_trajectory.dir/cmake_clean.cmake
.PHONY : cartographer/proto/CMakeFiles/proto_trajectory.dir/clean

cartographer/proto/CMakeFiles/proto_trajectory.dir/depend: cartographer/proto/trajectory.pb.cc
cartographer/proto/CMakeFiles/proto_trajectory.dir/depend: cartographer/proto/trajectory.pb.h
	cd /home/fan/ros_slam/cartographer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fan/ros_slam/cartographer /home/fan/ros_slam/cartographer/cartographer/proto /home/fan/ros_slam/cartographer/build /home/fan/ros_slam/cartographer/build/cartographer/proto /home/fan/ros_slam/cartographer/build/cartographer/proto/CMakeFiles/proto_trajectory.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cartographer/proto/CMakeFiles/proto_trajectory.dir/depend

