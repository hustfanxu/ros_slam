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
include cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/depend.make

# Include the progress variables for this target.
include cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/progress.make

# Include the compile flags for this target's objects.
include cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/flags.make

cartographer/transform/proto/transform.pb.cc: ../cartographer/transform/proto/transform.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running C++ protocol buffer compiler on transform.proto"
	cd /home/fan/ros_slam/cartographer/build/cartographer/transform/proto && /usr/bin/protoc --cpp_out /home/fan/ros_slam/cartographer/build -I /home/fan/ros_slam/cartographer /home/fan/ros_slam/cartographer/cartographer/transform/proto/transform.proto

cartographer/transform/proto/transform.pb.h: cartographer/transform/proto/transform.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate cartographer/transform/proto/transform.pb.h

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o: cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/flags.make
cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o: cartographer/transform/proto/transform.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o"
	cd /home/fan/ros_slam/cartographer/build/cartographer/transform/proto && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o -c /home/fan/ros_slam/cartographer/build/cartographer/transform/proto/transform.pb.cc

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/transform_proto_transform.dir/transform.pb.cc.i"
	cd /home/fan/ros_slam/cartographer/build/cartographer/transform/proto && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fan/ros_slam/cartographer/build/cartographer/transform/proto/transform.pb.cc > CMakeFiles/transform_proto_transform.dir/transform.pb.cc.i

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/transform_proto_transform.dir/transform.pb.cc.s"
	cd /home/fan/ros_slam/cartographer/build/cartographer/transform/proto && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fan/ros_slam/cartographer/build/cartographer/transform/proto/transform.pb.cc -o CMakeFiles/transform_proto_transform.dir/transform.pb.cc.s

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o.requires:

.PHONY : cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o.requires

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o.provides: cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o.requires
	$(MAKE) -f cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/build.make cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o.provides.build
.PHONY : cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o.provides

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o.provides.build: cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o


# Object files for target transform_proto_transform
transform_proto_transform_OBJECTS = \
"CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o"

# External object files for target transform_proto_transform
transform_proto_transform_EXTERNAL_OBJECTS =

cartographer/transform/proto/libtransform_proto_transform.a: cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o
cartographer/transform/proto/libtransform_proto_transform.a: cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/build.make
cartographer/transform/proto/libtransform_proto_transform.a: cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libtransform_proto_transform.a"
	cd /home/fan/ros_slam/cartographer/build/cartographer/transform/proto && $(CMAKE_COMMAND) -P CMakeFiles/transform_proto_transform.dir/cmake_clean_target.cmake
	cd /home/fan/ros_slam/cartographer/build/cartographer/transform/proto && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/transform_proto_transform.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/build: cartographer/transform/proto/libtransform_proto_transform.a

.PHONY : cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/build

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/requires: cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/transform.pb.cc.o.requires

.PHONY : cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/requires

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/clean:
	cd /home/fan/ros_slam/cartographer/build/cartographer/transform/proto && $(CMAKE_COMMAND) -P CMakeFiles/transform_proto_transform.dir/cmake_clean.cmake
.PHONY : cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/clean

cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/depend: cartographer/transform/proto/transform.pb.cc
cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/depend: cartographer/transform/proto/transform.pb.h
	cd /home/fan/ros_slam/cartographer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fan/ros_slam/cartographer /home/fan/ros_slam/cartographer/cartographer/transform/proto /home/fan/ros_slam/cartographer/build /home/fan/ros_slam/cartographer/build/cartographer/transform/proto /home/fan/ros_slam/cartographer/build/cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cartographer/transform/proto/CMakeFiles/transform_proto_transform.dir/depend
