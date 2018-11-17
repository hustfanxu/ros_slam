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
include cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/depend.make

# Include the progress variables for this target.
include cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/progress.make

# Include the compile flags for this target's objects.
include cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/flags.make

cartographer/mapping/proto/sparse_pose_graph_options.pb.cc: ../cartographer/mapping/proto/sparse_pose_graph_options.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running C++ protocol buffer compiler on sparse_pose_graph_options.proto"
	cd /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto && /usr/bin/protoc --cpp_out /home/fan/ros_slam/cartographer/build -I /home/fan/ros_slam/cartographer /home/fan/ros_slam/cartographer/cartographer/mapping/proto/sparse_pose_graph_options.proto

cartographer/mapping/proto/sparse_pose_graph_options.pb.h: cartographer/mapping/proto/sparse_pose_graph_options.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate cartographer/mapping/proto/sparse_pose_graph_options.pb.h

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o: cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/flags.make
cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o: cartographer/mapping/proto/sparse_pose_graph_options.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o"
	cd /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o -c /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto/sparse_pose_graph_options.pb.cc

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.i"
	cd /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto/sparse_pose_graph_options.pb.cc > CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.i

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.s"
	cd /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto/sparse_pose_graph_options.pb.cc -o CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.s

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o.requires:

.PHONY : cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o.requires

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o.provides: cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o.requires
	$(MAKE) -f cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/build.make cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o.provides.build
.PHONY : cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o.provides

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o.provides.build: cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o


# Object files for target mapping_proto_sparse_pose_graph_options
mapping_proto_sparse_pose_graph_options_OBJECTS = \
"CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o"

# External object files for target mapping_proto_sparse_pose_graph_options
mapping_proto_sparse_pose_graph_options_EXTERNAL_OBJECTS =

cartographer/mapping/proto/libmapping_proto_sparse_pose_graph_options.a: cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o
cartographer/mapping/proto/libmapping_proto_sparse_pose_graph_options.a: cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/build.make
cartographer/mapping/proto/libmapping_proto_sparse_pose_graph_options.a: cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fan/ros_slam/cartographer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libmapping_proto_sparse_pose_graph_options.a"
	cd /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto && $(CMAKE_COMMAND) -P CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/cmake_clean_target.cmake
	cd /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/build: cartographer/mapping/proto/libmapping_proto_sparse_pose_graph_options.a

.PHONY : cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/build

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/requires: cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/sparse_pose_graph_options.pb.cc.o.requires

.PHONY : cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/requires

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/clean:
	cd /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto && $(CMAKE_COMMAND) -P CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/cmake_clean.cmake
.PHONY : cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/clean

cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/depend: cartographer/mapping/proto/sparse_pose_graph_options.pb.cc
cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/depend: cartographer/mapping/proto/sparse_pose_graph_options.pb.h
	cd /home/fan/ros_slam/cartographer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fan/ros_slam/cartographer /home/fan/ros_slam/cartographer/cartographer/mapping/proto /home/fan/ros_slam/cartographer/build /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto /home/fan/ros_slam/cartographer/build/cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cartographer/mapping/proto/CMakeFiles/mapping_proto_sparse_pose_graph_options.dir/depend

