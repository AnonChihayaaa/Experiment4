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
CMAKE_SOURCE_DIR = /home/firefly/class/Experiment3/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/firefly/class/Experiment3/build

# Include any dependencies generated for this target.
include serial_ros/CMakeFiles/serial_ros_node.dir/depend.make

# Include the progress variables for this target.
include serial_ros/CMakeFiles/serial_ros_node.dir/progress.make

# Include the compile flags for this target's objects.
include serial_ros/CMakeFiles/serial_ros_node.dir/flags.make

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o: serial_ros/CMakeFiles/serial_ros_node.dir/flags.make
serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o: /home/firefly/class/Experiment3/src/serial_ros/src/serial_ros.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/firefly/class/Experiment3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o"
	cd /home/firefly/class/Experiment3/build/serial_ros && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o -c /home/firefly/class/Experiment3/src/serial_ros/src/serial_ros.cpp

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.i"
	cd /home/firefly/class/Experiment3/build/serial_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/firefly/class/Experiment3/src/serial_ros/src/serial_ros.cpp > CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.i

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.s"
	cd /home/firefly/class/Experiment3/build/serial_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/firefly/class/Experiment3/src/serial_ros/src/serial_ros.cpp -o CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.s

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o.requires:

.PHONY : serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o.requires

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o.provides: serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o.requires
	$(MAKE) -f serial_ros/CMakeFiles/serial_ros_node.dir/build.make serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o.provides.build
.PHONY : serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o.provides

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o.provides.build: serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o


serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o: serial_ros/CMakeFiles/serial_ros_node.dir/flags.make
serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o: /home/firefly/class/Experiment3/src/serial_ros/src/serial_ros_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/firefly/class/Experiment3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o"
	cd /home/firefly/class/Experiment3/build/serial_ros && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o -c /home/firefly/class/Experiment3/src/serial_ros/src/serial_ros_node.cpp

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.i"
	cd /home/firefly/class/Experiment3/build/serial_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/firefly/class/Experiment3/src/serial_ros/src/serial_ros_node.cpp > CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.i

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.s"
	cd /home/firefly/class/Experiment3/build/serial_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/firefly/class/Experiment3/src/serial_ros/src/serial_ros_node.cpp -o CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.s

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o.requires:

.PHONY : serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o.requires

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o.provides: serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o.requires
	$(MAKE) -f serial_ros/CMakeFiles/serial_ros_node.dir/build.make serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o.provides.build
.PHONY : serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o.provides

serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o.provides.build: serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o


# Object files for target serial_ros_node
serial_ros_node_OBJECTS = \
"CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o" \
"CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o"

# External object files for target serial_ros_node
serial_ros_node_EXTERNAL_OBJECTS =

/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: serial_ros/CMakeFiles/serial_ros_node.dir/build.make
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libserial.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libroslib.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/librospack.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libtf.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libtf2_ros.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libactionlib.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libmessage_filters.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libroscpp.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libtf2.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/librosconsole.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/librostime.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /opt/ros/melodic/lib/libcpp_common.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node: serial_ros/CMakeFiles/serial_ros_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/firefly/class/Experiment3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node"
	cd /home/firefly/class/Experiment3/build/serial_ros && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial_ros_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
serial_ros/CMakeFiles/serial_ros_node.dir/build: /home/firefly/class/Experiment3/devel/lib/serial_ros/serial_ros_node

.PHONY : serial_ros/CMakeFiles/serial_ros_node.dir/build

serial_ros/CMakeFiles/serial_ros_node.dir/requires: serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros.cpp.o.requires
serial_ros/CMakeFiles/serial_ros_node.dir/requires: serial_ros/CMakeFiles/serial_ros_node.dir/src/serial_ros_node.cpp.o.requires

.PHONY : serial_ros/CMakeFiles/serial_ros_node.dir/requires

serial_ros/CMakeFiles/serial_ros_node.dir/clean:
	cd /home/firefly/class/Experiment3/build/serial_ros && $(CMAKE_COMMAND) -P CMakeFiles/serial_ros_node.dir/cmake_clean.cmake
.PHONY : serial_ros/CMakeFiles/serial_ros_node.dir/clean

serial_ros/CMakeFiles/serial_ros_node.dir/depend:
	cd /home/firefly/class/Experiment3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/firefly/class/Experiment3/src /home/firefly/class/Experiment3/src/serial_ros /home/firefly/class/Experiment3/build /home/firefly/class/Experiment3/build/serial_ros /home/firefly/class/Experiment3/build/serial_ros/CMakeFiles/serial_ros_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial_ros/CMakeFiles/serial_ros_node.dir/depend

